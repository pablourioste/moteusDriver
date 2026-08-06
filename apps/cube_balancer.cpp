// cube_balancer -- the 200 Hz reaction wheel balancing loop.
//
// Combines IImuSensor and IMotorDriver through StateEstimator and
// BalancingController.  Everything hardware-specific lives behind those two
// interfaces; this file is the real-time loop and the safety plumbing.
//
// SAFETY: this is the only executable that commands torque from a feedback
// law.  Read the checklist printed at startup before running it.  The gains
// in BalancingController are placeholders and will not balance the cube --
// see docs/3d_scaling/README.md.

#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iostream>
#include <memory>
#include <string>

#include <unistd.h>

#include "core/BalancingController.hpp"
#include "core/StateEstimator.hpp"
#include "core/Types.hpp"
#include "drivers/ImuDriver.hpp"
#include "drivers/MoteusConfig.hpp"
#include "drivers/MoteusDriverWrapper.hpp"

namespace {

// Flipped by the signal handler; the control loop polls it and exits.  The
// handler must stay async-signal-safe, so it does nothing but set this and
// command a stop -- no printing, no allocation.
std::atomic<bool> g_shutdown{false};

// Non-owning pointer to the driver the handler stops.  Raw rather than a
// smart pointer because the handler cannot safely touch a refcount.
cube::IMotorDriver* g_driver = nullptr;

void HandleSignal(int) {
  g_shutdown.store(true);
  // Without this, Ctrl-C leaves the board holding its last command and
  // drawing current indefinitely.  This is the whole point of the handler.
  if (g_driver) { g_driver->stop(); }
}

constexpr double kRadToDeg = 57.29577951308232;

double MonotonicSeconds() {
  struct timespec ts = {};
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) +
         static_cast<double>(ts.tv_nsec) / 1e9;
}

void PrintUsage(const char* argv0) {
  std::cout
      << "Usage: " << argv0 << " [options]\n\n"
      << "  --config PATH       moteus config file to push\n"
      << "  --i2c PATH          IMU I2C device (default /dev/i2c-1)\n"
      << "  --rate HZ           control rate (default 200)\n"
      << "  --max-torque NM     torque clamp (default from build)\n"
      << "  --k-theta G         angle gain override\n"
      << "  --k-theta-dot G     rate gain override\n"
      << "  --k-omega G         wheel speed gain override\n"
      << "  --theta-offset RAD  balance-point offset (from imu_test)\n"
      << "  --gain-scale G      scale the whole control law, [0,1] "
         "(default 1.0)\n"
      << "                      use e.g. 0.1 for a first closed-loop "
         "attempt\n"
      << "  --dry-run           run the loop but never command torque\n"
      << "  --help\n";
}

}  // namespace

int main(int argc, char** argv) {
  auto moteus_config = cube::MoteusConfig::FromBuildDefaults();
  // Carries the measured gyro bias and accel offset/scale from the CMake
  // cache.  A default-constructed Config would be the identity transform --
  // an uncalibrated sensor feeding the balancing law.
  auto imu_config = cube::ImuDriver::Config::FromBuildDefaults();
  cube::BalancingController::Config control_config;
  cube::StateEstimator::Config estimator_config;

  // Seed the IMU from the build-time defaults so -DIMU_I2C_DEVICE at
  // configure time actually takes effect; --i2c still overrides per run.
#ifdef DEFAULT_IMU_I2C_DEVICE
  imu_config.i2c_device = DEFAULT_IMU_I2C_DEVICE;
#endif
#ifdef DEFAULT_IMU_I2C_ADDRESS
  imu_config.i2c_address = static_cast<std::uint8_t>(DEFAULT_IMU_I2C_ADDRESS);
#endif

  double rate_hz = 200.0;
  bool dry_run = false;
  double gain_scale = 1.0;

  for (int i = 1; i < argc; i++) {
    const std::string arg = argv[i];
    const bool has_value = (i + 1 < argc);
    if (arg == "--help") {
      PrintUsage(argv[0]);
      return 0;
    } else if (arg == "--dry-run") {
      dry_run = true;
    } else if (arg == "--config" && has_value) {
      moteus_config.config_path = argv[++i];
    } else if (arg == "--i2c" && has_value) {
      imu_config.i2c_device = argv[++i];
    } else if (arg == "--rate" && has_value) {
      rate_hz = std::atof(argv[++i]);
    } else if (arg == "--max-torque" && has_value) {
      control_config.max_torque_nm = std::atof(argv[++i]);
    } else if (arg == "--k-theta" && has_value) {
      control_config.k_theta = std::atof(argv[++i]);
    } else if (arg == "--k-theta-dot" && has_value) {
      control_config.k_theta_dot = std::atof(argv[++i]);
    } else if (arg == "--k-omega" && has_value) {
      control_config.k_omega = std::atof(argv[++i]);
    } else if (arg == "--theta-offset" && has_value) {
      estimator_config.theta_offset = std::atof(argv[++i]);
    } else if (arg == "--gain-scale" && has_value) {
      gain_scale = std::atof(argv[++i]);
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      PrintUsage(argv[0]);
      return 1;
    }
  }

  if (!(rate_hz > 0.0) || rate_hz > 1000.0) {
    std::cerr << "Invalid rate " << rate_hz << " Hz\n";
    return 1;
  }
  const double period = 1.0 / rate_hz;
  const long period_us = static_cast<long>(period * 1e6);

  // The estimator's reject-bad-dt gate is relative to this, not a fixed
  // constant, so it has to track whatever rate the loop actually runs at.
  estimator_config.nominal_dt = period;

  // BalancingController::update() trusts gain_scale as given -- validating
  // it is this call site's job, since it is the source.  Written as
  // !(x >= 0) rather than (x < 0) so a NaN from a malformed --gain-scale
  // argument is also rejected: every comparison against NaN is false, so
  // the naive form would let it straight through.
  if (!(gain_scale >= 0.0) || !(gain_scale <= 1.0)) {
    std::cerr << "Invalid --gain-scale " << gain_scale
              << " (must be in [0, 1])\n";
    return 1;
  }

  // ---------------------------------------------------------------------
  // Refuse to run on unset configuration.
  //
  // The control gains and filter parameters ship as NaN sentinels because
  // they cannot be guessed without measuring the rig.  This check exists
  // because NaN does NOT fail loudly on its own: every comparison against a
  // NaN is false, so `fabs(theta) > max_tilt_rad` would silently never
  // trip, and the e-stop would be gone.  Better to stop here with a name
  // than to run with a disabled safety envelope.
  // ---------------------------------------------------------------------
  {
    cube::BalancingController probe_controller(control_config);
    cube::StateEstimator probe_estimator(estimator_config);

    const char* unset_gain = probe_controller.firstUnsetField();
    const char* unset_filter = probe_estimator.firstUnsetField();

    if (unset_gain != nullptr || unset_filter != nullptr) {
      std::cerr
          << "Refusing to run: the control configuration is incomplete.\n\n";
      if (unset_filter != nullptr) {
        std::cerr << "  StateEstimator::Config::" << unset_filter
                  << " is unset.\n";
      }
      if (unset_gain != nullptr) {
        std::cerr << "  BalancingController::Config::" << unset_gain
                  << " is unset.\n";
      }
      std::cerr
          << "\nThese are deliberately left NaN -- they cannot be guessed "
             "without\n"
             "measuring the rig.  See:\n"
             "  docs/3d_scaling/README.md section 3  measuring inertia, mass, "
             "CoM\n"
             "  docs/3d_scaling/README.md section 4  computing the LQR gains\n"
             "  include/core/StateEstimator.hpp      per-field guidance\n"
             "  include/core/BalancingController.hpp per-gain guidance\n\n"
             "Set them in the Config structs, or pass --k-theta / "
             "--k-theta-dot /\n--k-omega / --max-torque / --theta-offset on "
             "the command line.\n";
      return 1;
    }
  }

  // The driver's clamp must not be tighter than the control law's, or the
  // law would silently never reach its own limit.
  if (control_config.max_torque_nm > moteus_config.max_torque_nm) {
    moteus_config.max_torque_nm = control_config.max_torque_nm;
  }

  // The watchdog has to outlast normal jitter but still stop the board
  // promptly if this process dies.  20 cycles is the same margin the
  // existing driver uses at 100 Hz.
  moteus_config.watchdog_timeout_s = 20.0 * period;

  std::cout
      << "=== Cube balancer ===\n"
      << "Rate:        " << rate_hz << " Hz (" << period_us << " us)\n"
      << "Max torque:  " << control_config.max_torque_nm << " Nm\n"
      << "Watchdog:    " << moteus_config.watchdog_timeout_s << " s\n"
      << "E-stop tilt: " << control_config.max_tilt_rad * kRadToDeg
      << " deg\n"
      << "Gains:       k_theta=" << control_config.k_theta
      << "  k_theta_dot=" << control_config.k_theta_dot
      << "  k_omega=" << control_config.k_omega << "\n"
      << "Gain scale:  " << gain_scale
      << (gain_scale < 1.0 ? "  (SOFTENED -- not the tuned response)\n"
                            : "\n")
      << (dry_run ? "Mode:        DRY RUN (no torque commanded)\n" : "")
      << "\n";

  if (!dry_run) {
    std::cout
        << "  !! The default gains are PLACEHOLDERS and will not balance the\n"
        << "  !! cube.  Confirm before running:\n"
        << "  !!   * imu_test passes and theta reads ~0 when balanced\n"
        << "  !!   * direct_motor_test passes, including the watchdog check\n"
        << "  !!   * the cube is on a soft surface with nothing in its arc\n"
        << "  !!   * you can reach the power cut\n\n";
  }

  cube::ImuDriver imu(imu_config);
  cube::MoteusDriverWrapper motor(moteus_config);
  cube::StateEstimator estimator(estimator_config);
  cube::BalancingController controller(control_config);

  std::string error;

  std::cout << "Initialising IMU... " << std::flush;
  if (!imu.initialize(&error)) {
    std::cout << "FAIL\n\n" << error << "\n";
    return 1;
  }
  std::cout << "OK\n";

  std::cout << "Measuring gyro bias -- HOLD STILL... " << std::flush;
  if (!imu.calibrateGyroBias(2.0, &error)) {
    std::cout << "FAIL\n" << error << "\n";
    return 1;
  }
  std::cout << "OK\n";

  std::cout << "Initialising motor... " << std::flush;
  if (!motor.initialize(&error)) {
    std::cout << "FAIL\n\n" << error << "\n";
    return 1;
  }
  std::cout << "OK\n";

  // Installed only after both devices are up, but before anything can
  // command motion, so there is no window where Ctrl-C would leave the motor
  // running.
  g_driver = &motor;
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  std::cout << "\nBalancing.  Ctrl-C to stop.\n\n";

  double last = MonotonicSeconds();
  double next_deadline = last + period;

  long cycle = 0;
  long late_cycles = 0;
  double worst_jitter = 0.0;

  while (!g_shutdown.load()) {
    const double now = MonotonicSeconds();
    const double dt = now - last;
    last = now;

    const cube::ImuData sample = imu.read();

    // Query first, then command.  The wheel speed this returns feeds the
    // k_omega term, so the control law acts on the state it just measured
    // rather than on the previous cycle's.
    const cube::MotorState motor_state = motor.query();

    const cube::BodyState body = estimator.update(sample, motor_state, dt);
    const cube::ControlOutput out =
        controller.update(body, motor_state, gain_scale);

    if (!dry_run && out.armed) {
      motor.sendTorque(out.torque_nm);
    } else if (!out.armed) {
      // Latched safety trip: stop the board and leave it stopped.
      motor.stop();
    }

    if (cycle % 40 == 0) {  // ~5 Hz of output
      std::printf(
          "\r  theta %+6.2f deg  rate %+7.2f deg/s  wheel %+8.1f rad/s  "
          "tau %+6.3f Nm  %-15s ",
          body.theta * kRadToDeg, body.theta_dot * kRadToDeg,
          body.wheel_omega, out.torque_nm, cube::ToString(out.safety));
      std::fflush(stdout);
    }

    // A latched trip is terminal: report it once and leave the loop rather
    // than spinning at 200 Hz doing nothing.
    if (!out.armed) {
      std::printf("\n\n  SAFETY TRIP: %s\n", cube::ToString(out.safety));
      if (out.safety == cube::SafetyState::kTiltExceeded) {
        std::printf("  Tilt %.2f deg exceeded the %.2f deg limit.\n",
                    body.theta * kRadToDeg,
                    control_config.max_tilt_rad * kRadToDeg);
      } else if (out.safety == cube::SafetyState::kMotorFault) {
        std::printf("  Controller fault code %d.\n", motor_state.fault);
      }
      break;
    }

    cycle++;

    // Absolute deadlines rather than a fixed sleep: sleeping for the period
    // accumulates every cycle's overrun into permanent drift, which at
    // 200 Hz turns into a real change in the effective control rate.
    next_deadline += period;
    const double slack = next_deadline - MonotonicSeconds();
    if (slack > 0.0) {
      ::usleep(static_cast<useconds_t>(slack * 1e6));
    } else {
      late_cycles++;
      if (-slack > worst_jitter) { worst_jitter = -slack; }
      // Missed the deadline badly enough that catching up would mean a burst
      // of back-to-back cycles; resynchronise instead.
      if (-slack > period) { next_deadline = MonotonicSeconds() + period; }
    }
  }

  // Reached on clean exit, on fault, and on a safety trip.  The signal
  // handler has already sent this on Ctrl-C, but a second stop is harmless
  // and covers the other paths.
  motor.stop();

  std::printf("\nStopped after %ld cycles.\n", cycle);
  if (cycle > 0) {
    std::printf("Late cycles: %ld (%.2f%%), worst overrun %.2f ms\n",
                late_cycles, 100.0 * late_cycles / cycle,
                worst_jitter * 1000.0);
    if (late_cycles > cycle / 20) {
      std::printf(
          "More than 5%% of cycles were late.  The control rate is not being "
          "met;\nconsider a lower --rate, or run with a real-time scheduling "
          "policy\n(chrt -f 50 ...).\n");
    }
  }

  return 0;
}
