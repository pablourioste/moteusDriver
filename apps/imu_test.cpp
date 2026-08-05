// imu_test -- validate and calibrate the BMI270 with no motor involved.
//
// Run this before ever enabling the motor.  It answers the three questions
// that decide whether the balancer can work at all:
//
//   1. Does the IMU respond and produce sane data?
//   2. What is the gyro zero-rate bias?
//   3. Which way is "positive tilt", and what is theta at the balance point?
//
// Question 3 is the one that bites: a sign error in the axis mapping makes
// the controller drive the cube over instead of catching it.  This tool
// prints the live angle so the mapping can be confirmed by hand.

#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

#include <unistd.h>

#include "core/StateEstimator.hpp"
#include "core/Types.hpp"
#include "drivers/ImuDriver.hpp"

namespace {

std::atomic<bool> g_shutdown{false};

void HandleSignal(int) { g_shutdown.store(true); }

constexpr double kRadToDeg = 57.29577951308232;
constexpr double kGravity = 9.80665;

void PrintUsage(const char* argv0) {
  std::cout
      << "Usage: " << argv0 << " [options]\n\n"
      << "  --device PATH     I2C device node (default /dev/i2c-1)\n"
      << "  --address ADDR    I2C address in hex, 68 or 69 (default 68)\n"
      << "  --bias-seconds S  gyro bias averaging window (default 3.0)\n"
      << "  --stream          skip the checks, just stream angle live\n"
      << "  --help\n";
}

}  // namespace

int main(int argc, char** argv) {
  // FromBuildDefaults() rather than a plain Config: it carries the measured
  // gyro bias and accel offset/scale from the CMake cache.  A
  // default-constructed Config is the identity transform, i.e. an
  // UNcalibrated sensor -- which for this program would mean reporting a
  // tilt that is quietly wrong by whatever the offsets are.
  auto config = cube::ImuDriver::Config::FromBuildDefaults();

  // Seed from the build-time defaults so -DIMU_I2C_DEVICE at configure time
  // actually takes effect; --device/--address still override per run.
#ifdef DEFAULT_IMU_I2C_DEVICE
  config.i2c_device = DEFAULT_IMU_I2C_DEVICE;
#endif
#ifdef DEFAULT_IMU_I2C_ADDRESS
  config.i2c_address = static_cast<std::uint8_t>(DEFAULT_IMU_I2C_ADDRESS);
#endif

  // Left at its scaffold defaults: this tool exists partly to MEASURE the
  // values that populate it (theta_offset, the axis mapping, the noise floor
  // that sets accel_gate and theta_deadband).
  cube::StateEstimator::Config estimator_config;

  double bias_seconds = 3.0;
  bool stream_only = false;

  for (int i = 1; i < argc; i++) {
    const std::string arg = argv[i];
    if (arg == "--help") {
      PrintUsage(argv[0]);
      return 0;
    } else if (arg == "--stream") {
      stream_only = true;
    } else if (arg == "--device" && i + 1 < argc) {
      config.i2c_device = argv[++i];
    } else if (arg == "--address" && i + 1 < argc) {
      config.i2c_address =
          static_cast<std::uint8_t>(std::strtol(argv[++i], nullptr, 16));
    } else if (arg == "--bias-seconds" && i + 1 < argc) {
      bias_seconds = std::atof(argv[++i]);
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      PrintUsage(argv[0]);
      return 1;
    }
  }

  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  cube::ImuDriver imu(config);

  std::cout << "=== BMI270 IMU test ===\n"
            << "Device:  " << config.i2c_device << "\n"
            << "Address: 0x" << std::hex
            << static_cast<int>(config.i2c_address) << std::dec << "\n\n";

  std::string error;

  // --- 1. Initialise ------------------------------------------------------
  std::cout << "[1/4] Initialising... " << std::flush;
  if (!imu.initialize(&error)) {
    std::cout << "FAIL\n\n" << error << "\n";
    return 1;
  }
  std::cout << "OK (chip ID verified, config blob loaded)\n";

  if (stream_only) {
    std::cout << "\nStreaming.  Ctrl-C to stop.\n";
  }

  // --- 2. Sanity check the raw data --------------------------------------
  if (!stream_only) {
    std::cout << "[2/4] Reading samples... " << std::flush;

    cube::ImuData sample;
    int valid = 0;
    double accel_magnitude_sum = 0.0;
    for (int i = 0; i < 100; i++) {
      sample = imu.read();
      if (sample.valid) {
        valid++;
        accel_magnitude_sum += std::sqrt(
            sample.accel_x * sample.accel_x +
            sample.accel_y * sample.accel_y +
            sample.accel_z * sample.accel_z);
      }
      ::usleep(2500);
    }

    if (valid == 0) {
      std::cout << "FAIL\nNo valid samples.\n";
      return 1;
    }

    const double mean_magnitude = accel_magnitude_sum / valid;
    std::cout << valid << "/100 valid\n";

    std::printf("      accel  x=%+8.3f  y=%+8.3f  z=%+8.3f  m/s^2\n",
                sample.accel_x, sample.accel_y, sample.accel_z);
    std::printf("      gyro   x=%+8.4f  y=%+8.4f  z=%+8.4f  rad/s\n",
                sample.gyro_x, sample.gyro_y, sample.gyro_z);
    std::printf("      temp   %.1f C\n", sample.temperature_c);
    std::printf("      |accel| = %.3f m/s^2 (expect ~%.3f at rest)\n",
                mean_magnitude, kGravity);

    // At rest the accelerometer must measure exactly one g.  Anything else
    // means a wrong range setting, a bad scale factor, or a moving rig --
    // and every downstream angle would be wrong.
    if (std::fabs(mean_magnitude - kGravity) > 0.15 * kGravity) {
      std::cout << "      WARNING: magnitude is not ~1 g.  Either the rig is "
                   "moving, or\n               the range/scale configuration "
                   "is wrong.  Do not trust\n               the angle below "
                   "until this reads ~9.8.\n";
    }

    // --- 3. Gyro bias -----------------------------------------------------
    std::cout << "[3/4] Measuring gyro bias over " << bias_seconds
              << " s -- HOLD THE RIG STILL... " << std::flush;

    if (!imu.calibrateGyroBias(bias_seconds, &error)) {
      std::cout << "FAIL\n      " << error << "\n";
      return 1;
    }

    const auto& c = imu.config();
    std::cout << "OK\n";
    std::printf("      bias   x=%+9.6f  y=%+9.6f  z=%+9.6f  rad/s\n",
                c.gyro_bias_x, c.gyro_bias_y, c.gyro_bias_z);
    std::printf("             (%+.3f, %+.3f, %+.3f deg/s)\n",
                c.gyro_bias_x * kRadToDeg, c.gyro_bias_y * kRadToDeg,
                c.gyro_bias_z * kRadToDeg);
    std::cout << "\n      Copy these into StateEstimator's config, or re-run "
                 "this at startup.\n";
  }

  // --- 4. Live inclination -----------------------------------------------
  std::cout << "\n[4/4] Live tilt.  Ctrl-C to stop.\n\n"
            << "  Verify BEFORE enabling the motor:\n"
            << "    * theta reads ~0 at the balance point (else set "
               "theta_offset)\n"
            << "    * tilting one way gives a consistent sign\n"
            << "    * theta_dot has the SAME sign as theta while falling that "
               "way\n"
            << "      (if not, fix gyro_axis or invert_theta -- a mismatch "
               "makes the\n"
            << "       controller amplify the fall instead of catching it)\n\n";

  // StateEstimator is a scaffold until you implement it (see
  // src/core/StateEstimator.cpp).  Raw-sensor validation -- the part that
  // does not depend on the filter -- still works and is printed either way;
  // the filtered columns only appear once the estimator is both implemented
  // and configured.
  cube::StateEstimator estimator(estimator_config);
  const bool estimator_ready = estimator.isConfigured();

  if (!estimator_ready) {
    std::printf(
        "  NOTE: StateEstimator is unconfigured (first unset field: %s), so\n"
        "  the filtered theta/theta_dot columns are omitted.  Raw sensor and\n"
        "  accelerometer-only angle are shown -- those are enough to check\n"
        "  wiring, noise and the accelerometer sign.\n\n",
        estimator.firstUnsetField());
  }

  cube::MotorState no_motor;  // valid=false: no wheel term in this tool

  double last = 0.0;
  {
    struct timespec ts = {};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    last = ts.tv_sec + ts.tv_nsec / 1e9;
  }

  int row = 0;
  while (!g_shutdown.load()) {
    const cube::ImuData sample = imu.read();

    struct timespec ts = {};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    const double now = ts.tv_sec + ts.tv_nsec / 1e9;
    const double dt = now - last;
    last = now;

    const cube::BodyState state = estimator.update(sample, no_motor, dt);

    if (row++ % 20 == 0) {  // ~10 Hz of output from a 200 Hz read
      if (estimator_ready) {
        std::printf(
            "\r  theta %+7.2f deg   theta_dot %+7.2f deg/s   accel-only "
            "%+7.2f deg   %s   ",
            state.theta * kRadToDeg, state.theta_dot * kRadToDeg,
            estimator.accelAngle(sample) * kRadToDeg,
            state.valid ? "ok " : "...");
      } else {
        // Raw gyro rate about each axis plus the accelerometer vector: the
        // measurements you need to pick gyro_axis/accel_axis_* and to read
        // theta_offset off a hand-balanced cube.
        std::printf(
            "\r  accel %+6.2f %+6.2f %+6.2f m/s^2   gyro %+7.3f %+7.3f "
            "%+7.3f rad/s   ",
            sample.accel_x, sample.accel_y, sample.accel_z,
            sample.gyro_x, sample.gyro_y, sample.gyro_z);
      }
      std::fflush(stdout);
    }

    ::usleep(5000);  // 200 Hz, matching the balancer
  }

  std::cout << "\n\nStopped.\n";
  return 0;
}
