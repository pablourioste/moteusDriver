// cube_balancer -- INTEGRATION.md S6, then N1/N2.  The real control loop.
//
// The Teensy port of apps/cube_balancer.cpp.  Same call sequence, same
// safety ordering, same absolute-deadline scheduler; the CLI shell, the
// signal handlers and the printf reporting are what fall away.
//
//   BMI270 --> StateEstimator --> BalancingController --> moteus
//
// core/ is compiled from src/core/ in place, exactly as the host build
// compiles it.  Nothing in this file is a copy of the control law -- a
// copy is how a simulation and a rig quietly stop agreeing.
//
// ---------------------------------------------------------------------
// SAFETY -- read before flashing
// ---------------------------------------------------------------------
//
// This is the only firmware that runs the feedback law.  It commands
// torque from an estimate, so a sign error here drives the cube into its
// fall at full authority rather than catching it.
//
//   S6  torque DISARMED (default).  Proves the loop holds 200 Hz and the
//       NaN gate survived the port.  Wheel off, motor power off.
//   N1  -D ENABLE_BALANCER_TORQUE, WHEEL STILL OFF.  First motor power.
//       Tilt by hand, confirm the shaft pushes the right way.
//   N2  wheel on, tethered, soft surface, ~30% gains.
//
// Like moteus_driver_test, the torque path is a separate build rather
// than a runtime toggle: without the flag, the call that commands torque
// is not in the binary.
//
// ---------------------------------------------------------------------
// WHY THIS REFUSES TO RUN
// ---------------------------------------------------------------------
//
// The gains and filter constants ship as NaN sentinels because they
// cannot be guessed without measuring the rig.  This is a safety
// property, not a placeholder convention: every comparison against NaN is
// false, so an unset max_tilt_rad would not fail loudly -- the tilt
// e-stop would silently cease to exist.  The loop therefore checks
// firstUnsetField() before constructing anything that can move the motor
// and halts naming the field.
//
// Fill them in via build_flags once docs/3d_scaling/README.md sections 3
// and 4 have produced measurements and LQR gains.

#include <Arduino.h>

#include <cmath>

#include "core/BalancingController.hpp"
#include "core/StateEstimator.hpp"
#include "core/TelemetryFrame.hpp"
#include "core/TelemetryRing.hpp"
#include "core/TelemetrySink.hpp"
#include "core/Types.hpp"
#include "embedded/Bmi270SpiDriver.hpp"
#include "embedded/TeensyClock.hpp"
#include "embedded/TeensyMoteusDriver.hpp"

namespace {

// --- rate -------------------------------------------------------------
constexpr double kControlHz = 200.0;
constexpr double kPeriod = 1.0 / kControlHz;
constexpr uint32_t kPeriodUs = static_cast<uint32_t>(1e6 / kControlHz);

// Board stops itself if it hears nothing for this long.  20 cycles is the
// same margin the host driver uses: long enough to absorb jitter, short
// enough that a dead host is caught in 100 ms.  THIS IS THE ONLY THING
// that stops the wheel if the Teensy hangs -- there is no OS to notice.
constexpr double kWatchdogS = 20.0 * kPeriod;

// --- physical e-stop (H5) ---------------------------------------------
//
// Normally-closed button between this pin and GND, read EVERY cycle.
// Closed (LOW) is the run state; opening it must stop the motor.
//
// The host has SIGINT -> stop().  There is no MCU equivalent: no OS, no
// signals, no way to interrupt a hung loop from a keyboard.  A
// software-only stop on a bare-metal device is not a stop.
//
// Wired normally-CLOSED deliberately, so that a cut wire or a pulled
// connector reads the same as a pressed button -- fail-safe rather than
// fail-silent.
constexpr int kEstopPin = 2;

// --- telemetry --------------------------------------------------------
constexpr uint32_t kTelemetryDecimation = 1;   // every cycle; raise if tight

cube::TelemetryRing<64> g_ring;

class UsbCdcSink : public cube::ITelemetrySink {
 public:
  // availableForWrite() is what makes this safe: it reports what the
  // endpoint will take right now, so the drain can decline instead of
  // blocking.  A blocking Serial.write() when the host stops reading
  // would stall the control loop -- which on a balancing rig is a fall,
  // not a lost log.
  int space() override { return Serial.availableForWrite(); }

  size_t write(const uint8_t* data, size_t length) override {
    return Serial.write(data, length);
  }

  const char* name() const override { return "usb_cdc"; }
};

UsbCdcSink g_sink;

// --- devices ----------------------------------------------------------
cube::Bmi270SpiDriver g_imu(
    cube::Bmi270SpiDriver::Config::FromBuildDefaults());
cube::TeensyMoteusDriver g_motor;
cube::TeensyClock g_clock;

cube::StateEstimator* g_estimator = nullptr;
cube::BalancingController* g_controller = nullptr;

// --- loop state -------------------------------------------------------
uint32_t g_seq = 0;
uint32_t g_cycles = 0;
uint32_t g_late_cycles = 0;
uint32_t g_dropped = 0;
double g_worst_overrun = 0.0;
bool g_running = false;

// Halt with a reason.  Deliberately terminal: a control loop that has
// tripped must not quietly resume, and there is no operator console here
// to decide otherwise.  Power-cycling is the reset.
void halt(const char* reason) {
  g_motor.stop();
  g_running = false;
  Serial.println();
  Serial.printf("HALTED: %s\r\n", reason);
  Serial.println("Motor stopped.  Power-cycle to restart.");
}

// Build the configs from build flags.  Anything not supplied stays at its
// NaN/-1 sentinel, which is what the gate below detects.
cube::StateEstimator::Config makeEstimatorConfig() {
  cube::StateEstimator::Config c;
#ifdef EST_TAU
  c.tau = EST_TAU;
#endif
#ifdef EST_GYRO_AXIS
  c.gyro_axis = EST_GYRO_AXIS;
#endif
#ifdef EST_ACCEL_AXIS_A
  c.accel_axis_a = EST_ACCEL_AXIS_A;
#endif
#ifdef EST_ACCEL_AXIS_B
  c.accel_axis_b = EST_ACCEL_AXIS_B;
#endif
#ifdef EST_INVERT_THETA
  c.invert_theta = EST_INVERT_THETA;
#endif
#ifdef EST_THETA_OFFSET
  c.theta_offset = EST_THETA_OFFSET;
#endif
#ifdef EST_RATE_CUTOFF_HZ
  c.rate_cutoff_hz = EST_RATE_CUTOFF_HZ;
#endif
#ifdef EST_WARMUP_SAMPLES
  c.warmup_samples = EST_WARMUP_SAMPLES;
#endif
#ifdef EST_ACCEL_GATE
  c.accel_gate = EST_ACCEL_GATE;
#endif
  return c;
}

cube::BalancingController::Config makeControlConfig() {
  cube::BalancingController::Config c;
#ifdef CTL_K_THETA
  c.k_theta = CTL_K_THETA;
#endif
#ifdef CTL_K_THETA_DOT
  c.k_theta_dot = CTL_K_THETA_DOT;
#endif
#ifdef CTL_K_OMEGA
  c.k_omega = CTL_K_OMEGA;
#endif
#ifdef CTL_MAX_TILT_RAD
  c.max_tilt_rad = CTL_MAX_TILT_RAD;
#endif
#ifdef CTL_MAX_TORQUE_NM
  c.max_torque_nm = CTL_MAX_TORQUE_NM;
#endif
#ifdef CTL_MAX_WHEEL_OMEGA
  c.max_wheel_omega = CTL_MAX_WHEEL_OMEGA;
#endif
#ifdef CTL_THETA_DEADBAND
  c.theta_deadband = CTL_THETA_DEADBAND;
#endif
  return c;
}

// Fill and queue one telemetry record.  Never printf from the loop:
// formatting doubles at 200 Hz destroys the 5 ms budget outright.
void recordTelemetry(const cube::ImuData& imu, const cube::MotorState& motor,
                     const cube::BodyState& body,
                     const cube::ControlOutput& out, double dt, double slack,
                     bool dry_run) {
  cube::TelemetryFrame frame = {};

  frame.seq = g_seq++;
  frame.t_us = static_cast<uint32_t>(g_clock.micros64());

  frame.accel_si[0] = static_cast<float>(imu.accel_x);
  frame.accel_si[1] = static_cast<float>(imu.accel_y);
  frame.accel_si[2] = static_cast<float>(imu.accel_z);
  frame.gyro_si[0] = static_cast<float>(imu.gyro_x);
  frame.gyro_si[1] = static_cast<float>(imu.gyro_y);
  frame.gyro_si[2] = static_cast<float>(imu.gyro_z);
  frame.imu_temp_c = static_cast<float>(imu.temperature_c);

  frame.theta = static_cast<float>(body.theta);
  frame.theta_dot = static_cast<float>(body.theta_dot);
  frame.wheel_omega = static_cast<float>(body.wheel_omega);

  frame.motor_position_rev = static_cast<float>(motor.position_rev);
  frame.motor_velocity_rev_s = static_cast<float>(motor.velocity_rev_s);
  frame.motor_torque_nm = static_cast<float>(motor.torque_nm);
  frame.bus_voltage_v = static_cast<float>(motor.bus_voltage);
  frame.motor_temp_c = static_cast<float>(motor.motor_temperature_c);

  frame.cmd_torque_nm = static_cast<float>(out.torque_nm);

  // The three term_* fields are not decoration: without them, gain tuning
  // is blind -- you cannot tell which term saturated the output.
  frame.term_theta = static_cast<float>(out.term_theta);
  frame.term_theta_dot = static_cast<float>(out.term_theta_dot);
  frame.term_omega = static_cast<float>(out.term_omega);

  frame.dt_s = static_cast<float>(dt);
  frame.slack_s = static_cast<float>(slack);

  frame.mode = static_cast<uint8_t>(motor.mode);
  frame.fault = static_cast<uint8_t>(motor.fault);
  frame.safety = static_cast<uint8_t>(out.safety);

  frame.flags = 0;
  if (out.armed) { frame.flags |= cube::kFlagArmed; }
  if (out.torque_clamped) { frame.flags |= cube::kFlagTorqueClamped; }
  if (imu.valid) { frame.flags |= cube::kFlagImuValid; }
  if (motor.valid) { frame.flags |= cube::kFlagMotorValid; }
  if (body.valid) { frame.flags |= cube::kFlagBodyValid; }
  if (dry_run) { frame.flags |= cube::kFlagDryRun; }
  if (g_dropped > 0) { frame.flags |= cube::kFlagOverflow; }

  // FinalizeFrame stamps magic/length/version and the CRC.
  cube::FinalizeFrame(&frame);

  if (!g_ring.push(&frame)) { g_dropped++; }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println();
#if defined(ENABLE_BALANCER_TORQUE)
  Serial.println("*********************************************************");
  Serial.println("***  BALANCER -- TORQUE ARMED.  THIS MOVES THE MOTOR.  ***");
  Serial.println("***  WHEEL OFF FOR N1.  Tethered + soft surface for N2.***");
  Serial.println("*********************************************************");
#else
  Serial.println("=== cube_balancer -- S6, torque DISARMED ===");
#endif
  Serial.println();

  // --- the NaN gate, before anything can move ---------------------------
  //
  // Checked on probe copies so no device is even constructed if the
  // configuration is incomplete.
  {
    const cube::StateEstimator probe_estimator(makeEstimatorConfig());
    const cube::BalancingController probe_controller(makeControlConfig());

    const char* unset_filter = probe_estimator.firstUnsetField();
    const char* unset_gain = probe_controller.firstUnsetField();

    if (unset_filter != nullptr || unset_gain != nullptr) {
      Serial.println("REFUSING TO RUN: the control configuration is incomplete.");
      Serial.println();
      if (unset_filter != nullptr) {
        Serial.printf("  StateEstimator::Config::%s is unset\r\n", unset_filter);
      }
      if (unset_gain != nullptr) {
        Serial.printf("  BalancingController::Config::%s is unset\r\n",
                      unset_gain);
      }
      Serial.println();
      Serial.println("These are deliberately NaN -- they cannot be guessed");
      Serial.println("without measuring the rig.  See:");
      Serial.println("  docs/3d_scaling/README.md s3   inertia, mass, CoM");
      Serial.println("  docs/3d_scaling/README.md s4   the LQR gains");
      Serial.println();
      Serial.println("Set them as build_flags in platformio.ini.");
      Serial.println();
      Serial.println("(Reaching this message is the EXPECTED S6 result while");
      Serial.println(" the gains are unmeasured -- it proves the gate works.)");
      while (true) { delay(1000); }
    }
  }

  // --- e-stop ----------------------------------------------------------
  // Pull-up plus normally-closed to GND: LOW is run, HIGH is stop.  An
  // unwired pin therefore reads HIGH and refuses to run, rather than
  // defaulting to "safe to move".
  pinMode(kEstopPin, INPUT_PULLUP);
  if (digitalRead(kEstopPin) == HIGH) {
    Serial.println("E-STOP is OPEN (or unwired) -- close it to run.");
    Serial.printf("Wire a normally-closed button: pin %d <-> GND\r\n",
                  kEstopPin);
    while (digitalRead(kEstopPin) == HIGH) { delay(200); }
  }

  // --- devices ---------------------------------------------------------
  std::string error;

  Serial.print("IMU... ");
  if (!g_imu.initialize(&error)) {
    Serial.println("FAIL");
    Serial.println(error.c_str());
    while (true) { delay(1000); }
  }
  Serial.println("OK");

  Serial.print("gyro bias -- HOLD STILL... ");
  if (!g_imu.calibrateGyroBias(2.0, &error)) {
    Serial.println("FAIL");
    Serial.println(error.c_str());
    while (true) { delay(1000); }
  }
  Serial.println("OK");

  Serial.print("motor... ");
  cube::TeensyMoteusDriver::Config motor_config;
  motor_config.watchdog_timeout_s = kWatchdogS;
  g_motor = cube::TeensyMoteusDriver(motor_config);
  if (!g_motor.initialize(&error)) {
    Serial.println("FAIL");
    Serial.println(error.c_str());
    while (true) { delay(1000); }
  }
  Serial.println("OK");

#if defined(ENABLE_BALANCER_TORQUE)
  // The single line in this project that permits motion from a feedback
  // law.  Deliberately greppable, deliberately inside the flag.
  g_motor.enableTorque(true);
  Serial.println("TORQUE ARMED");
#else
  Serial.println("torque disarmed (S6)");
#endif

  static cube::StateEstimator estimator(makeEstimatorConfig());
  static cube::BalancingController controller(makeControlConfig());
  g_estimator = &estimator;
  g_controller = &controller;

  Serial.printf("rate %.0f Hz, watchdog %.0f ms\r\n", kControlHz,
                kWatchdogS * 1000.0);
  Serial.println("running.");

  g_running = true;
}

void loop() {
  if (!g_running) { delay(1000); return; }

  static double last = g_clock.seconds();
  static double next_deadline = g_clock.seconds() + kPeriod;

  const double now = g_clock.seconds();
  const double dt = now - last;
  last = now;

  // --- e-stop, every cycle ---------------------------------------------
  // First thing in the cycle: a stop must not wait on a sensor read.
  if (digitalRead(kEstopPin) == HIGH) {
    halt("physical e-stop opened");
    return;
  }

  // --- the control sequence --------------------------------------------
  //
  // Identical to apps/cube_balancer.cpp:253-273.  Query precedes command,
  // so the wheel speed feeding k_omega is measured THIS cycle rather than
  // the previous one.
  const cube::ImuData sample = g_imu.read();
  const cube::MotorState motor_state = g_motor.query();
  const cube::BodyState body = g_estimator->update(sample, motor_state, dt);
  const cube::ControlOutput out = g_controller->update(body, motor_state);

#if defined(ENABLE_BALANCER_TORQUE)
  const bool dry_run = false;
  if (out.armed) {
    g_motor.sendTorque(out.torque_nm);
  } else {
    g_motor.stop();
  }
#else
  // S6: the loop runs end to end, the estimator and controller are
  // exercised, and nothing is ever commanded.
  const bool dry_run = true;
  if (!out.armed) { g_motor.stop(); }
#endif

  g_cycles++;

  // --- schedule --------------------------------------------------------
  //
  // Absolute deadlines, not a fixed delay: sleeping for the period
  // accumulates every cycle's overrun into permanent drift, which at
  // 200 Hz is a real change in the effective control rate.
  next_deadline += kPeriod;
  const double slack = next_deadline - g_clock.seconds();

  if ((g_seq % kTelemetryDecimation) == 0) {
    recordTelemetry(sample, motor_state, body, out, dt, slack, dry_run);
  } else {
    g_seq++;
  }

  // A latched trip is terminal: report and stop, rather than spinning at
  // 200 Hz doing nothing.  reset() is a deliberate human act.
  if (!out.armed) {
    halt(cube::ToString(out.safety));
    Serial.printf("  theta %.3f rad   wheel %.1f rad/s   fault %d\r\n",
                  body.theta, body.wheel_omega, motor_state.fault);
    Serial.printf("  %lu cycles, %lu late (%.2f%%), worst overrun %.2f ms\r\n",
                  (unsigned long)g_cycles, (unsigned long)g_late_cycles,
                  g_cycles ? 100.0 * g_late_cycles / g_cycles : 0.0,
                  g_worst_overrun * 1000.0);
    return;
  }

  if (slack > 0.0) {
    // Drain telemetry in the slack, never inside the deadline.  The
    // budget caps the work per call so a backlog is paid down over
    // several cycles instead of one burst that overruns by itself.
    cube::DrainTelemetry(&g_ring, &g_sink, 4);

    const double remaining = next_deadline - g_clock.seconds();
    if (remaining > 0.0) {
      delayMicroseconds(static_cast<uint32_t>(remaining * 1e6));
    }
  } else {
    g_late_cycles++;
    if (-slack > g_worst_overrun) { g_worst_overrun = -slack; }
    // Missed badly enough that catching up would mean a burst of
    // back-to-back cycles; resynchronise instead.
    if (-slack > kPeriod) { next_deadline = g_clock.seconds() + kPeriod; }
  }

  (void)kPeriodUs;
}
