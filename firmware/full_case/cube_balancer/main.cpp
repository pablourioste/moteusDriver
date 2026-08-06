// cube_balancer -- INTEGRATION.md S6, then N1/N2.  The real control loop.
//
// The Teensy port of apps/cube_balancer.cpp.  Same call sequence, same
// safety ordering, same absolute-deadline scheduler; the CLI shell and the
// signal handlers are what fall away, replaced by single-char commands on a
// second USB serial port and the application state machine below.
//
//   BMI270 --> StateEstimator --> BalancingController --> moteus
//
// core/ is compiled from src/core/ in place, exactly as the host build
// compiles it.  Nothing in this file is a copy of the control law -- a
// copy is how a simulation and a rig quietly stop agreeing.
//
// ---------------------------------------------------------------------
// THE APPLICATION STATE MACHINE -- what this file owns
// ---------------------------------------------------------------------
//
// core/ decides WHAT torque the physics wants.  This file decides WHETHER
// the rig is allowed to have it, and it is the only file allowed to
// command zero torque for that reason.  Six states, no implicit path back
// into motion:
//
//   INIT    boot: bring up SPI/CAN, validate config, blocking on hard
//           failure (this is the ONE unrecoverable case -- broken
//           hardware needs a fix and a reflash, not a retry command)
//   CALIB   hold still, average the gyro; retries itself on rejection
//   IDLE    estimator runs, telemetry flows, torque HARD-ZEROED
//   ARMED   torque still zero; waiting for the rig to be presented
//           upright and still (the entry gate, below)
//   BALANCE entry gate passed; torque live (if not observe mode)
//   FAULT   torque zero, moteus stop(), LATCHED -- only 'r' gets out,
//           and only with the physical e-stop confirmed closed
//
// A trip inside BalancingController (out.armed == false) moves here
// automatically; the physical e-stop and the operator SPACE command do
// too.  controller.reset() is called from exactly one place: the 'r'
// handler.  See handleCommand() below -- this is the pitfall the spec
// calls out explicitly: re-arming is a deliberate human act, and calling
// reset() anywhere else is how a rig ends up re-tripping repeatedly while
// someone tries to work out why.
//
// ---------------------------------------------------------------------
// SAFETY -- read before flashing
// ---------------------------------------------------------------------
//
// This is the only firmware that runs the feedback law.  It commands
// torque from an estimate, so a sign error here drives the cube into its
// fall at full authority rather than catching it.
//
//   S6  torque path NOT COMPILED IN (default env).  Proves the loop and
//       the state machine hold 400 Hz and the NaN gate survived the
//       port.  Wheel off, motor power off.  'o' (observe toggle) is
//       accepted but is a no-op -- there is nothing to toggle.
//   N1  -D ENABLE_BALANCER_TORQUE, WHEEL STILL OFF.  First motor power.
//       Boots in OBSERVE MODE regardless -- 'o' must be pressed
//       explicitly before BALANCE can command real torque.  Tilt by
//       hand, confirm the shaft pushes the right way.
//   N2  wheel on, tethered, soft surface.  gain_scale boots at 0.1 and
//       is stepped with '+' through {0.1, 0.3, 0.6, 1.0}, one run per
//       step, per the spec.
//
// Like moteus_driver_test, the torque CALL SITE is a separate build
// rather than a runtime toggle: without -D ENABLE_BALANCER_TORQUE, the
// instruction that commands torque is not in the binary, full stop.
// Observe mode and the state machine are additional, independent layers
// on top of that -- they narrow when a torque-capable binary WILL
// command torque; they cannot make an S6 binary command any.
//
// If the cube drives its own fall, DO NOT flip the sign in the control
// law: the bug is upstream, in EST_INVERT_THETA or SOURCE0_SIGN, and
// negating the law to compensate hides it somewhere worse.
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
// Fill them in via build_flags: measure the rig into
// docs/3d_scaling/measurements.md, run tools/lqr/solve_gains.py, and paste
// its output into [env:cube_balancer].

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

// --- rate ---------------------------------------------------------------
//
// Matched to the IMU's configured ODR (Bmi270SpiDriver::Config::odr
// defaults to 400 Hz) rather than chosen independently, per the spec:
// running the control loop slower than the sensor just discards samples
// for no benefit.
constexpr double kControlHz = 400.0;
constexpr double kPeriod = 1.0 / kControlHz;

// --- motor torque constant ----------------------------------------------
//
// K_t = 60 / (2*pi*kv), with kv = 373.6 from this motor's own calibration
// (data/calibration/moteus-cal-AD4AN1QwUBYgOTNO-*.log, newest 2026-08-03).
// Used at boot to turn the board's servo.max_current_A into the torque it
// can actually produce -- see setup().  Re-derive it if the motor is ever
// swapped or recalibrated; a stale value here makes the check below agree
// with the wrong hardware.
constexpr double kTorqueConstantNmPerA = 0.0256;

// Board stops itself if it hears nothing for this long.  20 cycles is the
// same margin the host driver uses: long enough to absorb jitter, short
// enough that a dead host is caught quickly.  At 400 Hz that is 50 ms --
// half of what it was at 200 Hz, since it is a cycle count, not a fixed
// time.  THIS IS THE ONLY THING that stops the wheel if the Teensy hangs
// -- there is no OS to notice.
constexpr double kWatchdogS = 20.0 * kPeriod;

constexpr double kDegToRad = 0.017453292519943295;

// --- physical e-stop (H5) -----------------------------------------------
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

// --- entry gate (ARMED -> BALANCE) ---------------------------------------
//
// Never enable torque unless the rig is already near upright and slow.
// Enabling at 30 degrees means commanding full torque into a cube lying
// on its side.  Held CONTINUOUSLY for kEntryGateDwellS -- a single sample
// passing while someone waves the cube through vertical is not the rig
// being presented for balancing.
constexpr double kEntryGateMaxTiltRad = 2.0 * kDegToRad;
constexpr double kEntryGateMaxRateRadS = 0.2;
constexpr double kEntryGateMaxWheelRadS = 1.0;
constexpr double kEntryGateDwellS = 0.5;

// --- CAN grace period -----------------------------------------------------
//
// BalancingController trips kSensorFault on a single !motor.valid, with
// no grace -- see src/core/BalancingController.cpp.  The spec is explicit
// that grace belongs in exactly one place; core/ already owns the
// immediate trip, so this is where it lives.  A short run of misses gets
// the last known-good MotorState instead of the fresh (invalid) one, so
// one dropped CAN frame does not e-stop the rig.  Telemetry and the entry
// gate still see the RAW reading -- only the estimator/controller inputs
// are patched.
//
// This is the spec's "CAN silence: 3 consecutive missed replies" TRIP
// threshold, not a count of tolerated misses: the 1st and 2nd misses are
// patched, and the 3rd is passed through raw, reaching
// BalancingController's own immediate trip on that 3rd miss.  Tune with
// the comparison in the CAN grace block below, not just this constant, if
// that reading of "3" is ever revisited.
constexpr int kCanTripOnMisses = 3;

// --- gain_scale -----------------------------------------------------------
//
// A fixed rung table matching the spec's explicit ramp (0.1 -> 0.3 -> 0.6
// -> 1.0, one step per run) rather than a uniform step, so mashing '+'
// cannot overshoot past the recommended sequence.  Boots at index 1
// (0.1), the spec's "first closed-loop attempt" value -- not 1.0, and not
// 0.0 either, since a scale of exactly zero would look like a hung
// control law rather than a deliberately conservative one.
constexpr double kGainScaleRungs[] = {0.0, 0.1, 0.3, 0.6, 1.0};
constexpr int kGainScaleRungCount =
    sizeof(kGainScaleRungs) / sizeof(kGainScaleRungs[0]);
constexpr int kGainScaleDefaultRung = 1;

// --- two USB serial ports, and why ------------------------------------
//
// `Serial` carries BINARY TelemetryFrame records and nothing else.  All
// text -- state transitions, '?', refusals -- and all operator input go to
// `SerialUSB1`, the second CDC endpoint that -D USB_DUAL_SERIAL enables.
//
// They cannot share a port.  A frame is found by scanning for its 0xA5C3
// magic, so any text injected into the stream is either skipped as garbage
// (best case, and it still costs a resync) or contains those two bytes and
// is decoded as the start of a frame (worst case, and it corrupts the
// record after it).  telemetry_test states the same rule; this file is the
// one that also has to READ commands, which makes it worse: you cannot run
// tools/telemetry/capture.py on a port you are simultaneously typing 'a',
// 'o' and '+' into.
//
// Practically: capture.py opens the FIRST port, `pio device monitor` the
// SECOND.  On Windows they enumerate as two consecutive COM numbers.
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
#define CUBE_SECOND_SERIAL 1
#else
#define CUBE_SECOND_SERIAL 0
#warning "cube_balancer built without USB_DUAL_SERIAL: console text will be \
mixed into the binary telemetry stream and capture.py will see corruption. \
Add -D USB_DUAL_SERIAL to this environment's build_flags."
#endif

#if CUBE_SECOND_SERIAL
// Stream, not the concrete class: everything this file needs (print,
// println, printf, available, read) is on Print/Stream, so the two ports
// are interchangeable here and only setup() has to know which is which.
Stream& g_console = SerialUSB1;
#else
Stream& g_console = Serial;
#endif

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

// --- application state machine ----------------------------------------
enum class AppState : uint8_t { kInit, kCalib, kIdle, kArmed, kBalance, kFault };

const char* ToString(AppState s) {
  switch (s) {
    case AppState::kInit: return "INIT";
    case AppState::kCalib: return "CALIB";
    case AppState::kIdle: return "IDLE";
    case AppState::kArmed: return "ARMED";
    case AppState::kBalance: return "BALANCE";
    case AppState::kFault: return "FAULT";
  }
  return "UNKNOWN";
}

AppState g_state = AppState::kInit;

// Set only by transitionTo(), read by telemetry and '?'.
const char* g_fault_reason = "";

// Never toggled automatically.  Boots true even in a torque-capable build
// -- see the SAFETY block above -- so leaving observe mode is always an
// explicit operator act, on top of (not instead of) the compile-time gate.
bool g_observe_mode = true;

// True only after a SUCCESSFUL calibrateGyroBias().  The driver zeroes
// the bias before measuring and does NOT restore it on rejection (see
// src/drivers/ImuDriver.cpp, ported verbatim into Bmi270SpiDriver), so a
// rejected recalibration leaves a WORSE bias than before it was
// attempted.  Gates the entry gate below so that cannot be armed past
// silently.
bool g_bias_valid = false;

int g_gain_scale_rung = kGainScaleDefaultRung;
double g_gain_scale = kGainScaleRungs[kGainScaleDefaultRung];

// 0.0 means "not currently passing".  Reset the instant any entry-gate
// condition fails, including on every cycle spent outside ARMED.
double g_gate_pass_since = 0.0;

// CAN grace bookkeeping -- see the constant's comment above.
bool g_have_good_motor = false;
int g_can_miss_count = 0;
cube::MotorState g_last_good_motor;

// Last raw sample and body/motor state, kept only so '?' can report them
// without re-reading the bus outside the control cycle.
cube::ImuData g_last_sample;
cube::BodyState g_last_body;
cube::MotorState g_last_motor;

// --- loop diagnostics ---------------------------------------------------
uint32_t g_seq = 0;
uint32_t g_cycles = 0;
uint32_t g_late_cycles = 0;
uint32_t g_dropped = 0;
double g_worst_overrun = 0.0;
bool g_running = false;

// Print + set.  The one place AppState changes, so every transition is
// visible on the console without hunting through the loop for where it
// happened.
void transitionTo(AppState next, const char* reason) {
  if (next == g_state) { return; }
  g_console.printf("STATE: %s -> %s  (%s)\r\n", ToString(g_state), ToString(next),
               reason);
  if (next == AppState::kFault) { g_fault_reason = reason; }
  if (next != AppState::kArmed) { g_gate_pass_since = 0.0; }
  g_state = next;
}

// Unrecoverable hardware bring-up failure -- SPI/CAN never came up, or a
// config field is still NaN.  Distinct from AppState::kFault, which is
// recoverable with 'r': there is no operator command that fixes a part
// that failed to answer on the bus.  Power-cycle after fixing it.
void hangForever(const char* reason) {
  g_motor.stop();
  g_console.println();
  g_console.printf("HALTED (unrecoverable): %s\r\n", reason);
  g_console.println("Fix the hardware/config and power-cycle to restart.");
  while (true) { delay(1000); }
}

// Build the configs from build flags.  Anything not supplied stays at its
// NaN/-1 sentinel, which is what the gate below detects.
cube::StateEstimator::Config makeEstimatorConfig() {
  cube::StateEstimator::Config c;
  // Tied to kControlHz, not a leftover assumption: the reject-bad-dt gate
  // is relative to this, so it has to track whatever the loop actually
  // runs at.
  c.nominal_dt = kPeriod;
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
#ifdef CTL_WHEEL_TAPER_START
  c.wheel_taper_start = CTL_WHEEL_TAPER_START;
#endif
#ifdef CTL_THETA_DEADBAND
  c.theta_deadband = CTL_THETA_DEADBAND;
#endif
#ifdef CTL_MAX_INVALID_CYCLES
  c.max_invalid_cycles = CTL_MAX_INVALID_CYCLES;
#endif
  return c;
}

// Spec Sec.6: "print the full config on boot... when a log turns up three
// days later, that header is the difference between a usable log and a
// mystery."  Also reachable on demand via '?'.
void printFullConfig() {
  const cube::StateEstimator::Config& e = g_estimator->config();
  const cube::BalancingController::Config& c = g_controller->config();
  g_console.println("--- config -------------------------------------------");
  g_console.printf("  built  %s %s\r\n", __DATE__, __TIME__);
  g_console.printf("  rate   %.0f Hz  (period %.3f ms, watchdog %.1f ms)\r\n",
               kControlHz, kPeriod * 1000.0, kWatchdogS * 1000.0);
  g_console.printf("  est    tau=%.4f  gyro_axis=%d  accel_axis=(%d,%d)  "
               "invert=%d  theta_offset=%.5f\r\n",
               e.tau, e.gyro_axis, e.accel_axis_a, e.accel_axis_b,
               e.invert_theta ? 1 : 0, e.theta_offset);
  g_console.printf("         rate_cutoff_hz=%.2f  nominal_dt=%.5f  "
               "warmup=%d  accel_gate=%.3f\r\n",
               e.rate_cutoff_hz, e.nominal_dt, e.warmup_samples,
               e.accel_gate);
  g_console.printf("  ctl    k_theta=%.5f  k_theta_dot=%.5f  k_omega=%.6f\r\n",
               c.k_theta, c.k_theta_dot, c.k_omega);
  g_console.printf("         max_tilt=%.4f rad  max_torque=%.4f Nm  "
               "max_wheel_omega=%.2f  wheel_taper_start=%.2f\r\n",
               c.max_tilt_rad, c.max_torque_nm, c.max_wheel_omega,
               c.wheel_taper_start);
  g_console.printf("         theta_deadband=%.5f  max_invalid_cycles=%d\r\n",
               c.theta_deadband, c.max_invalid_cycles);
  g_console.println("-------------------------------------------------------");
}

void printStatus() {
  printFullConfig();
  g_console.printf("  state       %s%s%s\r\n", ToString(g_state),
               g_state == AppState::kFault ? "  reason: " : "",
               g_state == AppState::kFault ? g_fault_reason : "");
#if defined(ENABLE_BALANCER_TORQUE)
  const char* observe_note = "";
#else
  const char* observe_note = "  (no-op: torque path not compiled in)";
#endif
  g_console.printf("  observe     %s%s\r\n", g_observe_mode ? "ON" : "OFF",
               observe_note);
  g_console.printf("  gain_scale  %.2f  (rung %d/%d)\r\n", g_gain_scale,
               g_gain_scale_rung, kGainScaleRungCount - 1);
  g_console.printf("  gyro bias   x=%+9.6f  y=%+9.6f  z=%+9.6f  rad/s  "
               "(%s)\r\n",
               g_imu.config().gyro_bias_x, g_imu.config().gyro_bias_y,
               g_imu.config().gyro_bias_z,
               g_bias_valid ? "valid" : "INVALID -- recalibrate");
  g_console.printf("  theta_accel %.4f rad  (raw, last sample)\r\n",
               g_estimator->accelAngle(g_last_sample));
  g_console.printf("  theta       %.4f rad  theta_dot %.4f rad/s  wheel_omega "
               "%.2f rad/s  (filtered, last cycle)\r\n",
               g_last_body.theta, g_last_body.theta_dot,
               g_last_body.wheel_omega);
  g_console.printf("  motor       valid=%d  mode=%d  fault=%d  bus=%.2f V  "
               "temp=%.1f C\r\n",
               g_last_motor.valid ? 1 : 0, g_last_motor.mode,
               g_last_motor.fault, g_last_motor.bus_voltage,
               g_last_motor.motor_temperature_c);
  g_console.printf("  cycles      %lu  late %lu (%.2f%%)  worst overrun "
               "%.2f ms  dropped-telemetry %lu\r\n",
               (unsigned long)g_cycles, (unsigned long)g_late_cycles,
               g_cycles ? 100.0 * g_late_cycles / g_cycles : 0.0,
               g_worst_overrun * 1000.0, (unsigned long)g_dropped);
}

void stepGainScale(int delta) {
  g_gain_scale_rung += delta;
  if (g_gain_scale_rung < 0) { g_gain_scale_rung = 0; }
  if (g_gain_scale_rung >= kGainScaleRungCount) {
    g_gain_scale_rung = kGainScaleRungCount - 1;
  }
  g_gain_scale = kGainScaleRungs[g_gain_scale_rung];
  g_console.printf("gain_scale -> %.2f (rung %d/%d)\r\n", g_gain_scale,
               g_gain_scale_rung, kGainScaleRungCount - 1);
}

// Shared by the blocking INIT-time calibration and the operator 'c'
// command.  Returns false on rejection; caller decides whether to retry.
// g_bias_valid is cleared BEFORE attempting, not after failing, because
// the driver has already zeroed the bias by the time calibrateGyroBias()
// returns at all -- there is no "old value" to fall back to once this
// starts.
bool runCalibration() {
  g_bias_valid = false;
  g_console.print("gyro bias -- HOLD STILL... ");
  std::string error;
  if (!g_imu.calibrateGyroBias(2.0, &error)) {
    g_console.println("FAIL");
    g_console.println(error.c_str());
    return false;
  }
  g_console.println("OK");
  g_bias_valid = true;
  return true;
}

void handleCommand(char c) {
  switch (c) {
    case 'a':
      if (g_state != AppState::kIdle) {
        g_console.printf("ARM ignored: not IDLE (currently %s)\r\n",
                     ToString(g_state));
      } else if (!g_bias_valid) {
        g_console.println(
            "ARM refused: gyro bias not valid -- recalibrate first ('c').");
      } else {
        transitionTo(AppState::kArmed, "operator ARM");
      }
      break;

    case 'd':
      if (g_state == AppState::kArmed || g_state == AppState::kBalance) {
        transitionTo(AppState::kIdle, "operator DISARM");
      } else {
        g_console.printf("DISARM ignored: not ARMED/BALANCE (currently %s)\r\n",
                     ToString(g_state));
      }
      break;

    case ' ':
      g_motor.stop();
      transitionTo(AppState::kFault, "operator e-stop");
      break;

    case 'r':
      if (g_state != AppState::kFault) {
        g_console.println("RESET ignored: not in FAULT.");
      } else if (digitalRead(kEstopPin) == HIGH) {
        g_console.println(
            "RESET refused: physical e-stop still open -- close it first.");
      } else {
        // The only call to these two reset()s in this file, and the only
        // place either happens that is not the direct result of an
        // explicit human command -- see the pitfall in the header
        // comment.  StateEstimator::reset() matters here specifically:
        // whatever happened between the trip and this moment (a fall, or
        // someone picking the rig up and reorienting it by hand) may have
        // put real but meaningless motion through the filter, and this
        // forces a fresh accelerometer-seeded start from wherever the rig
        // actually is now, per StateEstimator.hpp's own "use after an
        // e-stop, before re-arming."
        g_controller->reset();
        g_estimator->reset();
        transitionTo(AppState::kIdle, "operator RESET");
      }
      break;

    case 'c':
      if (g_state != AppState::kIdle) {
        g_console.printf("RECALIBRATE ignored: only from IDLE (currently %s)\r\n",
                     ToString(g_state));
      } else {
        transitionTo(AppState::kCalib, "operator recalibrate");
        runCalibration();  // single attempt; operator is present and
                            // watching, unlike the INIT-time retry loop
        transitionTo(AppState::kIdle, g_bias_valid
                                          ? "recalibration OK"
                                          : "recalibration FAILED -- ARM "
                                            "refused until retried");
      }
      break;

    case 'o':
#if defined(ENABLE_BALANCER_TORQUE)
      g_observe_mode = !g_observe_mode;
      g_console.printf("observe mode: %s\r\n",
                   g_observe_mode ? "ON (torque hard-zeroed)"
                                  : "OFF -- TORQUE LIVE IN BALANCE");
#else
      g_console.println(
          "observe mode toggle ignored: torque path not compiled in "
          "(build with -D ENABLE_BALANCER_TORQUE for N1/N2).");
#endif
      break;

    case '+':
      stepGainScale(+1);
      break;

    case '-':
      stepGainScale(-1);
      break;

    case '?':
      printStatus();
      break;

    default:
      break;  // ignore newlines, unrecognised bytes
  }
}

void pollCommands() {
  while (g_console.available() > 0) {
    handleCommand(static_cast<char>(g_console.read()));
  }
}

// ARMED -> BALANCE.  Uses the RAW motor reading, not the CAN-grace-patched
// one: arming into the closed loop should require a genuinely fresh
// healthy reply, not one held over from a few cycles ago.
void checkEntryGate(const cube::BodyState& body, const cube::MotorState& motor) {
  const bool ok = body.valid &&
                  std::fabs(body.theta) < kEntryGateMaxTiltRad &&
                  std::fabs(body.theta_dot) < kEntryGateMaxRateRadS &&
                  std::fabs(body.wheel_omega) < kEntryGateMaxWheelRadS &&
                  motor.valid && motor.fault == 0;
  if (!ok) {
    g_gate_pass_since = 0.0;
    return;
  }
  const double now = g_clock.seconds();
  if (g_gate_pass_since == 0.0) {
    g_gate_pass_since = now;
    return;
  }
  if (now - g_gate_pass_since >= kEntryGateDwellS) {
    transitionTo(AppState::kBalance, "entry gate passed");
  }
}

// Fill and queue one telemetry record.  Never printf from the loop:
// formatting doubles at 400 Hz destroys the budget outright.
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
  // dry_run here means "torque not authorised this cycle" -- state,
  // observe mode and the trip latch collapsed into the one flag the wire
  // format already has, rather than a build-time constant.  AppState,
  // gain_scale and gyro bias are NOT in this record -- see the header
  // comment on TelemetryFrame and the PR notes: v1 is unchanged, that
  // data goes out as text (state transitions, '?') for now.
  if (dry_run) { frame.flags |= cube::kFlagDryRun; }
  if (g_dropped > 0) { frame.flags |= cube::kFlagOverflow; }

  // FinalizeFrame stamps magic/length/version and the CRC.
  cube::FinalizeFrame(&frame);

  if (!g_ring.push(&frame)) { g_dropped++; }
}

}  // namespace

void setup() {
  // Baud is ignored on USB CDC -- both ports run at full USB speed
  // regardless of what is passed here.  begin() is still required to
  // enumerate them.
  Serial.begin(115200);
#if CUBE_SECOND_SERIAL
  SerialUSB1.begin(115200);
#endif
  // Bounded wait for USB enumeration, so the banner and any refusal
  // message are not printed into a port nobody has opened yet.  Both CDC
  // interfaces belong to one composite device and come up together, so
  // waiting on `Serial` covers the console too.  Bounded, so a rig running
  // headless still boots.
  while (!Serial && millis() < 3000) {}

  g_console.println();
#if defined(ENABLE_BALANCER_TORQUE)
  g_console.println("*********************************************************");
  g_console.println("***  BALANCER -- TORQUE-CAPABLE BUILD.                 ***");
  g_console.println("***  Boots in OBSERVE MODE regardless -- 'o' to arm    ***");
  g_console.println("***  real torque.  WHEEL OFF for N1.                   ***");
  g_console.println("*********************************************************");
#else
  g_console.println("=== cube_balancer -- S6, torque path not compiled in ===");
#endif
  g_console.println();

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
      g_console.println("REFUSING TO RUN: the control configuration is incomplete.");
      g_console.println();
      if (unset_filter != nullptr) {
        g_console.printf("  StateEstimator::Config::%s is unset\r\n", unset_filter);
      }
      if (unset_gain != nullptr) {
        g_console.printf("  BalancingController::Config::%s is unset\r\n",
                      unset_gain);
      }
      g_console.println();
      g_console.println("These are deliberately NaN -- they cannot be guessed");
      g_console.println("without measuring the rig.  See:");
      g_console.println("  docs/3d_scaling/README.md s3   inertia, mass, CoM");
      g_console.println("  docs/3d_scaling/README.md s4   the LQR gains");
      g_console.println();
      g_console.println("Set them as build_flags in platformio.ini.");
      g_console.println();
      g_console.println("(Reaching this message is the EXPECTED S6 result while");
      g_console.println(" the gains are unmeasured -- it proves the gate works.)");
      hangForever("configuration incomplete");
    }
  }

  // --- e-stop ----------------------------------------------------------
  // Pull-up plus normally-closed to GND: LOW is run, HIGH is stop.  An
  // unwired pin therefore reads HIGH and refuses to run, rather than
  // defaulting to "safe to move".
  pinMode(kEstopPin, INPUT_PULLUP);
  if (digitalRead(kEstopPin) == HIGH) {
    g_console.println("E-STOP is OPEN (or unwired) -- close it to run.");
    g_console.printf("Wire a normally-closed button: pin %d <-> GND\r\n",
                  kEstopPin);
    while (digitalRead(kEstopPin) == HIGH) { delay(200); }
  }

  // --- devices ---------------------------------------------------------
  std::string error;

  g_console.print("IMU... ");
  if (!g_imu.initialize(&error)) {
    g_console.println("FAIL");
    g_console.println(error.c_str());
    hangForever("IMU initialize() failed");
  }
  g_console.println("OK");

  g_console.print("motor... ");
  cube::TeensyMoteusDriver::Config motor_config;
  motor_config.watchdog_timeout_s = kWatchdogS;

  // Two torque clamps sit in series -- BalancingController's and this
  // driver's -- and the effective ceiling is the lower of them.  The
  // control law's is the one that is supposed to bind: it is the number
  // the LQR solve assumed, and the number the operator tuned.  The
  // driver's is a backstop.  So raise the driver's whenever the law's is
  // higher, exactly as apps/cube_balancer.cpp does; leaving it at its 0.5
  // default means a law clamped to, say, 0.6 Nm silently never reaches its
  // own limit and every torque trace is clipped for reasons nothing
  // reports.
  const double law_max_torque_nm = makeControlConfig().max_torque_nm;
  if (law_max_torque_nm > motor_config.max_torque_nm) {
    motor_config.max_torque_nm = law_max_torque_nm;
  }

  g_motor = cube::TeensyMoteusDriver(motor_config);
  if (!g_motor.initialize(&error)) {
    g_console.println("FAIL");
    g_console.println(error.c_str());
    hangForever("motor initialize() failed");
  }
  g_console.println("OK");

  // --- what the board can ACTUALLY deliver -------------------------------
  //
  // Both clamps above are software believing things about the hardware.
  // The hardware's own limit is servo.max_current_A, which lives in the
  // board's flash and is provisioned from the host (S5) -- nothing in this
  // firmware sets it, so nothing in this firmware knows it without asking.
  //
  // Asking matters because the two failure directions look nothing alike.
  // Too HIGH is rig-destroying: exceeding the motor's or the board's rating
  // damages hardware.  Too LOW is quiet: the control law saturates below
  // the authority its gains were computed for, the cube fails to catch
  // itself, and every trace looks like the gains are wrong rather than like
  // the current limit is.  A power cycle after a session that pushed a
  // volatile config produces exactly the second case.
  //
  // So read it back and refuse on a mismatch -- the same refuse-to-run
  // discipline firstUnsetField() applies to the gains, extended across the
  // CAN link to the board's own settings.  Startup-only: the exchange takes
  // several round trips.  See TeensyMoteusDriver::readConfigDouble().
  {
    double max_current_a = 0.0;
    g_console.print("servo.max_current_A... ");
    if (!g_motor.readConfigDouble("servo.max_current_A", &max_current_a, &error)) {
      g_console.println("FAIL");
      g_console.println(error.c_str());
      hangForever("could not read servo.max_current_A back from the board");
    }

    const double board_ceiling_nm = kTorqueConstantNmPerA * max_current_a;
    g_console.printf("%.2f A  ->  %.3f Nm at K_t = %.4f Nm/A\r\n",
                     max_current_a, board_ceiling_nm, kTorqueConstantNmPerA);

    if (!(max_current_a > 0.0)) {
      // Also catches a NaN read, which would make every comparison below
      // false and wave the check through.
      hangForever("servo.max_current_A is zero, negative or unreadable");
    }
    if (law_max_torque_nm > board_ceiling_nm) {
      g_console.printf(
          "  CTL_MAX_TORQUE_NM = %.3f Nm, but %.2f A can only deliver "
          "%.3f Nm.\r\n",
          law_max_torque_nm, max_current_a, board_ceiling_nm);
      g_console.println(
          "  The control law would saturate before reaching its own limit,");
      g_console.println(
          "  so the LQR gains were solved for authority the rig does not");
      g_console.println("  have.  Lower CTL_MAX_TORQUE_NM, or raise");
      g_console.println(
          "  SERVO_MAX_CURRENT_A -- but only within the LOWER of the motor's");
      g_console.println("  and the board's rating.");
      hangForever("max_torque_nm exceeds what servo.max_current_A can deliver");
    }
  }

  static cube::StateEstimator estimator(makeEstimatorConfig());
  static cube::BalancingController controller(makeControlConfig());
  g_estimator = &estimator;
  g_controller = &controller;

  // --- CALIB -------------------------------------------------------------
  //
  // "retry on rejection", per the spec -- this is the one place a failure
  // is expected and self-heals rather than halting: someone bumping the
  // rig during the 3 s window is normal, not a hardware fault.
  transitionTo(AppState::kCalib, "init succeeded");
  while (!runCalibration()) {
    g_console.println("retrying in 1s -- hold the rig still...");
    delay(1000);
  }

#if defined(ENABLE_BALANCER_TORQUE)
  // The single line in this project that permits motion from a feedback
  // law to exist in the binary at all.  Deliberately greppable.  Runtime
  // authority (state == BALANCE, not observe_mode, armed) still gates
  // every individual sendTorque() call in the loop below.
  g_motor.enableTorque(true);
#endif

  printFullConfig();
  g_console.printf("rate %.0f Hz, watchdog %.0f ms\r\n", kControlHz,
                kWatchdogS * 1000.0);
  g_console.println(
      "Commands: a=arm d=disarm SPACE=estop r=reset c=recal o=observe "
      "+/-=gain ?=status");

  transitionTo(AppState::kIdle, "calibration OK");
  g_running = true;
}

void loop() {
  if (!g_running) { delay(1000); return; }

  static double last = g_clock.seconds();
  static double next_deadline = g_clock.seconds() + kPeriod;

  const double now = g_clock.seconds();
  const double dt = now - last;
  last = now;

  // --- e-stop, every cycle, before anything else -------------------------
  // A stop must not wait on a sensor read.  Falls through rather than
  // returning: with authority gated by AppState below, letting the rest
  // of the cycle run keeps telemetry and '?' alive while FAULT-ed, which
  // return-early-and-freeze did not.
  if (digitalRead(kEstopPin) == HIGH) {
    g_motor.stop();
    transitionTo(AppState::kFault, "physical e-stop opened");
  }

  // --- operator commands ---------------------------------------------
  pollCommands();

  // --- the control sequence --------------------------------------------
  //
  // Identical shape to apps/cube_balancer.cpp.  Query precedes command,
  // so the wheel speed feeding k_omega is measured THIS cycle rather than
  // the previous one.
  const cube::ImuData sample = g_imu.read();
  g_last_sample = sample;
  const cube::MotorState motor_state = g_motor.query();

  // --- CAN grace period ----------------------------------------------
  //
  // See the kCanTripOnMisses comment.  motor_for_controller is what the
  // estimator and controller see; motor_state (raw) is what telemetry and
  // the entry gate see, so a held-over reading never gets reported as
  // fresh.
  cube::MotorState motor_for_controller = motor_state;
  if (motor_state.valid) {
    g_last_good_motor = motor_state;
    g_have_good_motor = true;
    g_can_miss_count = 0;
  } else {
    g_can_miss_count++;
    if (g_have_good_motor && g_can_miss_count < kCanTripOnMisses) {
      motor_for_controller = g_last_good_motor;
    }
    // else: grace exhausted (this is the kCanTripOnMisses-th consecutive
    // miss, or no good reading has ever arrived) -- motor_for_controller
    // stays the raw, invalid reading, and BalancingController's own
    // !motor.valid -> kSensorFault fires.
  }

  const cube::BodyState body =
      g_estimator->update(sample, motor_for_controller, dt);
  const cube::ControlOutput out =
      g_controller->update(body, motor_for_controller, g_gain_scale);
  g_last_body = body;
  g_last_motor = motor_state;

  if (!out.armed && g_state != AppState::kFault) {
    transitionTo(AppState::kFault, cube::ToString(out.safety));
    // The numbers at the moment of the trip -- '?' after the fact only
    // shows the CURRENT (post-trip, likely settling) state, and this is
    // what actually tripped it.
    g_console.printf(
        "  theta %.3f rad   theta_dot %.3f rad/s   wheel %.1f rad/s   "
        "motor fault %d\r\n",
        body.theta, body.theta_dot, body.wheel_omega, motor_state.fault);
  }

  // --- apply authority: state machine + observe mode + the trip latch ---
  const bool authority =
      (g_state == AppState::kBalance) && !g_observe_mode && out.armed;
#if defined(ENABLE_BALANCER_TORQUE)
  if (authority) {
    g_motor.sendTorque(out.torque_nm);
  } else {
    g_motor.stop();
  }
#else
  // Torque path not compiled in at all -- always query-only, regardless
  // of `authority`, which in this build is always false anyway
  // (g_observe_mode is forced true).  stop() still asserts "no command"
  // on the wire every cycle rather than relying on the board's own
  // watchdog to notice silence.
  g_motor.stop();
#endif

  g_cycles++;

  if (g_state == AppState::kArmed) {
    checkEntryGate(body, motor_state);
  }

  // --- schedule --------------------------------------------------------
  //
  // Absolute deadlines, not a fixed delay: sleeping for the period
  // accumulates every cycle's overrun into permanent drift, which at
  // 400 Hz is a real change in the effective control rate.
  next_deadline += kPeriod;
  const double slack = next_deadline - g_clock.seconds();

  // Decimate on the CYCLE count, not on frame.seq.  seq has to stay a
  // gapless count of frames EMITTED, because that is what the host uses to
  // detect loss: tools/telemetry/decode.py reads a jump in seq as dropped
  // frames.  Driving the decimation from seq itself would make every
  // skipped cycle look like a dropped frame the moment kTelemetryDecimation
  // is raised above 1 -- a debugging trail that leads nowhere, since
  // nothing was actually lost.
  if ((g_cycles % kTelemetryDecimation) == 0) {
    recordTelemetry(sample, motor_state, body, out, dt, slack, !authority);
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
}
