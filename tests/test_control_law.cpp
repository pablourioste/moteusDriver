// Host-side tests for StateEstimator and BalancingController.
//
// Deliberately free-standing, matching tests/test_telemetry_ring.cpp: no
// Boost, no gtest, plain checks and a non-zero exit code.  The upstream
// moteus tests need Boost.Test and are skipped where it is absent, and
// these are exactly the two modules that must not silently go unverified.
//
// THESE RUN ON A DESKTOP WITH NO HARDWARE.  That is the point.  Both
// classes are pure functions of their inputs -- they take an ImuData and a
// MotorState and return a BodyState / ControlOutput, touching no bus and no
// clock -- so the entire safety envelope can be exercised before the rig is
// ever powered.  Every check below corresponds to a specific piece of
// reasoning written down in src/core/*.cpp; if one of these fails, read the
// comment at the line it names before changing the test.
//
// What is NOT tested here, because it cannot be: whether the GAINS are
// right.  That needs the rig.  These tests pin the machinery around the
// gains -- the sentinel gate, the trip ordering, the latch, the clamps --
// which is the part that has to be correct no matter what the gains end up
// being.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "core/BalancingController.hpp"
#include "core/StateEstimator.hpp"
#include "core/Types.hpp"

namespace {

int g_failures = 0;

void Check(bool condition, const char* what) {
  if (!condition) {
    std::printf("  FAIL  %s\n", what);
    g_failures++;
  }
}

void CheckNear(double got, double want, double tol, const char* what) {
  if (!(std::fabs(got - want) <= tol)) {
    std::printf("  FAIL  %s\n", what);
    std::printf("        got %.9g, want %.9g (tol %.3g)\n", got, want, tol);
    g_failures++;
  }
}

// Loops below check the same property every iteration, so one broken
// property would print dozens of identical lines.  They bail out on the
// first NEW failure -- measured against a baseline taken at loop entry, not
// against g_failures being zero, or an unrelated earlier failure would cut
// every later loop short after one pass and cascade into false reports.
class LoopGuard {
 public:
  LoopGuard() : baseline_(g_failures) {}
  bool failed() const { return g_failures > baseline_; }

 private:
  int baseline_;
};

void CheckSafety(const cube::ControlOutput& out, cube::SafetyState want,
                 const char* what) {
  if (out.safety != want) {
    std::printf("  FAIL  %s\n", what);
    std::printf("        got %s, want %s\n", cube::ToString(out.safety),
                cube::ToString(want));
    g_failures++;
  }
}

// Standard gravity, repeated here rather than pulled from the estimator's
// anonymous namespace.  An independent copy is the point: if someone
// changes the constant in StateEstimator.cpp, the accelerometer trust gate
// tests below should notice.
constexpr double kG = 9.80665;
constexpr double kPi = 3.14159265358979323846;

// --------------------------------------------------------------- fixtures

// A fully-configured controller.  Numbers chosen to make the arithmetic in
// the tests checkable by hand, NOT to resemble this rig -- the real gains
// come from docs/3d_scaling/measurements.md and are not known yet.
cube::BalancingController::Config MakeControlConfig() {
  cube::BalancingController::Config c;
  c.k_theta = 1.0;
  c.k_theta_dot = 0.0;
  c.k_omega = 0.0;
  c.max_tilt_rad = 0.25;
  c.max_torque_nm = 1.0;
  c.max_wheel_omega = 300.0;
  c.wheel_taper_start = 255.0;
  c.theta_deadband = 0.0;
  c.max_invalid_cycles = 3;
  return c;
}

// IMU flat on a cube face, tilting about sensor X: gravity swings in Y-Z.
// This is the mapping StateEstimator.hpp uses as its worked example.
cube::StateEstimator::Config MakeEstimatorConfig() {
  cube::StateEstimator::Config c;
  c.tau = 0.5;
  c.gyro_axis = 0;
  c.accel_axis_a = 1;
  c.accel_axis_b = 2;
  c.invert_theta = false;
  c.theta_offset = 0.0;
  c.rate_cutoff_hz = 15.0;
  c.nominal_dt = 0.0025;  // 400 Hz, the rig's rate
  c.warmup_samples = 1;
  c.accel_gate = 0.25;
  return c;
}

cube::BodyState MakeBody(double theta, double theta_dot, double wheel_omega) {
  cube::BodyState s;
  s.theta = theta;
  s.theta_dot = theta_dot;
  s.wheel_omega = wheel_omega;
  s.valid = true;
  return s;
}

cube::MotorState MakeMotor(int fault = 0) {
  cube::MotorState m;
  m.valid = true;
  m.fault = fault;
  return m;
}

// An IMU sample tilted by `angle` in the Y-Z plane, at exactly 1 g.
cube::ImuData MakeImu(double angle, double gyro_x = 0.0) {
  cube::ImuData d;
  d.accel_x = 0.0;
  d.accel_y = kG * std::sin(angle);
  d.accel_z = kG * std::cos(angle);
  d.gyro_x = gyro_x;
  d.valid = true;
  return d;
}

// ============================================================ CONTROLLER
// ------------------------------------------------- the sentinel gate

void TestUnsetFieldsAreNamedInOrder() {
  std::printf("BalancingController: firstUnsetField names each sentinel\n");

  // The order matters: it is the order the operator is asked to fix them
  // in, and cube_balancer prints only the FIRST one.  A field silently
  // dropping out of this chain is a field that can reach the rig unset.
  cube::BalancingController::Config c;
  const char* expected[] = {"k_theta",         "k_theta_dot",
                            "k_omega",         "max_tilt_rad",
                            "max_torque_nm",   "max_wheel_omega",
                            "wheel_taper_start", "theta_deadband",
                            "max_invalid_cycles"};
  double* const fields[] = {&c.k_theta,         &c.k_theta_dot,
                            &c.k_omega,         &c.max_tilt_rad,
                            &c.max_torque_nm,   &c.max_wheel_omega,
                            &c.wheel_taper_start, &c.theta_deadband};

  for (int i = 0; i < 8; i++) {
    const cube::BalancingController probe(c);
    const char* got = probe.firstUnsetField();
    Check(got != nullptr && std::string(got) == expected[i],
          expected[i]);
    // theta_deadband's documented preferred value is 0.0, which is a
    // perfectly good SET value -- the sentinel is NaN, not zero.
    *fields[i] = 0.0;
  }

  {
    const cube::BalancingController probe(c);
    const char* got = probe.firstUnsetField();
    Check(got != nullptr && std::string(got) == expected[8],
          "max_invalid_cycles (an int, sentinel -1 not NaN)");
  }

  c.max_invalid_cycles = 0;
  const cube::BalancingController done(c);
  Check(done.firstUnsetField() == nullptr, "all set -> nullptr");
  Check(done.isConfigured(), "all set -> isConfigured()");
}

void TestUnconfiguredRefusesToCommand() {
  std::printf("BalancingController: unconfigured commands nothing\n");

  // The whole reason the sentinels are NaN rather than a plausible default.
  // Note this deliberately does NOT latch -- it is a refusal to start, not
  // a trip, so a controller that is later configured properly still works.
  cube::BalancingController controller;  // every field a sentinel
  const cube::ControlOutput out =
      controller.update(MakeBody(0.05, 0.0, 0.0), MakeMotor());

  Check(!out.armed, "unconfigured -> not armed");
  CheckSafety(out, cube::SafetyState::kUnconfigured, "unconfigured -> kUnconfigured");
  CheckNear(out.torque_nm, 0.0, 0.0, "unconfigured -> zero torque");
}

// ------------------------------------------------- the safety envelope

void TestSafetyChainPrecedence() {
  std::printf("BalancingController: safety chain order\n");

  // src/core/BalancingController.cpp PART A is an if/else-if chain, not
  // five independent ifs: exactly one condition acts per cycle and the
  // FIRST one in the chain is what gets reported.  That report is the
  // diagnosis someone reads off the terminal after a crash, so which one
  // wins is a contract, not an implementation detail.

  // Motor fault beats tilt, even with both true.
  {
    cube::BalancingController c(MakeControlConfig());
    const cube::ControlOutput out = c.update(MakeBody(9.0, 0.0, 0.0), MakeMotor(33));
    CheckSafety(out, cube::SafetyState::kMotorFault, "motor fault beats tilt");
  }

  // Tilt beats wheel saturation.
  {
    cube::BalancingController c(MakeControlConfig());
    const cube::ControlOutput out = c.update(MakeBody(9.0, 0.0, 9999.0), MakeMotor());
    CheckSafety(out, cube::SafetyState::kTiltExceeded, "tilt beats wheel saturation");
  }

  // Wheel saturation beats a dropped controller reply -- a dropped reply
  // during a fall is usually a symptom of the fall, not its cause.
  {
    cube::BalancingController c(MakeControlConfig());
    cube::MotorState dead;
    dead.valid = false;
    const cube::ControlOutput out = c.update(MakeBody(0.0, 0.0, 9999.0), dead);
    CheckSafety(out, cube::SafetyState::kWheelSaturated, "wheel saturation beats sensor");
  }

  // A fault code on an INVALID motor reading is not a motor fault: there
  // was no reply, so the fault field is stale.  It falls through to the
  // dropped-reply trip instead.
  {
    cube::BalancingController c(MakeControlConfig());
    cube::MotorState stale;
    stale.valid = false;
    stale.fault = 33;
    const cube::ControlOutput out = c.update(MakeBody(0.0, 0.0, 0.0), stale);
    CheckSafety(out, cube::SafetyState::kSensorFault, "fault on invalid reply -> kSensorFault");
  }
}

void TestTiltIsStrictlyGreater() {
  std::printf("BalancingController: tilt e-stop boundary\n");

  // Exactly at the limit is still inside it.  Worth pinning because the
  // e-stop threshold is the number an operator reasons about in degrees,
  // and an off-by-one-comparison here is invisible until the rig is at the
  // edge of its envelope.
  cube::BalancingController at(MakeControlConfig());
  const cube::ControlOutput ok = at.update(MakeBody(0.25, 0.0, 0.0), MakeMotor());
  Check(ok.armed, "theta == max_tilt_rad stays armed");

  cube::BalancingController past(MakeControlConfig());
  const cube::ControlOutput trip =
      past.update(MakeBody(0.2500001, 0.0, 0.0), MakeMotor());
  CheckSafety(trip, cube::SafetyState::kTiltExceeded, "theta just past max trips");
}

void TestTripLatchesAndFirstTripWins() {
  std::printf("BalancingController: the latch\n");

  cube::BalancingController c(MakeControlConfig());
  const cube::ControlOutput first = c.update(MakeBody(9.0, 0.0, 0.0), MakeMotor());
  CheckSafety(first, cube::SafetyState::kTiltExceeded, "trips on tilt");

  // Now present a perfectly healthy state.  It must stay tripped: re-arming
  // is a deliberate human act, and a controller that recovers on its own is
  // a controller that re-energises a rig nobody has looked at yet.
  const cube::ControlOutput after = c.update(MakeBody(0.0, 0.0, 0.0), MakeMotor());
  Check(!after.armed, "stays disarmed on a healthy cycle");
  CheckSafety(after, cube::SafetyState::kTiltExceeded, "keeps the ORIGINAL reason");
  CheckNear(after.torque_nm, 0.0, 0.0, "disarmed -> zero torque");

  // A motor fault arriving on the way down must not overwrite the tilt
  // diagnosis -- the tilt is what actually happened first.
  const cube::ControlOutput later = c.update(MakeBody(0.0, 0.0, 0.0), MakeMotor(33));
  CheckSafety(later, cube::SafetyState::kTiltExceeded, "later fault does not overwrite");

  c.reset();
  Check(c.armed(), "reset() re-arms");
  const cube::ControlOutput back = c.update(MakeBody(0.0, 0.0, 0.0), MakeMotor());
  Check(back.armed, "commands again after reset()");
  CheckSafety(back, cube::SafetyState::kOk, "reset() clears the reason");
}

void TestNotReadyIsNotATrip() {
  std::printf("BalancingController: kNotReady vs kSensorFault\n");

  cube::BalancingController c(MakeControlConfig());
  cube::BodyState warming;  // valid == false, never yet valid
  warming.valid = false;

  // Ordinary startup: the estimator has not converged.  Zero torque, but
  // still armed -- this is not a fault and must not need a human to clear.
  LoopGuard guard;
  for (int i = 0; i < 50; i++) {
    const cube::ControlOutput out = c.update(warming, MakeMotor());
    Check(out.armed, "warming up stays armed");
    CheckSafety(out, cube::SafetyState::kNotReady, "warming up -> kNotReady");
    CheckNear(out.torque_nm, 0.0, 0.0, "warming up -> zero torque");
    if (guard.failed()) { break; }  // don't print 50 identical failures
  }
  Check(c.armed(), "50 never-valid cycles did not trip");
}

void TestMidFlightSensorLossTrips() {
  std::printf("BalancingController: sensor death after a valid estimate\n");

  // The gap Config::max_invalid_cycles exists to close.  Once the estimator
  // HAS been valid, an invalid run is a sensor dying, not a startup -- and
  // reporting kNotReady for it forever reads to an operator as "still
  // warming up" while the rig coasts at zero torque and falls.
  cube::BalancingController c(MakeControlConfig());
  const cube::ControlOutput good = c.update(MakeBody(0.0, 0.0, 0.0), MakeMotor());
  CheckSafety(good, cube::SafetyState::kOk, "one good cycle first");

  cube::BodyState lost;
  lost.valid = false;

  // max_invalid_cycles == 3, and the comparison is `++count > max`, so
  // three cycles are tolerated and the fourth trips.
  for (int i = 1; i <= 3; i++) {
    const cube::ControlOutput out = c.update(lost, MakeMotor());
    Check(out.armed, "within the grace period, still armed");
    CheckSafety(out, cube::SafetyState::kNotReady, "within the grace period, kNotReady");
  }
  const cube::ControlOutput dead = c.update(lost, MakeMotor());
  Check(!dead.armed, "past the grace period, disarmed");
  CheckSafety(dead, cube::SafetyState::kSensorFault, "past the grace period, kSensorFault");

  // A good cycle in the middle of a run of misses must clear the count, or
  // an occasional dropped read accumulates into a trip over minutes.
  cube::BalancingController d(MakeControlConfig());
  d.update(MakeBody(0.0, 0.0, 0.0), MakeMotor());
  for (int round = 0; round < 5; round++) {
    d.update(lost, MakeMotor());
    d.update(lost, MakeMotor());
    d.update(MakeBody(0.0, 0.0, 0.0), MakeMotor());
  }
  Check(d.armed(), "intermittent misses do not accumulate into a trip");
}

void TestResetClearsTheInvalidCounter() {
  std::printf("BalancingController: reset() clears the sensor counters too\n");

  // reset() has to clear ever_valid_/invalid_count_, not just the latch.
  // Leaving them set means the re-armed controller carries a partially
  // spent grace period into the next run and trips early.
  cube::BalancingController c(MakeControlConfig());
  c.update(MakeBody(0.0, 0.0, 0.0), MakeMotor());

  cube::BodyState lost;
  lost.valid = false;
  c.update(lost, MakeMotor());
  c.update(lost, MakeMotor());
  c.update(lost, MakeMotor());
  c.update(lost, MakeMotor());
  Check(!c.armed(), "tripped as expected");

  c.reset();
  // After reset, the estimator counts as never-valid again, so an invalid
  // state is kNotReady indefinitely rather than an immediate re-trip.
  LoopGuard guard;
  for (int i = 0; i < 10; i++) {
    const cube::ControlOutput out = c.update(lost, MakeMotor());
    CheckSafety(out, cube::SafetyState::kNotReady, "after reset, invalid is kNotReady again");
    if (guard.failed()) { break; }
  }
}

// ------------------------------------------------- the control law

void TestSignIsStabilising() {
  std::printf("BalancingController: the control law opposes the tilt\n");

  // THE most important check in this file.  A sign error here does not
  // produce a rig that balances badly; it produces a rig that drives its
  // own fall at full authority.  With k_theta = 1 and theta = +0.1, the
  // torque must be NEGATIVE.
  cube::BalancingController c(MakeControlConfig());
  const cube::ControlOutput pos = c.update(MakeBody(0.1, 0.0, 0.0), MakeMotor());
  CheckNear(pos.torque_nm, -0.1, 1e-12, "theta = +0.1 -> tau = -0.1");
  CheckNear(pos.term_theta, 0.1, 1e-12, "term_theta is pre-negation");

  cube::BalancingController d(MakeControlConfig());
  const cube::ControlOutput neg = d.update(MakeBody(-0.1, 0.0, 0.0), MakeMotor());
  CheckNear(neg.torque_nm, 0.1, 1e-12, "theta = -0.1 -> tau = +0.1");
}

void TestThreeTermsAreReportedSeparately() {
  std::printf("BalancingController: per-term reporting\n");

  // Without these, gain tuning is blind -- you cannot tell which term
  // saturated the output.  They are captured pre-scale and pre-clamp, so
  // they stay comparable across runs at different gain_scale.
  cube::BalancingController::Config cfg = MakeControlConfig();
  cfg.k_theta = 2.0;
  cfg.k_theta_dot = 0.3;
  cfg.k_omega = 0.001;
  cube::BalancingController c(cfg);

  const cube::ControlOutput out = c.update(MakeBody(0.1, 0.5, 100.0), MakeMotor(), 0.5);
  CheckNear(out.term_theta, 0.2, 1e-12, "term_theta = k*theta");
  CheckNear(out.term_theta_dot, 0.15, 1e-12, "term_theta_dot = k*theta_dot");
  CheckNear(out.term_omega, 0.1, 1e-12, "term_omega = k*wheel_omega");
  // Terms unscaled, output scaled.
  CheckNear(out.torque_nm, -0.5 * 0.45, 1e-12, "torque = -sum * gain_scale");
  Check(out.gain_scaled, "gain_scale < 1 sets gain_scaled");
}

void TestGainScaleScalesTheSumNotTheGains() {
  std::printf("BalancingController: gain_scale\n");

  cube::BalancingController c(MakeControlConfig());
  const cube::ControlOutput full = c.update(MakeBody(0.1, 0.0, 0.0), MakeMotor(), 1.0);
  CheckNear(full.torque_nm, -0.1, 1e-12, "gain_scale 1.0 is the nominal law");
  Check(!full.gain_scaled, "gain_scale == 1 does not set gain_scaled");

  const cube::ControlOutput soft = c.update(MakeBody(0.1, 0.0, 0.0), MakeMotor(), 0.1);
  CheckNear(soft.torque_nm, -0.01, 1e-12, "gain_scale 0.1 softens uniformly");
  Check(soft.gain_scaled, "gain_scale < 1 sets gain_scaled");

  const cube::ControlOutput zero = c.update(MakeBody(0.1, 0.0, 0.0), MakeMotor(), 0.0);
  CheckNear(zero.torque_nm, 0.0, 1e-12, "gain_scale 0 commands nothing");
  Check(zero.armed, "gain_scale 0 is still armed -- it is not a trip");
}

void TestDeadbandAppliesToThetaOnly() {
  std::printf("BalancingController: theta deadband\n");

  // Deadbanding theta_dot or wheel_omega too would leave the cube undamped
  // exactly where damping matters most -- near vertical, where it spends
  // all of its time.
  cube::BalancingController::Config cfg = MakeControlConfig();
  cfg.theta_deadband = 0.01;
  cfg.k_theta_dot = 1.0;
  cube::BalancingController c(cfg);

  const cube::ControlOutput inside = c.update(MakeBody(0.005, 0.5, 0.0), MakeMotor());
  CheckNear(inside.term_theta, 0.0, 1e-12, "inside the deadband, theta term is zeroed");
  CheckNear(inside.term_theta_dot, 0.5, 1e-12, "theta_dot is NOT deadbanded");
  CheckNear(inside.torque_nm, -0.5, 1e-12, "damping survives the deadband");

  const cube::ControlOutput outside = c.update(MakeBody(0.02, 0.0, 0.0), MakeMotor());
  CheckNear(outside.term_theta, 0.02, 1e-12, "outside the deadband, theta term is live");
}

void TestWheelTaperFadesSpinUpOnly() {
  std::printf("BalancingController: the wheel-speed taper\n");

  // max_wheel_omega 300, wheel_taper_start 255 -> a 45 rad/s fade region.
  // At |omega| = 280 the factor is (300 - 280) / 45 = 0.4444...
  const double kExpected = (300.0 - 280.0) / 45.0;

  // Spinning up: torque and wheel speed share a sign.  Faded.
  {
    cube::BalancingController c(MakeControlConfig());
    const cube::ControlOutput out = c.update(MakeBody(-0.1, 0.0, 280.0), MakeMotor());
    Check(out.wheel_taper_active, "spin-up near the cap is tapered");
    CheckNear(out.torque_nm, 0.1 * kExpected, 1e-12, "spin-up torque is faded");
  }

  // Braking: opposite signs.  NEVER faded -- this is the recovery action
  // the taper exists to preserve.  A panel-stage test of this law saw the
  // wheel reach 39.7 of a 40 rad/s cap on a legitimate recovery, which is
  // exactly the case a hard trip alone punishes.
  {
    cube::BalancingController c(MakeControlConfig());
    const cube::ControlOutput out = c.update(MakeBody(0.1, 0.0, 280.0), MakeMotor());
    Check(!out.wheel_taper_active, "braking near the cap is not tapered");
    CheckNear(out.torque_nm, -0.1, 1e-12, "braking torque is untouched");
  }

  // Below the fade region: no taper at all.
  {
    cube::BalancingController c(MakeControlConfig());
    const cube::ControlOutput out = c.update(MakeBody(-0.1, 0.0, 100.0), MakeMotor());
    Check(!out.wheel_taper_active, "below wheel_taper_start there is no fade");
    CheckNear(out.torque_nm, 0.1, 1e-12, "torque is unfaded well below the cap");
  }

  // Misconfigured: wheel_taper_start at or past max_wheel_omega leaves no
  // region to divide into.  Defined as "no taper" rather than letting a
  // non-positive denominator produce 0, a sign flip, or a 0/0 NaN right at
  // the cap.  The hard trip is still the real backstop either way.
  {
    cube::BalancingController::Config cfg = MakeControlConfig();
    cfg.wheel_taper_start = cfg.max_wheel_omega;
    cube::BalancingController c(cfg);
    const cube::ControlOutput out = c.update(MakeBody(-0.1, 0.0, 299.0), MakeMotor());
    Check(!out.wheel_taper_active, "span == 0 -> no taper");
    CheckNear(out.torque_nm, 0.1, 1e-12, "span == 0 -> torque unmodified");
    Check(std::isfinite(out.torque_nm), "span == 0 -> finite (not 0/0)");
  }
  {
    cube::BalancingController::Config cfg = MakeControlConfig();
    cfg.wheel_taper_start = cfg.max_wheel_omega + 50.0;
    cube::BalancingController c(cfg);
    const cube::ControlOutput out = c.update(MakeBody(-0.1, 0.0, 299.0), MakeMotor());
    CheckNear(out.torque_nm, 0.1, 1e-12, "span < 0 -> torque unmodified");
  }
}

void TestTorqueClampAndItsFlag() {
  std::printf("BalancingController: the torque clamp\n");

  cube::BalancingController::Config cfg = MakeControlConfig();
  cfg.k_theta = 100.0;
  cfg.max_torque_nm = 0.2;
  cube::BalancingController c(cfg);

  const cube::ControlOutput hot = c.update(MakeBody(0.1, 0.0, 0.0), MakeMotor());
  CheckNear(hot.torque_nm, -0.2, 1e-12, "clamped to -max_torque_nm");
  Check(hot.torque_clamped, "clamping sets torque_clamped");
  CheckNear(hot.term_theta, 10.0, 1e-12, "the term still reports the pre-clamp value");

  const cube::ControlOutput cool = c.update(MakeBody(0.0005, 0.0, 0.0), MakeMotor());
  CheckNear(cool.torque_nm, -0.05, 1e-12, "inside the clamp, untouched");
  Check(!cool.torque_clamped, "no clamping -> flag clear");
}

void TestNonFiniteTorqueTrips() {
  std::printf("BalancingController: the non-finite guard\n");

  // A NaN reaching moteus's feedforward_torque field is not a no-op, it is
  // an invalid command -- and std::clamp would pass a NaN straight through,
  // because every comparison against NaN is false.  Note the tilt e-stop
  // does NOT catch this for the same reason (fabs(NaN) > limit is false),
  // so this guard is the only thing between a NaN estimate and the wire.
  cube::BalancingController c(MakeControlConfig());
  const cube::ControlOutput out =
      c.update(MakeBody(std::nan(""), 0.0, 0.0), MakeMotor());

  Check(!out.armed, "NaN theta -> disarmed");
  CheckSafety(out, cube::SafetyState::kSensorFault, "NaN theta -> kSensorFault");
  CheckNear(out.torque_nm, 0.0, 0.0, "NaN theta -> zero torque, not NaN");
  Check(std::isfinite(out.torque_nm), "the commanded torque is finite");

  // Same for an infinite one, and for a NaN arriving via gain_scale.
  cube::BalancingController d(MakeControlConfig());
  const cube::ControlOutput inf =
      d.update(MakeBody(0.0, 0.0, 0.0), MakeMotor(), std::nan(""));
  Check(!inf.armed, "NaN gain_scale -> disarmed");
  Check(std::isfinite(inf.torque_nm), "NaN gain_scale -> finite torque");
}

// ============================================================= ESTIMATOR

void TestEstimatorUnsetFieldsAreNamed() {
  std::printf("StateEstimator: firstUnsetField names each sentinel\n");

  cube::StateEstimator::Config c;
  const cube::StateEstimator all_unset(c);
  Check(all_unset.firstUnsetField() != nullptr &&
            std::string(all_unset.firstUnsetField()) == "tau",
        "tau reported first");
  Check(!all_unset.isConfigured(), "sentinels -> not configured");

  // invert_theta is deliberately absent from this chain: it is a bool, so
  // there is no value that can mean "unset".  Getting it wrong is the most
  // dangerous single mistake in the project and NO code can catch it --
  // only tilting the rig and watching the sign can.  See StateEstimator.hpp.
  const cube::StateEstimator ok(MakeEstimatorConfig());
  Check(ok.firstUnsetField() == nullptr, "fully set -> nullptr");
  Check(ok.isConfigured(), "fully set -> isConfigured()");
}

void TestAccelAngleGeometry() {
  std::printf("StateEstimator: accelAngle\n");

  const cube::StateEstimator e(MakeEstimatorConfig());

  CheckNear(e.accelAngle(MakeImu(0.0)), 0.0, 1e-12, "upright -> 0");
  CheckNear(e.accelAngle(MakeImu(0.1)), 0.1, 1e-12, "tilted +0.1 -> +0.1");
  CheckNear(e.accelAngle(MakeImu(-0.1)), -0.1, 1e-12, "tilted -0.1 -> -0.1");

  // atan2, not asin: the magnitude cancels in the ratio, so an
  // uncalibrated scale factor does not bias the angle.
  cube::ImuData half = MakeImu(0.1);
  half.accel_y *= 0.5;
  half.accel_z *= 0.5;
  CheckNear(e.accelAngle(half), 0.1, 1e-12, "angle is independent of magnitude");
}

void TestOffsetIsSubtractedBeforeInversion() {
  std::printf("StateEstimator: theta_offset vs invert_theta ordering\n");

  // theta_offset is measured in the sensor's OWN raw sign convention -- it
  // is a reading taken at the balance point -- so it has to be removed
  // before that convention is flipped.  The other order applies the offset
  // with the wrong sign and biases the cube toward one side, which on a
  // rig looks like a mysterious permanent lean rather than a code bug.
  cube::StateEstimator::Config cfg = MakeEstimatorConfig();
  cfg.theta_offset = 0.05;
  cfg.invert_theta = true;
  const cube::StateEstimator e(cfg);

  // Correct:   -(0.1 - 0.05) = -0.05
  // Backwards:  (-0.1) - 0.05 = -0.15
  CheckNear(e.accelAngle(MakeImu(0.1)), -0.05, 1e-12,
            "offset removed before the sign flip");
}

void TestSeedsFromAccelerometerOnFirstSample() {
  std::printf("StateEstimator: seed on the first sample\n");

  // Starting from theta = 0 would make the filter walk to the true angle
  // over several tau, while the controller is already acting on the wrong
  // estimate.  With tau = 0.5 and dt = 0.0025, alpha = 0.995: an unseeded
  // filter would report ~0.0005 rad here instead of 0.1.
  cube::StateEstimator e(MakeEstimatorConfig());
  const cube::BodyState s = e.update(MakeImu(0.1), cube::MotorState{}, 0.0025);
  CheckNear(s.theta, 0.1, 1e-9, "first sample lands on the accelerometer angle");

  e.reset();
  const cube::BodyState after = e.update(MakeImu(-0.2), cube::MotorState{}, 0.0025);
  CheckNear(after.theta, -0.2, 1e-9, "reset() re-seeds on the next sample");
}

void TestRejectsBadDt() {
  std::printf("StateEstimator: the dt gate\n");

  // The bound is +/-2x nominal_dt, NOT a fixed constant.  A fixed 0.1 s
  // ceiling would be ~40x nominal at 400 Hz, waving through a run of dozens
  // of missed cycles as though it were one slightly-late one.
  cube::StateEstimator::Config cfg = MakeEstimatorConfig();
  cfg.warmup_samples = 1;
  const double n = cfg.nominal_dt;  // 0.0025

  struct Case { double dt; bool want_valid; const char* what; };
  const Case cases[] = {
      {n,            true,  "dt == nominal is accepted"},
      {0.5 * n,      true,  "dt == 0.5x nominal is accepted (boundary)"},
      {2.0 * n,      true,  "dt == 2x nominal is accepted (boundary)"},
      {0.4 * n,      false, "dt < 0.5x nominal is rejected (duplicated cycle)"},
      {2.1 * n,      false, "dt > 2x nominal is rejected (missed cycles)"},
      {0.0,          false, "dt == 0 is rejected"},
      {-n,           false, "negative dt is rejected"},
  };

  for (const Case& c : cases) {
    cube::StateEstimator e(cfg);
    e.update(MakeImu(0.0), cube::MotorState{}, n);  // seed and warm up
    const cube::BodyState s = e.update(MakeImu(0.0), cube::MotorState{}, c.dt);
    Check(s.valid == c.want_valid, c.what);
  }

  // An invalid IMU read is rejected the same way -- a dropped SPI/I2C word
  // is not evidence the cube stopped moving.
  cube::StateEstimator e(cfg);
  e.update(MakeImu(0.0), cube::MotorState{}, n);
  cube::ImuData bad = MakeImu(0.3);
  bad.valid = false;
  const cube::BodyState s = e.update(bad, cube::MotorState{}, n);
  Check(!s.valid, "an invalid IMU sample is rejected");
  CheckNear(s.theta, 0.0, 1e-9, "a rejected sample holds the previous estimate");
}

void TestWheelSpeedSurvivesABadImuRead() {
  std::printf("StateEstimator: wheel speed is independent of the IMU\n");

  // Step 1 runs BEFORE the reject-bad-input gate on purpose: a stale
  // accelerometer read is not a reason to also hold a perfectly good CAN
  // reply.  The k_omega term is what keeps the wheel off its saturation
  // limit, so freezing it during an IMU hiccup is the wrong failure.
  cube::StateEstimator e(MakeEstimatorConfig());
  cube::MotorState m;
  m.valid = true;
  m.velocity_rev_s = 2.0;

  cube::ImuData bad = MakeImu(0.0);
  bad.valid = false;
  const cube::BodyState s = e.update(bad, m, 0.0025);

  Check(!s.valid, "the estimate is still reported invalid");
  CheckNear(s.wheel_omega, 2.0 * 2.0 * kPi, 1e-9, "wheel speed still updated");
}

void TestAccelTrustGate() {
  std::printf("StateEstimator: the accelerometer trust gate\n");

  // During a hard correction the measured vector is mostly wheel reaction,
  // not gravity.  Blending it would drag theta toward whatever direction
  // the cube happened to be accelerating.
  cube::StateEstimator::Config cfg = MakeEstimatorConfig();
  cfg.warmup_samples = 1;
  cube::StateEstimator e(cfg);

  // Seed at 0.
  e.update(MakeImu(0.0), cube::MotorState{}, cfg.nominal_dt);

  // A sample claiming a big tilt, but at 2 g -- well outside the +/-25%
  // gate.  The gyro reads zero, so a TRUSTED sample would pull theta
  // toward 0.5 rad; a rejected one leaves it at the integrated value, 0.
  cube::ImuData shaken = MakeImu(0.5);
  shaken.accel_y *= 2.0;
  shaken.accel_z *= 2.0;
  const cube::BodyState s = e.update(shaken, cube::MotorState{}, cfg.nominal_dt);
  CheckNear(s.theta, 0.0, 1e-9, "an untrusted sample is not blended in");

  // The same angle at 1 g IS blended.
  const cube::BodyState t = e.update(MakeImu(0.5), cube::MotorState{}, cfg.nominal_dt);
  Check(t.theta > 0.0, "a trusted sample does pull theta toward it");
}

void TestGyroIntegrationAndInversion() {
  std::printf("StateEstimator: gyro integration and invert_theta\n");

  // invert_theta must be applied to the gyro rate as well as to the accel
  // angle.  If only one half is flipped, the two halves of the filter
  // disagree about which way is positive and fight each other -- which on
  // a rig looks like an estimate that lags and then snaps back.
  cube::StateEstimator::Config cfg = MakeEstimatorConfig();
  cfg.warmup_samples = 1;

  {
    cube::StateEstimator e(cfg);
    e.update(MakeImu(0.0), cube::MotorState{}, cfg.nominal_dt);
    const cube::BodyState s =
        e.update(MakeImu(0.0, 1.0), cube::MotorState{}, cfg.nominal_dt);
    Check(s.theta > 0.0, "positive gyro rate raises theta");
    Check(s.theta_dot > 0.0, "positive gyro rate gives positive theta_dot");
  }

  cfg.invert_theta = true;
  {
    cube::StateEstimator e(cfg);
    e.update(MakeImu(0.0), cube::MotorState{}, cfg.nominal_dt);
    const cube::BodyState s =
        e.update(MakeImu(0.0, 1.0), cube::MotorState{}, cfg.nominal_dt);
    Check(s.theta < 0.0, "invert_theta flips the gyro contribution too");
    Check(s.theta_dot < 0.0, "invert_theta flips theta_dot");
  }
}

void TestRateLowPassIsDtAware() {
  std::printf("StateEstimator: theta_dot low-pass\n");

  // rate_cutoff_hz is a FREQUENCY, not a filter coefficient, precisely so
  // it keeps its meaning when the loop rate changes.  Pick numbers that
  // make the coefficient exactly 0.5: rc = 1/(2*pi*f), and with f chosen
  // so rc == dt, beta = dt/(dt+rc) = 0.5.
  cube::StateEstimator::Config cfg = MakeEstimatorConfig();
  cfg.nominal_dt = 0.01;
  cfg.warmup_samples = 1;
  cfg.tau = 1e9;  // effectively pure gyro, so theta does not disturb this
  const double dt = 0.01;
  cfg.rate_cutoff_hz = 1.0 / (2.0 * kPi * dt);

  cube::StateEstimator e(cfg);
  e.update(MakeImu(0.0), cube::MotorState{}, dt);  // seed, theta_dot -> 0.5*0

  // theta_dot starts at 0; one step with a unit rate gives 0.5, the next
  // 0.75.  A hardcoded coefficient would not track dt and these would drift.
  const cube::BodyState a = e.update(MakeImu(0.0, 1.0), cube::MotorState{}, dt);
  CheckNear(a.theta_dot, 0.5, 1e-9, "one step of a unit rate -> 0.5");
  const cube::BodyState b = e.update(MakeImu(0.0, 1.0), cube::MotorState{}, dt);
  CheckNear(b.theta_dot, 0.75, 1e-9, "two steps -> 0.75");

  // Halving dt halves beta's numerator and doubles the effective smoothing,
  // so the same rate converges more slowly -- the point of a frequency.
  cube::StateEstimator f(cfg);
  f.update(MakeImu(0.0), cube::MotorState{}, dt);
  const cube::BodyState half =
      f.update(MakeImu(0.0, 1.0), cube::MotorState{}, 0.5 * dt);
  Check(half.theta_dot < 0.5, "a shorter dt filters more, not the same");
}

void TestWarmupGatesValidity() {
  std::printf("StateEstimator: warmup\n");

  cube::StateEstimator::Config cfg = MakeEstimatorConfig();
  cfg.warmup_samples = 5;
  cube::StateEstimator e(cfg);

  LoopGuard guard;
  for (int i = 1; i <= 4; i++) {
    const cube::BodyState s = e.update(MakeImu(0.0), cube::MotorState{}, cfg.nominal_dt);
    Check(!s.valid, "still warming up");
    if (guard.failed()) { break; }
  }
  const cube::BodyState fifth =
      e.update(MakeImu(0.0), cube::MotorState{}, cfg.nominal_dt);
  Check(fifth.valid, "valid on the warmup_samples-th sample");

  // Rejected samples must not count toward the warmup -- otherwise a run of
  // bad reads "warms up" an estimator that has seen nothing.
  cube::StateEstimator f(cfg);
  for (int i = 0; i < 20; i++) {
    cube::ImuData bad = MakeImu(0.0);
    bad.valid = false;
    f.update(bad, cube::MotorState{}, cfg.nominal_dt);
  }
  const cube::BodyState first_good =
      f.update(MakeImu(0.0), cube::MotorState{}, cfg.nominal_dt);
  Check(!first_good.valid, "rejected samples do not count toward warmup");

  // reset() puts it back to square one.
  cube::StateEstimator g(cfg);
  for (int i = 0; i < 10; i++) {
    g.update(MakeImu(0.0), cube::MotorState{}, cfg.nominal_dt);
  }
  Check(g.state().valid, "warmed up");
  g.reset();
  const cube::BodyState after = g.update(MakeImu(0.0), cube::MotorState{}, cfg.nominal_dt);
  Check(!after.valid, "reset() drops the sample count");
}

void TestNonFiniteSampleIsCaught() {
  std::printf("StateEstimator: the non-finite guard\n");

  // A NaN theta reaching BalancingController would slide straight past
  // every safety comparison -- fabs(NaN) > limit is false.  The estimator
  // has to report invalid rather than pass it on.
  cube::StateEstimator::Config cfg = MakeEstimatorConfig();
  cfg.warmup_samples = 1;

  // A NaN GYRO rate is what step 8 exists for: it goes into `predicted`,
  // which is used on both sides of the complementary blend, so it reaches
  // theta whether or not the accelerometer is trusted.
  {
    cube::StateEstimator e(cfg);
    e.update(MakeImu(0.0), cube::MotorState{}, cfg.nominal_dt);
    const cube::BodyState s =
        e.update(MakeImu(0.0, std::nan("")), cube::MotorState{}, cfg.nominal_dt);
    Check(!s.valid, "a NaN gyro rate is reported invalid");
  }

  // A NaN ACCELEROMETER reading takes a different route, and this pins it
  // because the route is not obvious: the magnitude comes out NaN, so the
  // trust gate's `fabs(magnitude - g) < gate*g` is false and the sample is
  // simply not blended in.  The filter coasts on the gyro and stays VALID.
  //
  // That is a defensible outcome -- it is exactly what the gate does for
  // any untrustworthy sample -- and it is unreachable from either real
  // driver, since both build their doubles from int16 counts and integer
  // arithmetic cannot produce a NaN.  It is recorded here so that anyone
  // who later makes the gate NaN-aware sees this expectation change
  // deliberately rather than discovering it on a rig.
  {
    cube::StateEstimator e(cfg);
    e.update(MakeImu(0.0), cube::MotorState{}, cfg.nominal_dt);
    cube::ImuData nan_accel = MakeImu(0.0);
    nan_accel.accel_y = std::nan("");
    const cube::BodyState s = e.update(nan_accel, cube::MotorState{}, cfg.nominal_dt);
    Check(s.valid, "a NaN accel is rejected by the trust gate, not step 8");
    Check(std::isfinite(s.theta), "and theta stays finite (pure gyro)");
  }
}

// ====================================================== the two together

void TestEstimatorFeedsControllerSafely() {
  std::printf("integration: a warming-up estimator does not arm the motor\n");

  // The one interaction that matters between the two modules: while the
  // estimator is warming up it reports valid == false, and the controller
  // must read that as kNotReady -- zero torque, still armed, no human
  // needed.  If either side got this wrong the rig would either refuse to
  // start after every power-on or command torque from a half-converged
  // estimate.
  cube::StateEstimator::Config ecfg = MakeEstimatorConfig();
  ecfg.warmup_samples = 10;
  cube::StateEstimator estimator(ecfg);
  cube::BalancingController controller(MakeControlConfig());

  cube::MotorState motor = MakeMotor();

  LoopGuard guard;
  for (int i = 0; i < 9; i++) {
    const cube::BodyState body =
        estimator.update(MakeImu(0.02), motor, ecfg.nominal_dt);
    const cube::ControlOutput out = controller.update(body, motor);
    CheckNear(out.torque_nm, 0.0, 0.0, "no torque while warming up");
    CheckSafety(out, cube::SafetyState::kNotReady, "kNotReady while warming up");
    if (guard.failed()) { break; }
  }

  const cube::BodyState body = estimator.update(MakeImu(0.02), motor, ecfg.nominal_dt);
  const cube::ControlOutput out = controller.update(body, motor);
  Check(body.valid, "estimator converged");
  CheckSafety(out, cube::SafetyState::kOk, "kOk once converged");
  Check(out.torque_nm < 0.0, "a positive tilt now produces a correcting torque");
}

}  // namespace

int main() {
  std::printf("=== control law tests (no hardware) ===\n");

  TestUnsetFieldsAreNamedInOrder();
  TestUnconfiguredRefusesToCommand();
  TestSafetyChainPrecedence();
  TestTiltIsStrictlyGreater();
  TestTripLatchesAndFirstTripWins();
  TestNotReadyIsNotATrip();
  TestMidFlightSensorLossTrips();
  TestResetClearsTheInvalidCounter();
  TestSignIsStabilising();
  TestThreeTermsAreReportedSeparately();
  TestGainScaleScalesTheSumNotTheGains();
  TestDeadbandAppliesToThetaOnly();
  TestWheelTaperFadesSpinUpOnly();
  TestTorqueClampAndItsFlag();
  TestNonFiniteTorqueTrips();

  TestEstimatorUnsetFieldsAreNamed();
  TestAccelAngleGeometry();
  TestOffsetIsSubtractedBeforeInversion();
  TestSeedsFromAccelerometerOnFirstSample();
  TestRejectsBadDt();
  TestWheelSpeedSurvivesABadImuRead();
  TestAccelTrustGate();
  TestGyroIntegrationAndInversion();
  TestRateLowPassIsDtAware();
  TestWarmupGatesValidity();
  TestNonFiniteSampleIsCaught();

  TestEstimatorFeedsControllerSafely();

  if (g_failures == 0) {
    std::printf("=== all tests passed ===\n");
    return 0;
  }
  std::printf("=== %d check(s) FAILED ===\n", g_failures);
  return 1;
}
