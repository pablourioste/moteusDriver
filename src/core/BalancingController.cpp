#include "core/BalancingController.hpp"

#include <algorithm>
#include <cmath>

namespace cube {

// --- Implemented: plumbing, no judgement involved --------------------------

const char* ToString(SafetyState state) {
  switch (state) {
    case SafetyState::kOk: return "OK";
    case SafetyState::kNotReady: return "NOT_READY";
    case SafetyState::kTiltExceeded: return "TILT_EXCEEDED";
    case SafetyState::kWheelSaturated: return "WHEEL_SATURATED";
    case SafetyState::kSensorFault: return "SENSOR_FAULT";
    case SafetyState::kMotorFault: return "MOTOR_FAULT";
    case SafetyState::kUnconfigured: return "UNCONFIGURED";
  }
  return "UNKNOWN";
}

BalancingController::BalancingController() = default;

BalancingController::BalancingController(const Config& config)
    : config_(config) {}

const char* BalancingController::firstUnsetField() const {
  if (std::isnan(config_.k_theta)) { return "k_theta"; }
  if (std::isnan(config_.k_theta_dot)) { return "k_theta_dot"; }
  if (std::isnan(config_.k_omega)) { return "k_omega"; }
  if (std::isnan(config_.max_tilt_rad)) { return "max_tilt_rad"; }
  if (std::isnan(config_.max_torque_nm)) { return "max_torque_nm"; }
  if (std::isnan(config_.max_wheel_omega)) { return "max_wheel_omega"; }
  if (std::isnan(config_.wheel_taper_start)) { return "wheel_taper_start"; }
  if (std::isnan(config_.theta_deadband)) { return "theta_deadband"; }
  if (config_.max_invalid_cycles < 0) { return "max_invalid_cycles"; }
  return nullptr;
}

bool BalancingController::isConfigured() const {
  return firstUnsetField() == nullptr;
}

void BalancingController::reset() {
  armed_ = true;
  safety_ = SafetyState::kOk;
  ever_valid_ = false;
  invalid_count_ = 0;
}

void BalancingController::trip(SafetyState reason) {
  // First trip wins.  A tilt e-stop followed by the motor faulting on the
  // way down should still report the tilt as the cause.
  if (armed_) {
    armed_ = false;
    safety_ = reason;
  }
}

// --- The part you write ----------------------------------------------------

ControlOutput BalancingController::update(const BodyState& state,
                                          const MotorState& motor,
                                          double gain_scale) {
  ControlOutput out;

  // Implemented guard: an unconfigured controller must never command torque.
  // This runs before anything else so a missing gain cannot be masked by a
  // later check passing.
  if (!isConfigured()) {
    out.armed = false;
    out.safety = SafetyState::kUnconfigured;
    out.torque_nm = 0.0;
    return out;
  }

  // Implemented: once tripped, stay tripped.  Re-arming is a deliberate
  // human act, so this check has to come before the control law can put the
  // rig back into motion.
  if (!armed_) {
    out.armed = false;
    out.safety = safety_;
    out.torque_nm = 0.0;
    return out;
  }

  // PART A -- THE SAFETY ENVELOPE.  Evaluated before any control math, so a
  // rig that is already past saving never gets another torque command.
  //
  // This is an if/else-if chain, not five independent ifs: only one
  // condition can act per cycle, and whichever comes first in the chain is
  // the one that wins and gets reported -- that is the diagnosis you read
  // off the terminal after a crash.  The order encodes "physically
  // unrecoverable" first:
  //
  //   1. The board already stopped driving and is telling you why.
  //   2. An invalid state is the normal startup condition while the
  //      estimator warms up -- NOT a trip, while it has never yet been
  //      valid.  But an estimate that WAS valid and has now gone invalid for
  //      too many consecutive cycles is a sensor dying mid-flight, not a
  //      startup condition, and reporting kNotReady for that forever would
  //      read to an operator as "still warming up" while the rig coasts at
  //      zero torque and falls.  So it counts, and trips kSensorFault past
  //      the grace period -- but only once ever_valid_ is true, so ordinary
  //      startup is unaffected.
  //   3. Past the tilt e-stop, a reaction wheel cannot recover the cube;
  //      continuing to drive only adds energy to the fall.
  //   4. Past wheel saturation there is no control authority left --
  //      stopping now is safer than spinning further into the motor's real
  //      limit.
  //   5. A dropped controller reply is checked last because during a fall
  //      it is usually a symptom of the fall, not the cause of it.
  if (motor.valid && motor.fault != 0) {
    trip(SafetyState::kMotorFault);
  } else if (!state.valid) {
    if (ever_valid_ && ++invalid_count_ > config_.max_invalid_cycles) {
      trip(SafetyState::kSensorFault);
    } else {
      out.armed = true;
      out.safety = SafetyState::kNotReady;
      out.torque_nm = 0.0;
      return out;
    }
  } else {
    ever_valid_ = true;
    invalid_count_ = 0;
    if (std::fabs(state.theta) > config_.max_tilt_rad) {
      trip(SafetyState::kTiltExceeded);
    } else if (std::fabs(state.wheel_omega) > config_.max_wheel_omega) {
      trip(SafetyState::kWheelSaturated);
    } else if (!motor.valid) {
      trip(SafetyState::kSensorFault);
    }
  }

  if (!armed_) {
    out.armed = false;
    out.safety = safety_;
    out.torque_nm = 0.0;
    return out;
  }

  // PART B -- THE CONTROL LAW.
  //
  // Deadband on theta only.  Deadbanding theta_dot or wheel_omega too would
  // leave the cube undamped exactly where damping matters most -- near
  // vertical, where it spends all its time.  Config::theta_deadband's
  // comment has the reason to prefer 0.0 here unless chatter is measured.
  const double theta =
      (std::fabs(state.theta) < config_.theta_deadband) ? 0.0 : state.theta;

  // Terms captured individually, pre-scale and pre-clamp, so tuning can see
  // which one dominates regardless of what gain_scale this cycle ran at.
  out.term_theta = config_.k_theta * theta;
  out.term_theta_dot = config_.k_theta_dot * state.theta_dot;
  out.term_omega = config_.k_omega * state.wheel_omega;

  // Sum, negated -- THE NEGATIVE SIGN IS WHAT MAKES THIS STABILISING, torque
  // opposes the tilt.  If the cube accelerates its own fall on the bench, do
  // NOT flip this sign to compensate.  The bug is in the estimator's sign
  // convention (invert_theta) or the encoder sign (SOURCE0_SIGN); flipping
  // it here only hides that bug and leaves the damping term pointing the
  // wrong way.
  double torque = -(out.term_theta + out.term_theta_dot + out.term_omega);

  // gain_scale applies to the WHOLE sum, never to individual gains --
  // scaling them separately would move the closed-loop poles instead of
  // uniformly softening the response.  Caller's responsibility to have
  // clamped it to [0, 1]; a bad value here (e.g. NaN) is caught by the
  // finite check below like any other bad input.
  torque *= gain_scale;
  out.gain_scaled = (gain_scale < 1.0);

  // Wheel-speed taper.  Between wheel_taper_start and max_wheel_omega, fade
  // torque that would spin the wheel up further in its current direction;
  // torque that brakes it is never faded, since that is the recovery action
  // the taper exists to preserve.  This is what lets a recovery use the
  // full envelope up to max_wheel_omega without nuisance-tripping
  // kWheelSaturated on a correction that was still working.
  const bool spinning_up = (torque >= 0.0) == (state.wheel_omega >= 0.0);
  if (spinning_up) {
    const double span = config_.max_wheel_omega - config_.wheel_taper_start;
    // span <= 0 means wheel_taper_start is misconfigured at or past
    // max_wheel_omega -- there is no fade region to divide into.  Rather
    // than let that division produce whatever a non-positive denominator
    // happens to give (0, a sign flip, or a 0/0 NaN right at the cap),
    // define it as "no taper": the hard trip above is still the real
    // backstop either way.
    const double s =
        (span > 0.0) ? std::clamp((config_.max_wheel_omega -
                                   std::fabs(state.wheel_omega)) / span,
                                  0.0, 1.0)
                     : 1.0;
    out.wheel_taper_active = (s < 1.0);
    torque *= s;
  }

  // Guard against a non-finite result before it reaches the clamp -- NaN
  // compares false against everything, so clamp() would silently pass a NaN
  // straight through.  A NaN reaching moteus's feedforward_torque field is
  // not a no-op, it is an invalid command, and that is not something to
  // discover live on the rig.
  if (!std::isfinite(torque)) {
    trip(SafetyState::kSensorFault);
    out.armed = false;
    out.safety = safety_;
    out.torque_nm = 0.0;
    return out;
  }

  const double clamped =
      std::clamp(torque, -config_.max_torque_nm, config_.max_torque_nm);
  out.torque_clamped = (clamped != torque);
  torque = clamped;

  out.armed = true;
  out.safety = SafetyState::kOk;
  out.torque_nm = torque;
  return out;
}

}  // namespace cube
