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
  if (std::isnan(config_.theta_deadband)) { return "theta_deadband"; }
  return nullptr;
}

bool BalancingController::isConfigured() const {
  return firstUnsetField() == nullptr;
}

void BalancingController::reset() {
  armed_ = true;
  safety_ = SafetyState::kOk;
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
                                          const MotorState& motor) {
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

  // ==== TODO(you): implement ==============================================
  //
  // PART A -- THE SAFETY ENVELOPE.  Evaluate before any control math, so a
  // rig that is already past saving never gets another torque command.
  //
  // Order matters: the checks that mean "physically unrecoverable" come
  // first, because whichever trips first is what gets reported, and that is
  // the diagnosis you will read off the terminal after a crash.
  //
  //   1. motor.valid && motor.fault != 0   -> trip(kMotorFault)
  //         The board has already stopped driving; it is telling you why.
  //
  //   2. !state.valid                      -> NOT a trip.  Return
  //         kNotReady with zero torque, still armed.  This is the normal
  //         startup state while the estimator warms up, and latching on it
  //         would mean the rig could never start.
  //
  //   3. |state.theta| > max_tilt_rad      -> trip(kTiltExceeded)
  //         The e-stop.  Past this a reaction wheel cannot recover the cube
  //         and continuing to drive only adds energy to the fall.
  //
  //   4. |state.wheel_omega| > max_wheel_omega -> trip(kWheelSaturated)
  //         No remaining control authority.  Stopping now is safer than
  //         spinning further into the motor's real limit.
  //
  //   5. !motor.valid                      -> trip(kSensorFault)
  //         The controller stopped answering.  Checked last because a
  //         dropped reply during a fall is a symptom, not the cause.
  //
  // After the checks, if !armed_, return zero torque with safety_.
  //
  // PART B -- THE CONTROL LAW.
  //
  //   Deadband, on theta ONLY:
  //     theta = (fabs(state.theta) < theta_deadband) ? 0.0 : state.theta;
  //
  //   Do NOT apply it to theta_dot or wheel_omega.  Deadbanding the rate
  //   term leaves the cube undamped exactly where damping matters most --
  //   near vertical, where it spends all its time.
  //
  //   Terms (store them in `out` so tuning can see which dominates):
  //     out.term_theta     = k_theta     * theta;
  //     out.term_theta_dot = k_theta_dot * state.theta_dot;
  //     out.term_omega     = k_omega     * state.wheel_omega;
  //
  //   Sum, negated:
  //     torque = -(term_theta + term_theta_dot + term_omega);
  //
  //   THE NEGATIVE SIGN IS WHAT MAKES THIS STABILISING -- torque opposes the
  //   tilt.  If the cube accelerates its own fall, do NOT flip this sign.
  //   The bug is in the estimator's sign convention (invert_theta) or the
  //   encoder sign (SOURCE0_SIGN); flipping here hides it and leaves the
  //   damping term pointing the wrong way.
  //
  //   Clamp, recording whether it bit:
  //     clamped = clamp(torque, -max_torque_nm, +max_torque_nm);
  //     out.torque_clamped = (clamped != torque);
  //
  //   Finally, guard against a non-finite result:
  //     if (!isfinite(torque)) trip(kSensorFault) and return zero.
  //
  //   A NaN reaching moteus's feedforward_torque field is not a no-op -- it
  //   is an invalid command, and the controller's response to it is not
  //   something to discover on a live rig.
  // ========================================================================
  (void)state;
  (void)motor;

  // Until implemented: inert and visibly so.  Reporting kNotReady with
  // armed=false means the balancer stops rather than silently commanding
  // zero torque as though that were a real control decision.
  out.armed = false;
  out.safety = SafetyState::kNotReady;
  out.torque_nm = 0.0;
  return out;
}

}  // namespace cube
