// Reaction wheel balancing control law, plus the safety envelope around it.
//
// STATUS: implemented.  The safety envelope and update() are in
// src/core/BalancingController.cpp.  What is still outstanding is
// MEASUREMENT, not code: every Config field below ships as a NaN/-1
// sentinel and has to be measured or computed before anything will run.
// See the per-field TODO(you) notes.
//
// THE CONTROL LAW
//
// Full state feedback on x = [theta, theta_dot, wheel_omega]:
//
//     tau = -(k_theta*theta + k_theta_dot*theta_dot + k_omega*wheel_omega)
//
// The first two terms catch the fall -- that much is an ordinary inverted
// pendulum.  The third is what makes a reaction wheel different from a
// pendulum on a cart, and it is worth understanding before you tune it.
//
// WHY THREE STATES AND NOT TWO
//
// A cart can push against the ground indefinitely.  A reaction wheel cannot:
// it works by trading angular momentum with the body, and it has a hard speed
// ceiling.  Once the wheel saturates, the cube has NO control authority left
// and falls regardless of how good the angle feedback is.
//
// So wheel speed is not a nuisance state -- it is a resource that has to be
// managed.  The k_omega term deliberately leans the cube a fraction of a
// degree away from vertical, so that gravity does the work of accelerating
// the wheel back toward zero.  The cube "spends" a little tilt to buy back
// momentum headroom.  It is small compared to the other two terms, and it is
// the reason this is a three-state law rather than a PD loop on angle.
//
// WHERE THE GAINS COME FROM
//
// They are NOT tunable by feel in the way a PID loop is -- the three terms
// trade against each other, and the stable region is not obvious from the
// outside.  Solve the LQR Riccati equation for the linearised plant instead.
// That needs four measured physical quantities:
//
//     I_b   body inertia about the pivot edge      kg m^2
//     I_w   wheel inertia about its own axis       kg m^2
//     m, l  total mass and pivot-to-CoM distance   kg, m
//
// docs/3d_scaling/README.md section 3 has the measurement procedures (a swing
// test for I_b) and section 4 has a runnable scipy solve that turns them into
// K, plus sanity checks on the result and a staged tuning procedure.
#pragma once

#include <cmath>
#include <string>

#include "core/Types.hpp"

namespace cube {

// Why the controller is not commanding a balancing torque.
enum class SafetyState {
  kOk,             // balancing normally
  kNotReady,       // estimator has not converged yet -- not a fault
  kTiltExceeded,   // |theta| past the e-stop limit -- latched
  kWheelSaturated, // wheel past its speed limit -- latched
  kSensorFault,    // IMU or controller stopped answering -- latched
  kMotorFault,     // controller reported a fault code -- latched
  kUnconfigured,   // a gain or limit is still unset -- refuses to run
};

const char* ToString(SafetyState state);

struct ControlOutput {
  // Torque to command, Nm.  Always 0.0 when `armed` is false.
  double torque_nm = 0.0;

  // False once a safety limit has tripped.  Latches: the only way back is
  // reset(), which the balancer requires a human to trigger.
  bool armed = true;

  SafetyState safety = SafetyState::kOk;

  // Individual terms, for logging and gain tuning.  Pre-clamp, so you can
  // see which one dominates and which one saturated.
  double term_theta = 0.0;
  double term_theta_dot = 0.0;
  double term_omega = 0.0;

  // True if the clamp actually limited the command this cycle.  Persistent
  // saturation means the gains are too hot or the cube is past recovery.
  bool torque_clamped = false;

  // True if the wheel-speed taper faded this cycle's torque (see
  // Config::wheel_taper_start).  A duty cycle that stays high during normal
  // recoveries means the taper is starting too early, not that the
  // disturbance was large.
  bool wheel_taper_active = false;

  // True if update() was called with gain_scale < 1.0.  Exists because
  // forgetting the gain scale is turned down is a classic half-hour of
  // confusion during bench tuning.
  bool gain_scaled = false;
};

class BalancingController {
 public:
  // Every field is UNSET on purpose.  There are no default gains, not even
  // conservative ones: a named constant holding a plausible number is
  // exactly what makes an invented value look authoritative, and these
  // cannot be guessed without the measurements above.
  struct Config {
    // Angle gain, Nm per rad.
    //
    // WHAT IT DOES: the restoring term.  Torque proportional to how far the
    // cube has fallen.
    //
    // HOW TO CHOOSE IT: from the LQR solve --
    // `./moteus-venv/bin/python tools/lqr/solve_gains.py`.  Expect it to be
    // the largest of the three in MAGNITUDE, by a wide margin.
    //
    // ITS SIGN IS NOT SETTLED, and this comment used to claim it was.
    // Solved against the plant in docs/3d_scaling/README.md section 4 --
    // whose B[1] = -1/I_total is the wheel's reaction on the body, per
    // Newton's third law -- a stabilising k_theta comes out NEGATIVE, given
    // that update() below already negates the sum.  That is the opposite of
    // what this comment previously asserted.
    //
    // Neither claim is verified, and no amount of algebra can verify
    // either: what a positive feedforward_torque physically does to the
    // cube depends on the motor phase order and SOURCE0_SIGN, which the
    // model cannot see.  The solve fixes the magnitude; N1 fixes the sign
    // (wheel off, tilt by hand, watch which way the shaft pushes).
    //
    // When it comes out wrong, fix EST_INVERT_THETA or SOURCE0_SIGN --
    // NEVER the sign of the law in update().  See that function's comment.
    //
    // TODO(you): compute, then confirm the sign at N1 and record it in
    // docs/3d_scaling/measurements.md.
    double k_theta = NAN;

    // Rate gain, Nm per rad/s.
    //
    // WHAT IT DOES: damping.  Without it the angle term alone produces a
    // cube that oscillates through vertical instead of settling.
    //
    // HOW TO CHOOSE IT: from the same solve.  Typically an order of
    // magnitude below k_theta.  If the rig oscillates, this is too small
    // relative to k_theta; if it feels sluggish, too large.
    //
    // TODO(you): compute.
    double k_theta_dot = NAN;

    // Wheel speed gain, Nm per rad/s.
    //
    // WHAT IT DOES: momentum management -- see the header comment above.
    //
    // HOW TO CHOOSE IT: from the same solve.  It should come out SMALL,
    // roughly a thousandth of k_theta.  Too large and the cube visibly
    // leans away from vertical to bleed wheel speed; too small and the
    // wheel drifts to saturation over tens of seconds and the cube drops
    // with no warning.
    //
    // TODO(you): compute.  Tune this one LAST, after the rig balances.
    double k_omega = NAN;

    // E-stop tilt limit, radians.
    //
    // WHAT IT DOES: past this angle the controller latches off and commands
    // zero torque.
    //
    // HOW TO CHOOSE IT: the angle beyond which a reaction wheel cannot
    // recover the cube -- driving past it only adds energy to the crash.
    // The recoverable envelope is a function of your torque ceiling and
    // inertia, so it follows from the same measurements as the gains.
    // 15 degrees (0.2618 rad) is a conservative starting point.
    //
    // TODO(you): set this.
    double max_tilt_rad = NAN;

    // Torque clamp, Nm.
    //
    // WHAT IT DOES: bounds the control law's output, independently of the
    // driver's own ceiling.
    //
    // HOW TO CHOOSE IT: at or below what the motor can actually deliver.
    // Real ceiling = K_t * servo.max_current_A, where K_t = 60/(2*pi*kv).
    // For this motor (kv ~ 373.6) that is ~0.0256 Nm/A, so 15 A gives about
    // 0.38 Nm.  Setting this above the real ceiling means the law believes
    // it has authority it does not have.
    //
    // TODO(you): set this from your measured current limit.
    double max_torque_nm = NAN;

    // Wheel speed at which the cube is declared unrecoverable, rad/s.  The
    // hard backstop -- see wheel_taper_start below for what keeps a
    // legitimate recovery from reaching it in normal operation.
    //
    // WHAT IT DOES: latches off before the wheel reaches the motor's real
    // ceiling, leaving margin to decelerate.
    //
    // HOW TO CHOOSE IT: well under the no-load speed.  For this motor at
    // ~20 V bus and kv 373.6, no-load is roughly 7500 RPM ~ 780 rad/s, so a
    // limit around 300 rad/s (~2900 RPM) leaves generous headroom.
    //
    // TODO(you): set this.
    double max_wheel_omega = NAN;

    // Wheel speed at which spin-up torque starts to fade, rad/s.  Must be
    // less than max_wheel_omega.
    //
    // WHAT IT DOES: between this speed and max_wheel_omega, torque that
    // would accelerate the wheel FURTHER in its current direction is
    // smoothly scaled toward zero; torque that brakes it is never touched.
    // Without this, a recovery that legitimately needs the wheel near its
    // cap trips kWheelSaturated instead of being allowed to finish -- a
    // panel-stage test of this control law saw the wheel reach 39.7 of a
    // 40 rad/s cap on a normal envelope-edge recovery, which is exactly the
    // case a hard trip alone punishes.
    //
    // HOW TO CHOOSE IT: below max_wheel_omega with enough margin that the
    // hard trip stays a true backstop rather than the taper's usual
    // operating point -- e.g. max_wheel_omega minus 10-15% of itself.
    //
    // TODO(you): set this once max_wheel_omega is set.
    double wheel_taper_start = NAN;

    // Deadband on theta, radians.
    //
    // WHAT IT DOES: inside it no torque is commanded, so the wheel stops
    // chattering around the balance point on sensor noise.
    //
    // HOW TO CHOOSE IT: just above the peak-to-peak theta noise you observe
    // in imu_test with the rig held still.  0.002 rad is about 0.11 deg.
    //
    // WARNING -- a nonzero deadband does not just sit quietly: with k_theta
    // zeroed out inside it, the remaining theta_dot/omega feedback has no
    // equilibrium at theta = 0, so the state does not settle there -- it
    // drifts across the dead zone, re-enters the k_theta region, gets pulled
    // back, and overshoots into the deadband on the other side.  That is a
    // limit cycle at roughly +/-theta_deadband, not a quiet zone, and it
    // gets WORSE as theta_deadband grows.  Prefer 0.0 (no deadband) unless
    // chatter is actually demonstrated on the rig; if it is, fix the noise
    // in the estimator (tau, rate_cutoff_hz) rather than papering over it
    // here.
    //
    // TODO(you): measure the noise floor.  Leave at 0.0 unless chatter is
    // demonstrated.
    double theta_deadband = NAN;

    // Consecutive invalid-estimate cycles allowed, once the estimator has
    // produced at least one valid sample, before latching kSensorFault.
    //
    // WHAT IT DOES: closes a gap in the "state.valid == false" path, which
    // is otherwise reported as kNotReady (not a trip, torque held at zero,
    // stays armed) forever.  That is correct while the estimator is still
    // warming up -- but if the IMU dies mid-flight, the same kNotReady
    // report reads to an operator as "still warming up", not "the sensor is
    // gone", and the rig coasts at zero torque and falls while looking
    // idle rather than faulted.  Counting only starts after the estimator
    // has been valid at least once, so ordinary startup is unaffected.
    //
    // HOW TO CHOOSE IT: a couple of loop periods' worth of grace for a
    // single dropped read, not so many that a genuine sensor death goes
    // unreported for long.  At 200 Hz, 10 cycles is 50 ms.
    //
    // TODO(you): set this.
    int max_invalid_cycles = -1;
  };

  BalancingController();
  explicit BalancingController(const Config& config);

  // True once every sentinel above has been replaced.  update() refuses to
  // arm while this is false.
  bool isConfigured() const;

  // Names the first unset field, for an error message.  nullptr when
  // isConfigured() is true.
  const char* firstUnsetField() const;

  // Compute the torque for one cycle.  Runs the safety checks first: if any
  // trips, the output is zero torque and disarmed, and it stays that way
  // until reset().
  //
  // gain_scale multiplies the whole computed torque, uniformly softening
  // the closed loop without moving its poles (scaling individual gains
  // instead would).  It is a call-site argument rather than a Config field
  // on purpose -- that keeps "this cycle is running softened" visible at
  // every call site instead of hidden in a struct.  Pass e.g. 0.1 for a
  // first closed-loop attempt, 1.0 once the gains are trusted.  Validate it
  // to [0, 1] at its source (wherever it is set); this function trusts what
  // it is given, same as it trusts the rest of Config.
  ControlOutput update(const BodyState& state, const MotorState& motor,
                       double gain_scale = 1.0);

  // Clear a latched safety trip and re-arm.  Only call this when a human has
  // confirmed the rig is back in a safe state.
  void reset();

  bool armed() const { return armed_; }
  SafetyState safety() const { return safety_; }
  const Config& config() const { return config_; }
  Config& mutableConfig() { return config_; }

 private:
  void trip(SafetyState reason);

  Config config_;
  bool armed_ = true;
  SafetyState safety_ = SafetyState::kOk;

  // Mid-flight sensor loss tracking -- see Config::max_invalid_cycles.
  bool ever_valid_ = false;
  int invalid_count_ = 0;
};

}  // namespace cube
