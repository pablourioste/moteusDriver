#include "core/StateEstimator.hpp"

#include <cmath>

namespace cube {
namespace {

// Standard gravity, m/s^2.  A defined SI constant, not a tuning value.
constexpr double kGravity = 9.80665;

}  // namespace

StateEstimator::StateEstimator() = default;

StateEstimator::StateEstimator(const Config& config) : config_(config) {}

// --- Configuration validation ---------------------------------------------
//
// Implemented, not scaffolded: this is the guard that keeps an unconfigured
// estimator from reaching the motor, so it has to work before anything else
// does.

const char* StateEstimator::firstUnsetField() const {
  if (std::isnan(config_.tau)) { return "tau"; }
  if (config_.gyro_axis < 0) { return "gyro_axis"; }
  if (config_.accel_axis_a < 0) { return "accel_axis_a"; }
  if (config_.accel_axis_b < 0) { return "accel_axis_b"; }
  if (std::isnan(config_.theta_offset)) { return "theta_offset"; }
  if (std::isnan(config_.rate_cutoff_hz)) { return "rate_cutoff_hz"; }
  if (config_.warmup_samples < 0) { return "warmup_samples"; }
  if (std::isnan(config_.accel_gate)) { return "accel_gate"; }
  return nullptr;
}

bool StateEstimator::isConfigured() const {
  return firstUnsetField() == nullptr;
}

// --- Axis dispatch ---------------------------------------------------------
//
// Implemented: trivial selection, no judgement involved.

double StateEstimator::accelAxis(const ImuData& imu, int axis) {
  switch (axis) {
    case 0: return imu.accel_x;
    case 1: return imu.accel_y;
    default: return imu.accel_z;
  }
}

double StateEstimator::gyroAxis(const ImuData& imu, int axis) {
  switch (axis) {
    case 0: return imu.gyro_x;
    case 1: return imu.gyro_y;
    default: return imu.gyro_z;
  }
}

void StateEstimator::reset() {
  state_ = BodyState{};
  sample_count_ = 0;
  seeded_ = false;
}

// --- The parts you write ---------------------------------------------------

double StateEstimator::accelAngle(const ImuData& imu) const {
  // ==== TODO(you): implement ==============================================
  //
  // Tilt from the gravity vector alone.  Two lines of real work:
  //
  //   a = accelAxis(imu, config_.accel_axis_a);
  //   b = accelAxis(imu, config_.accel_axis_b);
  //   angle = atan2(a, b) - config_.theta_offset;
  //   if (config_.invert_theta) angle = -angle;
  //
  // WHY atan2 AND NOT asin:  asin needs the vector normalised first and
  // loses conditioning as the argument approaches +/-1.  atan2 stays well
  // conditioned through the entire circle and needs no normalisation,
  // because the magnitude cancels in the ratio.
  //
  // WHY THE OFFSET IS SUBTRACTED BEFORE THE INVERSION:  theta_offset is
  // measured in the sensor's own sign convention (it is a raw reading at the
  // balance point), so it has to be removed before the convention is
  // flipped.  Doing it the other way round puts the offset in with the wrong
  // sign and biases the cube toward one side.
  // ========================================================================
  (void)imu;
  return NAN;
}

BodyState StateEstimator::update(const ImuData& imu, const MotorState& motor,
                                 Seconds dt) {
  // ==== TODO(you): implement ==============================================
  //
  // Suggested order.  Each step has a reason, and skipping one produces a
  // specific failure -- noted so you can recognise it on the rig.
  //
  // 1. WHEEL SPEED.  Copy motor.velocity_rad_s() into state_.wheel_omega
  //    when motor.valid.  No filtering: the board already PLL-filters the
  //    encoder velocity (motor_position.sources.0.pll_filter_hz = 400), so
  //    filtering again only adds phase lag to the term that is supposed to
  //    stop the wheel saturating.
  //
  // 2. REJECT BAD INPUT before integrating anything.
  //      !imu.valid           -> hold the previous estimate, set
  //                              state_.valid = false, return.  Do NOT
  //                              integrate: a dropped I2C read is not
  //                              evidence that the cube stopped moving.
  //      dt <= 0 || dt > 0.1  -> a missed or duplicated cycle.  Integrating
  //                              a 5-second dt injects a step into theta
  //                              that the filter then spends seconds
  //                              unwinding.
  //
  // 3. ACCELEROMETER ANGLE.  theta_accel = accelAngle(imu).  Absolute but
  //    noisy; this is the drift reference.
  //
  // 4. SEED ON THE FIRST SAMPLE.  Set state_.theta = theta_accel directly
  //    and mark seeded_.  Starting from zero instead makes the filter walk
  //    to the true angle over several tau -- while the controller is already
  //    acting on the wrong estimate.
  //
  // 5. GATE THE ACCELEROMETER.  During a hard correction the measured vector
  //    is mostly wheel reaction, not gravity:
  //
  //      magnitude = sqrt(ax^2 + ay^2 + az^2)
  //      trust     = fabs(magnitude - kGravity) < config_.accel_gate * kGravity
  //
  //    Blending an untrusted sample drags theta toward whatever direction
  //    the cube happened to be accelerating.
  //
  // 6. COMPLEMENTARY BLEND.
  //
  //      alpha     = tau / (tau + dt)
  //      predicted = state_.theta + gyro_rate * dt
  //      theta     = trust ? alpha * predicted + (1 - alpha) * theta_accel
  //                        : predicted
  //
  //    DERIVE alpha FROM THE MEASURED dt, never hardcode it.  That is what
  //    makes the filter behave identically at 200 Hz and at 50 Hz, and what
  //    keeps a late cycle from silently changing the crossover frequency.
  //
  //    Remember to apply config_.invert_theta to gyro_rate as well as to the
  //    accelerometer angle, or the two halves of the filter will disagree
  //    about which way is positive and fight each other.
  //
  // 7. THETA_DOT.  The gyro measures it directly -- no fusion needed.
  //    Low-pass only to keep noise out of the derivative gain, and make the
  //    coefficient dt-aware for the same reason as step 6:
  //
  //      rc        = 1.0 / (2 * M_PI * config_.rate_cutoff_hz)
  //      beta      = dt / (dt + rc)
  //      theta_dot = (1 - beta) * state_.theta_dot + beta * gyro_rate
  //
  // 8. VALIDITY.  Count samples and report state_.valid only once
  //    sample_count_ >= config_.warmup_samples.
  //
  // SIGN CONVENTION (the one that can break the rig):  theta must be
  // positive when the cube tilts toward the direction a POSITIVE wheel
  // torque corrects.  Verify with imu_test before enabling the motor.  If it
  // is backwards the controller amplifies the fall instead of catching it.
  // ========================================================================
  (void)imu;
  (void)motor;
  (void)dt;

  // Until implemented, never claim a valid estimate.  BalancingController
  // treats an invalid estimate as "not ready" and commands zero torque, so
  // an unimplemented estimator cannot move the motor.
  state_.valid = false;
  return state_;
}

}  // namespace cube
