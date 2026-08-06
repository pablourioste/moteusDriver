#include "core/StateEstimator.hpp"

#include <cmath>

namespace cube {
namespace {

// Standard gravity, m/s^2.  A defined SI constant, not a tuning value.
constexpr double kGravity = 9.80665;

// Pi, used to turn rate_cutoff_hz into a filter time constant.  Defined
// locally rather than relying on M_PI: it is not part of the C++ standard,
// and this file has to compile identically on the host and on the Teensy
// cross-compiler.
constexpr double kPi = 3.14159265358979323846;

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
  if (std::isnan(config_.nominal_dt)) { return "nominal_dt"; }
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
  // atan2, not asin: asin needs the vector normalised first and loses
  // conditioning as the argument approaches +/-1.  atan2 stays well
  // conditioned through the entire circle and needs no normalisation,
  // because the magnitude cancels in the ratio.
  const double a = accelAxis(imu, config_.accel_axis_a);
  const double b = accelAxis(imu, config_.accel_axis_b);
  double angle = std::atan2(a, b) - config_.theta_offset;

  // Offset subtracted BEFORE the inversion: theta_offset is measured in the
  // sensor's own raw sign convention (a reading taken at the balance point),
  // so it has to be removed before that convention is flipped.  Doing it the
  // other way round applies the offset with the wrong sign and biases the
  // cube toward one side.
  if (config_.invert_theta) {
    angle = -angle;
  }
  return angle;
}

BodyState StateEstimator::update(const ImuData& imu, const MotorState& motor,
                                 Seconds dt) {
  // 1. Wheel speed.  Independent of the IMU chain, so it is updated before
  // the reject-bad-input gate below -- a stale accelerometer read is not a
  // reason to also hold a perfectly good CAN reply.  No filtering: the board
  // already PLL-filters the encoder velocity
  // (motor_position.sources.0.pll_filter_hz = 400), so filtering again only
  // adds phase lag to the term that is supposed to stop the wheel
  // saturating.
  if (motor.valid) {
    state_.wheel_omega = motor.velocity_rad_s();
  }

  // 2. Reject bad input before integrating anything.  A dropped I2C/SPI read
  // is not evidence the cube stopped moving, and a missed or duplicated
  // cycle (dt far from nominal) would inject a step into theta that the
  // filter then spends seconds unwinding.  Hold the previous estimate and
  // report not-valid rather than integrate either one.
  //
  // The dt bound is +/-2x nominal_dt, not a fixed constant: a fixed 0.1 s
  // ceiling is roughly 40x nominal at a 400 Hz loop, which would wave
  // through a run of dozens of missed cycles as if it were one slightly-late
  // one.  Tying it to the configured rate keeps the gate meaningful as the
  // loop rate changes.
  if (!imu.valid || dt <= 0.0 || dt < 0.5 * config_.nominal_dt ||
      dt > 2.0 * config_.nominal_dt) {
    state_.valid = false;
    return state_;
  }

  // 3. Accelerometer angle.  Absolute but noisy; this is the drift
  // reference the gyro integration is corrected against.
  const double theta_accel = accelAngle(imu);

  // 4. Seed on the first sample.  Starting from theta = 0 instead would make
  // the filter walk to the true angle over several tau, while the controller
  // is already acting on the wrong estimate in the meantime.
  if (!seeded_) {
    state_.theta = theta_accel;
    seeded_ = true;
  }

  // 5. Gate the accelerometer.  During a hard correction the measured vector
  // is mostly wheel reaction, not gravity; blending an untrusted sample
  // would drag theta toward whatever direction the cube happened to be
  // accelerating.
  const double magnitude = std::sqrt(imu.accel_x * imu.accel_x +
                                     imu.accel_y * imu.accel_y +
                                     imu.accel_z * imu.accel_z);
  const bool trust_accel =
      std::fabs(magnitude - kGravity) < config_.accel_gate * kGravity;

  // 6. Complementary blend.  invert_theta is applied to the gyro rate here,
  // the same way it is applied inside accelAngle() -- otherwise the two
  // halves of the filter disagree about which way is positive and fight
  // each other.  alpha is derived from the MEASURED dt every cycle, never
  // hardcoded: that is what keeps a late cycle from silently shifting the
  // crossover frequency.
  double gyro_rate = gyroAxis(imu, config_.gyro_axis);
  if (config_.invert_theta) {
    gyro_rate = -gyro_rate;
  }
  const double alpha = config_.tau / (config_.tau + dt);
  const double predicted = state_.theta + gyro_rate * dt;
  state_.theta = trust_accel
                     ? (alpha * predicted + (1.0 - alpha) * theta_accel)
                     : predicted;

  // 7. theta_dot.  The gyro measures it directly -- no fusion needed, just a
  // low-pass to keep sensor noise out of the derivative gain (theta_dot is
  // multiplied by k_theta_dot in the balancing law, so noise here becomes
  // torque chatter).  The coefficient is dt-aware for the same reason alpha
  // is above.
  const double rc = 1.0 / (2.0 * kPi * config_.rate_cutoff_hz);
  const double beta = dt / (dt + rc);
  state_.theta_dot = (1.0 - beta) * state_.theta_dot + beta * gyro_rate;

  // 8. A NaN slipping through here (a bad SPI/I2C word, tau misconfigured to
  // 0, ...) must not reach the controller silently: every comparison
  // against NaN is false, so BalancingController's safety checks would pass
  // right over it instead of tripping.
  if (!std::isfinite(state_.theta) || !std::isfinite(state_.theta_dot)) {
    state_.valid = false;
    return state_;
  }

  // 9. Validity.  Count samples and report state_.valid only once
  // sample_count_ >= warmup_samples -- the filter is seeded from the
  // accelerometer above, so this is a short settling allowance, not a full
  // convergence wait.
  ++sample_count_;
  state_.valid = (sample_count_ >= config_.warmup_samples);

  return state_;
}

}  // namespace cube
