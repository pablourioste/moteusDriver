// Fuses accelerometer and gyroscope into a tilt estimate.
//
// STATUS: implemented.  update() and accelAngle() are in
// src/core/StateEstimator.cpp, each step numbered against the derivation
// below.  What is still outstanding is MEASUREMENT, not code: every
// Config field below ships as a NaN/-1 sentinel and has to be measured on
// the rig before anything will run.  See the per-field TODO(you) notes.
//
// WHY A COMPLEMENTARY FILTER
//
// For a single tilt axis a complementary filter and a Kalman filter perform
// almost identically, and this one has a single tuning parameter with a
// physical meaning -- which matters more on a rig that has not been
// characterised yet.
//
// The division of labour is the standard one, and it follows from what each
// sensor actually measures:
//
//   The accelerometer measures the sum of gravity and the cube's own
//   acceleration.  At rest that is purely gravity, so it gives an ABSOLUTE
//   tilt reference with no drift -- but it is useless during fast motion,
//   which is exactly when the controller is working hardest.
//
//   The gyroscope measures angular rate directly.  It is clean and fast and
//   unaffected by linear acceleration, but it must be INTEGRATED to get an
//   angle, so its zero-rate bias accumulates without bound.
//
// So: trust the gyro over short intervals, and let the accelerometer pull the
// estimate back toward the gravity vector slowly.  tau is where you set the
// boundary between "short" and "slow".
#pragma once

#include <cmath>

#include "core/Types.hpp"

namespace cube {

class StateEstimator {
 public:
  // Every field here is UNSET on purpose.  NAN and -1 are sentinels: the
  // estimator refuses to run until you replace them, so a rig can never
  // balance on numbers nobody chose.  Validate with isConfigured().
  struct Config {
    // Complementary filter time constant, seconds.
    //
    // WHAT IT DOES: sets the crossover between the two sensors.  Below tau
    // the estimate follows the gyro; above it, the accelerometer.
    //
    // HOW TO CHOOSE IT: start near 0.5 s.  Too low and wheel-induced
    // acceleration corrupts tilt; too high and gyro bias drifts in before
    // the accelerometer corrects it.
    //
    // TODO(you): set this, then verify with imu_test that tilt tracks a hand
    // disturbance without lag and without jitter.
    double tau = NAN;

    // Which gyro axis measures rotation about the balancing edge, and which
    // two accelerometer axes span the plane the cube tilts in.
    // 0 = x, 1 = y, 2 = z.
    //
    // WHAT THEY DO: map the sensor's physical frame onto the one-dimensional
    // tilt problem.  gyro_axis is the rate; accel_axis_a and _b are the
    // atan2 numerator and denominator.
    //
    // HOW TO CHOOSE THEM: depends entirely on how the IMU is bolted to the
    // cube.  For an IMU flat on a cube face with the cube tilting about the
    // sensor X axis, gravity swings in the Y-Z plane: gyro_axis = 0,
    // accel_axis_a = 1, accel_axis_b = 2.
    //
    // TODO(you): set these, then confirm with imu_test that tilting the rig
    // one way moves theta in one consistent direction.
    int gyro_axis = -1;
    int accel_axis_a = -1;
    int accel_axis_b = -1;

    // Invert the tilt sign.
    //
    // WHAT IT DOES: flips theta and theta_dot together, so the sign
    // convention below can be satisfied without rewiring or re-mounting.
    //
    // HOW TO CHOOSE IT: run imu_test and tilt the cube toward the direction
    // a POSITIVE wheel torque would correct.  If theta goes negative, set
    // this true.
    //
    // TODO(you): verify before enabling the motor.  Getting this wrong makes
    // the controller drive the cube over instead of catching it -- this is
    // the single most dangerous unset value in the project.
    bool invert_theta = false;

    // Accelerometer angle at the true balance point, radians.
    //
    // WHAT IT DOES: subtracted from the computed angle so theta reads zero
    // when the cube is actually balanced.
    //
    // HOW TO CHOOSE IT: it is never exactly zero -- the IMU is never mounted
    // perfectly square and the centre of mass is never exactly over the
    // pivot edge.  Balance the cube by hand, read the angle imu_test prints,
    // and put that number here.
    //
    // TODO(you): measure it.  A nonzero true offset left at 0.0 makes the
    // cube converge to a permanent lean instead of upright.
    double theta_offset = NAN;

    // Cutoff frequency of the low-pass on theta_dot, Hz.
    //
    // WHAT IT DOES: keeps gyro noise out of the derivative term.  theta_dot
    // is multiplied by k_theta_dot, so noise here becomes torque chatter.
    //
    // HOW TO CHOOSE IT: high enough to pass real cube motion (a few Hz),
    // low enough to reject sensor noise.  10-20 Hz is a sane starting band,
    // but check it is not eating the phase budget: a first-order low-pass at
    // 10 Hz adds roughly 9 degrees of phase lag at a ~1.6 Hz closed-loop
    // bandwidth, stacking on top of whatever the loop's own sample delay
    // already costs.  At 50 Hz the same figure is about 1.8 degrees --
    // effectively free.  Prefer >= 30 Hz unless the noise floor forces it
    // lower.
    //
    // NOTE: this is a FREQUENCY, not a filter coefficient, precisely so it
    // keeps its meaning when the loop rate changes.  Derive the per-sample
    // coefficient from the measured dt -- see step 7 in the .cpp.
    //
    // TODO(you): set this.
    double rate_cutoff_hz = NAN;

    // Expected loop interval, seconds -- i.e. 1 / control_rate_hz.
    //
    // WHAT IT DOES: bounds how far the MEASURED dt passed to update() is
    // allowed to drift from the intended one before a cycle is treated as
    // missed or duplicated rather than integrated.  See update() for the
    // exact bound.
    //
    // HOW TO CHOOSE IT: the reciprocal of whatever rate the caller's loop
    // actually runs at (cube_balancer's --rate, or the Teensy's fixed loop
    // period).  Getting this wrong doesn't corrupt the filter -- it just
    // makes the reject-bad-dt gate too loose or too tight.
    //
    // TODO(you): set this to 1.0 / <your control rate>.
    double nominal_dt = NAN;

    // Samples to accumulate before the estimate is reported valid.
    //
    // WHAT IT DOES: keeps the balancer from acting on a half-converged
    // estimate at startup.  The filter is seeded from the accelerometer on
    // the first sample, so this is a short settling allowance, not a full
    // convergence wait.
    //
    // HOW TO CHOOSE IT: a few tens of samples.  At 200 Hz, 20 samples is
    // 100 ms.
    //
    // TODO(you): set this.
    int warmup_samples = -1;

    // Accelerometer trust gate, as a fraction of g.
    //
    // WHAT IT DOES: a sample whose acceleration magnitude is further than
    // this from 1 g is dominated by cube motion rather than gravity, so it
    // carries no tilt information and the accelerometer correction is
    // skipped for that cycle.
    //
    // HOW TO CHOOSE IT: measure |accel| with imu_test while the wheel makes
    // a hard correction, and set the gate just above the quiet-state spread
    // but below the disturbance peaks.  0.25 (i.e. +/-25% of g) is a
    // plausible starting point.
    //
    // TODO(you): set this.
    double accel_gate = NAN;
  };

  StateEstimator();
  explicit StateEstimator(const Config& config);

  // True once every sentinel above has been replaced with a real value.
  // Applications must check this before running: an estimator built from the
  // defaults produces NaN, and a NaN theta silently disables every
  // comparison in the safety envelope.
  bool isConfigured() const;

  // Names the first unset field, for an error message.  Returns nullptr when
  // isConfigured() is true.
  const char* firstUnsetField() const;

  // Feed one IMU sample and one motor state.  `dt` is the interval since the
  // previous call, in seconds; pass the MEASURED value, not the nominal
  // period, because a late cycle otherwise corrupts the integration.
  //
  // SIGN CONVENTION: theta is positive when the cube tilts toward the
  // direction that a positive wheel torque corrects.  That relationship is
  // established by the axis configuration above and must be verified on the
  // rig with imu_test.
  BodyState update(const ImuData& imu, const MotorState& motor, Seconds dt);

  // Drop all history.  Use after an e-stop, before re-arming.
  void reset();

  const BodyState& state() const { return state_; }

  // Tilt implied by the accelerometer alone, radians.  Unfiltered; used by
  // imu_test to report static inclination and to measure theta_offset.
  double accelAngle(const ImuData& imu) const;

  const Config& config() const { return config_; }

 private:
  // Pull one axis out of a sample by index, so the axis mapping above can be
  // configuration rather than compiled-in field access.
  static double accelAxis(const ImuData& imu, int axis);
  static double gyroAxis(const ImuData& imu, int axis);

  Config config_;
  BodyState state_;
  int sample_count_ = 0;
  bool seeded_ = false;
};

}  // namespace cube
