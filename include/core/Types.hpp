// Plain data types shared between the core control code and the hardware
// drivers.  Nothing here may include a driver header or a vendor SDK: the
// core/ tree has to compile without any hardware present so the control law
// can be unit tested on a desktop.
#pragma once

#include <cstdint>

namespace cube {

// Seconds.  Used for every timestamp and interval in the system so there is
// only one time unit to reason about.
using Seconds = double;

// A single IMU sample in SI units.  Raw counts are converted by the driver;
// nothing above the driver layer should ever see an LSB.
struct ImuData {
  // Linear acceleration, m/s^2, sensor frame.
  double accel_x = 0.0;
  double accel_y = 0.0;
  double accel_z = 0.0;

  // Angular rate, rad/s, sensor frame.
  double gyro_x = 0.0;
  double gyro_y = 0.0;
  double gyro_z = 0.0;

  // Die temperature, degrees C.  Useful mainly for spotting gyro bias drift.
  double temperature_c = 0.0;

  // Monotonic timestamp of the sample, seconds.
  Seconds timestamp = 0.0;

  // False if the read failed or the sample is stale.  The balancer treats a
  // bad sample as a safety event rather than silently reusing the last one.
  bool valid = false;
};

// Motor state as reported by the controller.  This mirrors the subset of
// moteus::Query::Result the balancer actually uses; keeping it separate means
// the control code does not depend on the moteus headers.
struct MotorState {
  // Reaction wheel angle, revolutions, and rate, rev/s, at the output.  With
  // a direct-drive wheel (ratio 1.0) these are also the rotor values.
  double position_rev = 0.0;
  double velocity_rev_s = 0.0;

  // Estimated torque at the output, Nm.
  double torque_nm = 0.0;

  // Bus voltage, V, and motor temperature, degrees C.
  double bus_voltage = 0.0;
  double motor_temperature_c = 0.0;

  // Controller mode and fault code, straight from the query.  Fault 0 means
  // no fault; any other value latches until a stop is issued.
  int mode = 0;
  int fault = 0;

  // False if the controller did not reply to the last command.
  bool valid = false;

  // Convenience: wheel rate in rad/s, which is what the control law wants.
  // Inline so core/ can use it without linking against the driver library.
  double velocity_rad_s() const {
    return velocity_rev_s * 6.283185307179586;
  }
};

// What the state estimator believes about the cube body right now.
struct BodyState {
  // Tilt from the balance point, radians.  Positive is a consistent
  // direction chosen by the estimator; the sign convention is documented on
  // StateEstimator::update().
  double theta = 0.0;

  // Tilt rate, rad/s.
  double theta_dot = 0.0;

  // Reaction wheel rate, rad/s.  Part of the state vector because the
  // controller has to bleed off wheel speed before it saturates.
  double wheel_omega = 0.0;

  // False until the estimator has enough samples to be trusted.
  bool valid = false;
};

}  // namespace cube
