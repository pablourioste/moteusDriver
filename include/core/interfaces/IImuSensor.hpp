// Abstract IMU. The balancer talks only to this, so a simulated IMU or a
// different part can be substituted without touching the control code.
#pragma once

#include <string>

#include "core/Types.hpp"

namespace cube {

class IImuSensor {
 public:
  virtual ~IImuSensor() = default;

  // Bring the sensor up.  Returns false and fills `error` on failure rather
  // than throwing, so the caller can decide whether a missing IMU is fatal.
  virtual bool initialize(std::string* error) = 0;

  // Read one sample.  Returns a sample with valid=false on a bus error; it
  // must never block for longer than a control period.
  virtual ImuData read() = 0;

  // Estimate and store the gyro zero-rate bias by averaging while the rig is
  // held still.  Blocks for roughly `duration`.  Returns false if the sensor
  // moved too much during the measurement for the result to be meaningful.
  virtual bool calibrateGyroBias(Seconds duration, std::string* error) = 0;

  // Human-readable part name, for logs.
  virtual std::string name() const = 0;
};

}  // namespace cube
