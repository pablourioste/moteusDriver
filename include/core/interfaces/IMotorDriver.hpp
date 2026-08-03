// Abstract reaction wheel motor. Implemented by MoteusDriverWrapper around
// the existing, verified moteus actuation code.
#pragma once

#include <string>

#include "core/Types.hpp"

namespace cube {

class IMotorDriver {
 public:
  virtual ~IMotorDriver() = default;

  // Open the transport, push the controller configuration, and clear any
  // latched fault.  Returns false and fills `error` on failure.
  virtual bool initialize(std::string* error) = 0;

  // Command a torque at the wheel, Nm, and return the resulting controller
  // state.  Implementations must clamp to their configured ceiling and must
  // arm a watchdog so the controller stops itself if this process dies.
  virtual MotorState sendTorque(double torque_nm) = 0;

  // Query state without commanding anything.
  virtual MotorState query() = 0;

  // Stop the controller.  Must be safe to call repeatedly, from a signal
  // handler path, and when initialize() failed.
  virtual void stop() = 0;

  // Largest torque this driver will pass through, Nm.
  virtual double maxTorqueNm() const = 0;

  virtual std::string name() const = 0;
};

}  // namespace cube
