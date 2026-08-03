// IMotorDriver implemented on the moteus controller.
//
// This is the existing, working actuation code from main.cpp moved behind an
// interface: ReadConfig, ApplyConfig, the ERR-retry, ConfigError and
// ControllerRejected are all preserved as they were.  The only additions are
// the interface plumbing and a torque-mode command shape.
#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "core/Types.hpp"
#include "core/interfaces/IMotorDriver.hpp"
#include "drivers/MoteusConfig.hpp"

// Forward declared so this header does not drag moteus.h -- and its transport
// and protocol headers -- into every translation unit that merely holds a
// driver pointer.
namespace mjbots {
namespace moteus {
class Controller;
}
}

namespace cube {

// Raised for anything wrong with the config file itself, so callers can tell
// it apart from a transport failure and report it accurately.
class ConfigError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// Raised when the controller answers a `conf set` with an error.  Distinct
// from a transport failure: the board is plainly reachable if it replied, so
// callers must not print the "no controller reachable" hint for this.
class ControllerRejected : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// Reads current_config.cfg: one `<name> <value>` pair per line, with blank
// lines and #-comments ignored.  Returns the lines verbatim (trimmed), since
// the file's format is already the argument format `conf set` expects.
//
// Free function rather than a method: it touches no hardware, so the config
// parser stays testable without a controller.
std::vector<std::string> ReadConfig(const std::string& path);

class MoteusDriverWrapper : public IMotorDriver {
 public:
  explicit MoteusDriverWrapper(const MoteusConfig& config);
  ~MoteusDriverWrapper() override;

  MoteusDriverWrapper(const MoteusDriverWrapper&) = delete;
  MoteusDriverWrapper& operator=(const MoteusDriverWrapper&) = delete;

  // Constructs the controller, pushes the config, and clears latched faults.
  // Catches ConfigError/ControllerRejected/transport failures and turns them
  // into a false return with a diagnostic message, so applications get the
  // same three-way error reporting main.cpp had.
  bool initialize(std::string* error) override;

  // Pure torque command.  position = NaN with kp_scale = kd_scale = 0 turns
  // off the board's position and velocity feedback entirely, leaving
  // feedforward_torque as the only term -- which is what a balancing law
  // needs, since the outer loop is doing the regulating.
  MotorState sendTorque(double torque_nm) override;

  MotorState query() override;

  // Safe to call repeatedly, before initialize(), and from a signal handler
  // path.  Never throws.
  void stop() override;

  double maxTorqueNm() const override { return config_.max_torque_nm; }
  std::string name() const override { return "moteus"; }

  // Push the configuration to the controller.  Each `conf set` replies with
  // "OK" or an "ERR ..." line; a rejected pin mode should stop us rather than
  // leave the board half-configured.  Throws ConfigError/ControllerRejected.
  //
  // Public because direct_motor_test may want to re-apply mid-session.
  void applyConfig(const std::string& path);

  // Raw controller access for code that needs the full moteus API -- the
  // health check drives velocity-mode commands the interface does not
  // expose.  Null before initialize() succeeds.
  mjbots::moteus::Controller* controller() { return controller_.get(); }

  const MoteusConfig& config() const { return config_; }

 private:
  MoteusConfig config_;
  std::unique_ptr<mjbots::moteus::Controller> controller_;
  bool initialized_ = false;
};

}  // namespace cube
