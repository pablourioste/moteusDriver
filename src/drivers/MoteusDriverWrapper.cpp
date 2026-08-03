#include "drivers/MoteusDriverWrapper.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>

#include <unistd.h>

#include "moteus.h"

namespace moteus = mjbots::moteus;

namespace cube {
namespace {

// Copy the fields the control code uses out of a moteus query reply.
MotorState FromQuery(const moteus::Query::Result& v) {
  MotorState state;
  state.position_rev = v.position;
  state.velocity_rev_s = v.velocity;
  state.torque_nm = v.torque;
  state.bus_voltage = v.voltage;
  state.motor_temperature_c = v.motor_temperature;
  state.mode = static_cast<int>(v.mode);
  state.fault = static_cast<int>(v.fault);
  state.valid = true;
  return state;
}

}  // namespace

MoteusConfig MoteusConfig::FromBuildDefaults() {
  MoteusConfig config;
#ifdef DEFAULT_CONTROLLER_ID
  config.controller_id = DEFAULT_CONTROLLER_ID;
#endif
#ifdef DEFAULT_CONFIG_PATH
  config.config_path = DEFAULT_CONFIG_PATH;
#endif
#ifdef APPLY_CONFIG_ON_STARTUP
  config.apply_config_on_startup = APPLY_CONFIG_ON_STARTUP;
#endif
#ifdef PERSIST_CONFIG_TO_FLASH
  config.persist_config_to_flash = PERSIST_CONFIG_TO_FLASH;
#endif
#ifdef MAX_TORQUE_NM
  config.max_torque_nm = MAX_TORQUE_NM;
#endif
#ifdef COMMAND_WATCHDOG_S
  config.watchdog_timeout_s = COMMAND_WATCHDOG_S;
#endif
#ifdef ROTOR_TO_OUTPUT_RATIO_DEF
  config.rotor_to_output_ratio = ROTOR_TO_OUTPUT_RATIO_DEF;
#endif
  return config;
}

// ---------------------------------------------------------------------------
// Config file parsing and push.
//
// Preserved verbatim from the original main.cpp -- these two functions are
// the part of the driver that is known to work against the real board, down
// to the retry timing.  Do not "clean up" the sleeps.
// ---------------------------------------------------------------------------

std::vector<std::string> ReadConfig(const std::string& path) {
  std::ifstream in(path);
  if (!in) {
    throw ConfigError("cannot open config file: " + path);
  }

  std::vector<std::string> result;
  std::string line;
  int line_number = 0;

  while (std::getline(in, line)) {
    line_number++;

    const auto comment = line.find('#');
    if (comment != std::string::npos) { line.erase(comment); }

    const auto begin = line.find_first_not_of(" \t\r");
    if (begin == std::string::npos) { continue; }
    const auto end = line.find_last_not_of(" \t\r");
    line = line.substr(begin, end - begin + 1);

    // A bare name with no value would silently turn into a malformed
    // `conf set`, so reject it here where we can name the line.
    if (line.find_first_of(" \t") == std::string::npos) {
      throw ConfigError(
          path + ":" + std::to_string(line_number) +
          ": expected '<name> <value>', got '" + line + "'");
    }

    result.push_back(line);
  }

  return result;
}

void MoteusDriverWrapper::applyConfig(const std::string& path) {
  const auto settings = ReadConfig(path);

  for (const auto& setting : settings) {
    const auto command = "conf set " + setting;
    auto response = controller_->DiagnosticCommand(command);

    // Some settings make the firmware revalidate its whole motor_position
    // configuration -- `motor_position.rotor_to_output_ratio` is one -- which
    // takes long enough that the reply to the *previous* command can still be
    // arriving.  The stale line then gets attributed to this command, showing
    // up as a bogus "ERR unknown command".  Give the board a moment and ask
    // once more before believing the rejection.
    if (response.find("ERR") != std::string::npos) {
      ::usleep(50000);
      response = controller_->DiagnosticCommand(command);
    }

    if (response.find("ERR") != std::string::npos) {
      throw ControllerRejected(
          "controller rejected '" + command + "': " + response);
    }

    // Settling between settings keeps the diagnostic channel in step; the
    // whole file is only ~60 lines, so the cost is negligible.
    ::usleep(2000);
  }

  if (config_.persist_config_to_flash) {
    controller_->DiagnosticCommand("conf write");
  }

  std::cout << "Applied " << settings.size() << " settings from " << path
            << (config_.persist_config_to_flash ? " (persisted to flash)" : "")
            << std::endl;
}

// ---------------------------------------------------------------------------

MoteusDriverWrapper::MoteusDriverWrapper(const MoteusConfig& config)
    : config_(config) {}

MoteusDriverWrapper::~MoteusDriverWrapper() {
  // Never leave the board energised because an object went out of scope.
  stop();
}

bool MoteusDriverWrapper::initialize(std::string* error) {
  const auto fail = [error](const std::string& message) {
    if (error) { *error = message; }
    return false;
  };

  try {
    moteus::Controller::Options options;
    options.id = config_.controller_id;
    controller_ = std::make_unique<moteus::Controller>(options);

    if (config_.apply_config_on_startup) {
      if (config_.config_path.empty()) {
        return fail("no config path set but apply_config_on_startup is true");
      }
      applyConfig(config_.config_path);
    }

    // Stop the board before commanding anything.  SetStop also clears a
    // latched fault, so a previous aborted run does not block this one.
    controller_->SetStop();

    initialized_ = true;
    return true;
  } catch (const ControllerRejected& e) {
    // The board answered, so it is reachable; only the setting was refused.
    return fail(
        std::string(e.what()) +
        "\n\nThe controller is reachable -- it replied -- but refused that "
        "setting.  Either the key does not exist on this firmware (check "
        "`moteus_tool -t 1 --dump-config`), or the value is out of range.  "
        "Re-run with -DAPPLY_CONFIG_ON_STARTUP=OFF to skip the config push.");
  } catch (const ConfigError& e) {
    // A bad config file is our own problem, not a missing controller; the
    // transport hint would only send the reader down the wrong path.
    return fail(std::string("config error: ") + e.what());
  } catch (const std::exception& e) {
    return fail(
        std::string(e.what()) +
        "\n\nNo moteus controller reachable.  Check that an fdcanusb is "
        "plugged in (/dev/ttyACM*) or a socketcan interface is up, then "
        "re-run.  See README.md for setup details.");
  }
}

MotorState MoteusDriverWrapper::sendTorque(double torque_nm) {
  MotorState state;
  if (!initialized_ || !controller_) { return state; }

  // A NaN would be sent to the board as an invalid feedforward term rather
  // than being ignored; refuse it here.
  if (!std::isfinite(torque_nm)) { torque_nm = 0.0; }

  // The driver's own ceiling, independent of the board's servo.max_current_A.
  const double limit = config_.max_torque_nm;
  if (torque_nm > limit) { torque_nm = limit; }
  if (torque_nm < -limit) { torque_nm = -limit; }

  moteus::PositionMode::Command command;

  // Pure torque mode.  position = NaN means "no position target", and zeroing
  // both gain scales removes the board's position and velocity feedback, so
  // feedforward_torque is the only term left.  This is deliberate: the
  // balancing law is the regulator, and a second regulator underneath it
  // would fight the outer loop.
  command.position = std::numeric_limits<double>::quiet_NaN();
  command.velocity = 0.0;
  command.kp_scale = 0.0;
  command.kd_scale = 0.0;
  command.feedforward_torque = torque_nm;
  command.maximum_torque = limit;

  // If this process dies or the link drops, the board stops itself rather
  // than holding the last torque forever.
  command.watchdog_timeout = config_.watchdog_timeout_s;

  const auto maybe_result = controller_->SetPosition(command);
  if (!maybe_result) { return state; }

  return FromQuery(maybe_result->values);
}

MotorState MoteusDriverWrapper::query() {
  MotorState state;
  if (!initialized_ || !controller_) { return state; }

  const auto maybe_result = controller_->SetQuery();
  if (!maybe_result) { return state; }

  return FromQuery(maybe_result->values);
}

void MoteusDriverWrapper::stop() {
  if (!controller_) { return; }
  try {
    controller_->SetStop();
  } catch (...) {
    // stop() is called from destructors and from the signal-handler path,
    // where an exception has nowhere to go.  A failed stop is already the
    // worst case; there is nothing better to do than not make it worse.
  }
}

}  // namespace cube
