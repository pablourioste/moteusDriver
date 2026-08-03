#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>
#include "moteus.h"

namespace moteus = mjbots::moteus;

namespace {

// Flipped by the signal handler; the control loop polls it and exits.  The
// handler must stay async-signal-safe, so it does nothing but set this and
// command a stop -- no printing, no allocation.
std::atomic<bool> g_shutdown{false};

// Non-owning pointer to the controller the handler stops.  Raw rather than a
// smart pointer because the handler cannot safely touch a refcount.
moteus::Controller* g_controller = nullptr;

void HandleSignal(int) {
  g_shutdown.store(true);
  // Without this, Ctrl-C leaves the board holding its last command and
  // drawing current indefinitely.  This is the whole point of the handler.
  if (g_controller) { g_controller->SetStop(); }
}

}  // namespace

// Raised for anything wrong with the config file itself, so main() can tell
// it apart from a transport failure and report it accurately.
class ConfigError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// Raised when the controller answers a `conf set` with an error.  Distinct
// from a transport failure: the board is plainly reachable if it replied, so
// main() must not print the "no controller reachable" hint for this.
class ControllerRejected : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

namespace {

// Reads current_config.cfg: one `<name> <value>` pair per line, with blank
// lines and #-comments ignored.  Returns the lines verbatim (trimmed), since
// the file's format is already the argument format `conf set` expects.
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

// Push the configuration to the controller.  Each `conf set` replies with
// "OK" or an "ERR ..." line; a rejected pin mode should stop us rather than
// leave the board half-configured.
void ApplyConfig(moteus::Controller* c, const std::string& path) {
  const auto settings = ReadConfig(path);

  for (const auto& setting : settings) {
    const auto command = "conf set " + setting;
    auto response = c->DiagnosticCommand(command);

    // Some settings make the firmware revalidate its whole motor_position
    // configuration -- `motor_position.rotor_to_output_ratio` is one -- which
    // takes long enough that the reply to the *previous* command can still be
    // arriving.  The stale line then gets attributed to this command, showing
    // up as a bogus "ERR unknown command".  Give the board a moment and ask
    // once more before believing the rejection.
    if (response.find("ERR") != std::string::npos) {
      ::usleep(50000);
      response = c->DiagnosticCommand(command);
    }

    if (response.find("ERR") != std::string::npos) {
      throw ControllerRejected(
          "controller rejected '" + command + "': " + response);
    }

    // Settling between settings keeps the diagnostic channel in step; the
    // whole file is only ~60 lines, so the cost is negligible.
    ::usleep(2000);
  }

  if (PERSIST_CONFIG_TO_FLASH) {
    c->DiagnosticCommand("conf write");
  }

  std::cout << "Applied " << settings.size() << " settings from " << path
            << (PERSIST_CONFIG_TO_FLASH ? " (persisted to flash)" : "")
            << std::endl;
}

}  // namespace

int main(int argc, char** argv) try {
  // Let an explicit --config win over the generated default, so a different
  // pin map can be tried without reconfiguring the build.
  std::string config_path = DEFAULT_CONFIG_PATH;
  double target_velocity = TARGET_VELOCITY_REV_S;

  // Both flags take a value, so each strips two argv entries.  The loop does
  // not advance on a match: after the shift, index i holds what used to be
  // i+2 and still needs checking.
  for (int i = 1; i < argc - 1;) {
    const std::string arg = argv[i];
    if (arg == "--config" || arg == "--velocity") {
      if (arg == "--config") {
        config_path = argv[i + 1];
      } else {
        target_velocity = std::atof(argv[i + 1]);
      }
      // Hide it from the library's parser, which would reject it.  Shift the
      // remaining arguments down and keep the vector null-terminated.
      for (int j = i; j + 2 <= argc; j++) { argv[j] = argv[j + 2]; }
      argc -= 2;
      continue;
    }
    i++;
  }

  // A typo like `--velocity abc` yields 0.0 from atof, which would silently
  // run the motor at a standstill instead of reporting the mistake.
  if (target_velocity == 0.0) {
    std::cerr << "Note: target velocity is 0 rev/s; the motor will hold "
                 "still.\n";
  }
  if (std::abs(target_velocity) > SERVO_MAX_VELOCITY_GUARD) {
    throw ConfigError(
        "requested velocity " + std::to_string(target_velocity) +
        " rev/s exceeds the configured servo.max_velocity backstop of " +
        std::to_string(SERVO_MAX_VELOCITY_GUARD) + " rev/s");
  }

  moteus::Controller::DefaultArgProcess(argc, argv);

  moteus::Controller c([]() {
    moteus::Controller::Options options;
    options.id = DEFAULT_CONTROLLER_ID;
    return options;
  }());

  // Installed before anything can command motion, so there is no window where
  // Ctrl-C would leave the motor running.
  g_controller = &c;
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  if (APPLY_CONFIG_ON_STARTUP) {
    ApplyConfig(&c, config_path);
  }

  // Stop the board before commanding anything.  SetStop also clears a latched
  // fault, so a previous aborted run does not block this one.
  c.SetStop();

  moteus::PositionMode::Command command;
  // position = NaN means "no position target": the board integrates the
  // velocity term forever instead of servoing to a setpoint.  This is how
  // moteus does constant-speed running -- there is no separate velocity mode.
  command.position = std::numeric_limits<double>::quiet_NaN();
  command.velocity = target_velocity;
  command.maximum_torque = MAX_TORQUE_NM;
  // kp_scale/kd_scale are left at their default 1.0 so the board's configured
  // position PID does the regulating.  Zeroing them, as the health check does
  // for its pure-torque steps, would leave feedforward torque as the only
  // term and the speed would not be held against load.
  //
  // The watchdog is the safety net rule 3 of CLAUDE.md asks for: if this
  // process dies or the link drops, the board stops itself rather than
  // spinning on with the last command.  It must exceed the loop period below
  // by a comfortable margin or normal jitter will trip it.
  command.watchdog_timeout = COMMAND_WATCHDOG_S;

  std::cout << "Running at " << target_velocity << " rev/s ("
            << (target_velocity * 60.0) << " RPM), max torque "
            << MAX_TORQUE_NM << " Nm.  Ctrl-C to stop.\n";

  while (!g_shutdown.load()) {
    const auto maybe_result = c.SetPosition(command);
    if (maybe_result) {
      const auto& v = maybe_result->values;
      std::cout << "Mode: " << static_cast<int>(v.mode)
                << " Fault: " << static_cast<int>(v.fault)
                << " Position: " << v.position
                << " Velocity: " << v.velocity
                << " Torque: " << v.torque
                << std::endl;

      // A fault means the board has already stopped driving; continuing to
      // spam commands just hides why it quit.
      if (v.fault != 0) {
        std::cerr << "\nController faulted (code "
                  << static_cast<int>(v.fault) << "); stopping.\n";
        break;
      }
    }
    ::usleep(LOOP_PERIOD_US);
  }

  // Reached on clean exit and on fault.  The signal handler has already sent
  // this on Ctrl-C, but a second stop is harmless and covers the other paths.
  c.SetStop();
  std::cout << "\nStopped.\n";
  return 0;
} catch (const ControllerRejected& e) {
  // The board answered, so it is reachable; only the setting was refused.
  std::cerr << "Error: " << e.what() << "\n\n"
            << "The controller is reachable -- it replied -- but refused\n"
            << "that setting.  Either the key does not exist on this\n"
            << "firmware (check `moteus_tool -t 1 --dump-config`), or the\n"
            << "value is out of range.  Re-run with\n"
            << "-DAPPLY_CONFIG_ON_STARTUP=OFF to skip the config push.\n";
  return 1;
} catch (const ConfigError& e) {
  // A bad config file is our own problem, not a missing controller; the
  // transport hint below would only send the reader down the wrong path.
  std::cerr << "Config error: " << e.what() << "\n";
  return 1;
} catch (const std::exception& e) {
  std::cerr << "Error: " << e.what() << "\n\n"
            << "No moteus controller reachable.  Check that an fdcanusb is\n"
            << "plugged in (/dev/ttyACM*) or a socketcan interface is up,\n"
            << "then re-run.  See README.md for setup details.\n";
  return 1;
}