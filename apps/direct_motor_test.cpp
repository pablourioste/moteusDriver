#include <unistd.h>

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "drivers/MoteusConfig.hpp"
#include "drivers/MoteusDriverWrapper.hpp"
#include "moteus.h"
#include "testing/CsvLogger.hpp"
#include "testing/MotorValidator.hpp"

namespace moteus = mjbots::moteus;

namespace {

// Set from the signal handler, polled by the menu loop.  The handler itself
// must stay async-signal-safe, so it does nothing but flip this flag and
// stop the board -- no printing, no allocation.
std::atomic<bool> g_interrupted{false};

// The controller the handler stops.  Raw, non-owning: the handler cannot take
// a lock or touch a shared_ptr refcount safely.
moteus::Controller* g_controller = nullptr;

void HandleSigint(int) {
  g_interrupted.store(true);
  if (g_controller) {
    // This is the whole reason the handler exists.  main.cpp's `while(true)`
    // loop leaves the board holding position and drawing current on Ctrl-C;
    // a program that deliberately applies torque must not repeat that.
    g_controller->SetStop();
  }
}

void PrintBanner(int id) {
  std::cout << "\n"
            << "  ============================================================\n"
            << "   moteus motor health check -- controller ID " << id << "\n"
            << "  ============================================================\n"
            << "\n"
            << "  SAFETY: tests 2, 3 and 5 apply torque and WILL move the\n"
            << "  motor.  Clear the mechanism, secure the rig, and keep the\n"
            << "  power switch in reach.  Ctrl-C stops the board at any\n"
            << "  point.\n";
}

void PrintMenu() {
  std::cout << "\n  Select:\n"
            << "    1) Connectivity & voltage sanity   (no motion)\n"
            << "    2) Direction & encoder feedback    (moves)\n"
            << "    3) Torque ramp & step response     (moves)\n"
            << "    4) Thermal & fault inspection      (no motion)\n"
            << "    5) Watchdog & emergency stop       (moves)\n"
            << "    6) Velocity hold, closed loop      (moves)\n"
            << "    v) Velocity hold at a speed you enter (moves)\n"
            << "    a) Run all in sequence\n"
            << "    q) Quit (stops the board first)\n"
            << "\n  choice> " << std::flush;
}

bool ConfirmMotion() {
  std::cout << "\n  This test applies torque and will move the motor.\n"
            << "  Type 'yes' to proceed: " << std::flush;
    std::string answer;
  if (!std::getline(std::cin, answer)) { return false; }
  return answer == "yes";
}

}  // namespace

int main(int argc, char** argv) try {
  bool run_all = false;
  bool assume_yes = false;
  bool apply_config = false;
  std::string config_path;

  // Strip our own flags before the library parser sees them; it rejects
  // anything it does not recognise.
  for (int i = 1; i < argc;) {
    const std::string arg = argv[i];
    if (arg == "--run-all" || arg == "--yes") {
      if (arg == "--run-all") { run_all = true; }
      if (arg == "--yes") { assume_yes = true; }
      for (int j = i; j + 1 <= argc; j++) { argv[j] = argv[j + 1]; }
      argc -= 1;
      continue;
    }
    // Pushing the config is opt-in here, unlike the balancer.  The health
    // check is often run precisely because the board is in an unknown state,
    // and silently rewriting its configuration first would mask what is
    // wrong with it.
    if (arg == "--apply-config" && i + 1 < argc) {
      apply_config = true;
      config_path = argv[i + 1];
      for (int j = i; j + 2 <= argc; j++) { argv[j] = argv[j + 2]; }
      argc -= 2;
      continue;
    }
    i++;
  }

  moteus::Controller::DefaultArgProcess(argc, argv);

  moteus::Controller controller([]() {
    moteus::Controller::Options options;
    options.id = DEFAULT_CONTROLLER_ID;
    return options;
  }());

  g_controller = &controller;
  std::signal(SIGINT, HandleSigint);
  std::signal(SIGTERM, HandleSigint);

  testing::CsvLogger logger("logs");
  testing::MotorValidator validator(&controller, &logger);

  PrintBanner(DEFAULT_CONTROLLER_ID);
  std::cout << "  logging to " << logger.path() << "\n";

  if (apply_config) {
    // Reuses the driver's config push -- the same ReadConfig/ApplyConfig
    // path, retries included -- rather than a second copy of that logic.
    auto config = cube::MoteusConfig::FromBuildDefaults();
    config.config_path = config_path;
    config.apply_config_on_startup = true;
    cube::MoteusDriverWrapper wrapper(config);

    std::cout << "  applying " << config_path << "...\n";
    std::string error;
    if (!wrapper.initialize(&error)) {
      std::cerr << "\n  config push failed: " << error << "\n";
      return 1;
    }
  }

  std::vector<testing::TestResult> results;

  if (run_all) {
    results = validator.RunAll();
    validator.Stop();
    const bool ok = testing::PrintSummary(results, logger.path());
    std::cout << "  wrote " << logger.rows() << " samples\n\n";
    return ok ? 0 : 1;
  }

  while (!g_interrupted.load()) {
    PrintMenu();
    std::string choice;
    if (!std::getline(std::cin, choice)) { break; }
    if (choice.empty()) { continue; }

    if (choice == "q" || choice == "Q") { break; }

    if (choice == "a" || choice == "A") {
      if (!assume_yes && !ConfirmMotion()) {
        std::cout << "  aborted.\n";
        continue;
      }
      results = validator.RunAll();
      testing::PrintSummary(results, logger.path());
      continue;
    }

    const bool moves = (choice == "2" || choice == "3" || choice == "5" ||
                        choice == "6" || choice == "v" || choice == "V");
    if (moves && !assume_yes && !ConfirmMotion()) {
      std::cout << "  aborted.\n";
      continue;
    }

    // Prompted before the test runs rather than inside it, so the validator
    // stays free of console input and can be driven from a script too.
    double custom_velocity = 0.0;
    if (choice == "v" || choice == "V") {
      std::cout << "  speed in rev/s (1.0 = 60 RPM, negative reverses): "
                << std::flush;
      std::string speed;
      if (!std::getline(std::cin, speed)) { break; }
      custom_velocity = std::atof(speed.c_str());
      if (custom_velocity == 0.0) {
        std::cout << "  not a usable speed; aborted.\n";
        continue;
      }
    }

    testing::TestResult r;
    if (choice == "1") {
      r = validator.TestConnectivity();
    } else if (choice == "2") {
      r = validator.TestDirection();
    } else if (choice == "3") {
      r = validator.TestTorqueRamp();
    } else if (choice == "4") {
      r = validator.TestThermal();
    } else if (choice == "5") {
      r = validator.TestWatchdog();
    } else if (choice == "6") {
      r = validator.TestVelocityHold();
    } else if (choice == "v" || choice == "V") {
      r = validator.TestVelocityHold(custom_velocity);
    } else {
      std::cout << "  unrecognised choice: " << choice << "\n";
      continue;
    }
    results.push_back(r);
  }

  // Belt and braces: the handler already stopped the board on Ctrl-C, but a
  // clean 'q' exit or an EOF gets here without one.
  validator.Stop();

  if (!results.empty()) {
    const bool ok = testing::PrintSummary(results, logger.path());
    std::cout << "  wrote " << logger.rows() << " samples\n\n";
    return ok ? 0 : 1;
  }

  std::cout << "\n  no tests run; board stopped.\n\n";
  return 0;
} catch (const std::exception& e) {
  // Never leave the motor energised because of an exception on the way out.
  if (g_controller) { g_controller->SetStop(); }
  std::cerr << "\nError: " << e.what() << "\n\n"
            << "No moteus controller reachable.  Check that an fdcanusb is\n"
            << "plugged in (/dev/ttyACM*) or a socketcan interface is up,\n"
            << "and that nothing else holds the port (fuser -v /dev/ttyACM0).\n";
  return 1;
}
