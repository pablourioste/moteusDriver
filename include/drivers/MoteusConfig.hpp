// Runtime configuration for the moteus driver.
//
// The fields here mirror the CMake cache variables in CMakeLists.txt.  The
// defaults are filled from the compile definitions the build already passes
// (DEFAULT_CONTROLLER_ID, MAX_TORQUE_NM, ...), so the single source of truth
// stays the build system -- this struct just carries those values to a place
// the driver can read them without every call site needing the macros.
#pragma once

#include <string>

namespace cube {

struct MoteusConfig {
  // CAN ID of the controller.  This rig has exactly one, at ID 1.
  int controller_id = 1;

  // Path to the generated current_config.cfg.  Defaults to the build-time
  // path baked in as DEFAULT_CONFIG_PATH.
  std::string config_path;

  // Push the config file at startup with `conf set`.  Matches
  // APPLY_CONFIG_ON_STARTUP.
  bool apply_config_on_startup = true;

  // Follow the push with `conf write`.  Matches PERSIST_CONFIG_TO_FLASH.
  // Off by default: settings then live in volatile memory only, which is
  // right while iterating and wrong once the rig is settled.
  bool persist_config_to_flash = false;

  // Torque ceiling passed as maximum_torque on every command, Nm.  This is
  // the driver's own clamp; the board additionally enforces
  // servo.max_current_A, which is the one that protects the hardware.
  double max_torque_nm = 0.5;

  // Per-command watchdog, seconds.  If the host stops sending for longer
  // than this the board stops itself.  Must comfortably exceed the control
  // period or normal jitter trips it: at 200 Hz (5 ms) a 0.1 s watchdog
  // leaves 20 cycles of margin.
  double watchdog_timeout_s = 0.1;

  // --- Rig-specific hardware facts --------------------------------------
  //
  // Recorded here so code and logs can refer to them, and so a mismatch
  // with the actual rig is visible in one place.  Changing these does NOT
  // reconfigure the board -- that is current_config.cfg's job.

  // Output turns per rotor turn.  1.0 = direct drive, which is how the
  // reaction wheel is mounted (wheel bolted straight to the motor shaft).
  double rotor_to_output_ratio = 1.0;

  // Motor pole count, from the calibration log.  Informational.
  int motor_poles = 24;

  // Encoder counts per revolution -- MA600 on aux2 over SPI at 6 MHz.
  int encoder_cpr = 65536;

  // Returns a config populated from the build-time compile definitions.
  // Defined in MoteusDriverWrapper.cpp, which is the only translation unit
  // that sees those macros.
  static MoteusConfig FromBuildDefaults();
};

}  // namespace cube
