// core_check -- INTEGRATION.md S1.  The first step of the Teensy port.
//
// Proves that the SAME src/core/*.cpp the host CMake build compiles also
// compiles, links and runs on the Teensy, with byte-identical types.  It
// touches no hardware: no IMU, no CAN, no transceiver, nothing wired.
//
// Why this exists as its own sketch:
//
//   The whole port rests on core/ being the intersection of two build
//   systems -- referenced in place by both, never copied.  A copy means the
//   host simulation and the hardware control law drift, and you end up
//   debugging the difference between your simulation and your rig.  This
//   sketch is the check that the sharing actually happens, run BEFORE any
//   driver work depends on it.
//
// It answers three questions, in order of how badly a wrong answer hurts:
//
//   [1] Do the shared types have the same layout on both targets?
//       ImuData and MotorState cross between builds.  If double were 4
//       bytes here and 8 on the host, every offset would differ and the
//       control law would read garbage -- silently.
//
//   [2] Does the NaN configuration gate survive the port?
//       Both Config structs ship as NaN/-1 sentinels, and firstUnsetField()
//       is what refuses to run on them.  That refusal is a safety property:
//       NaN comparisons are all false, so an unset max_tilt_rad would not
//       fail loudly -- the tilt e-stop would cease to exist.  If the gate
//       did not survive, an unconfigured balancer would arm.
//
//   [3] Does the double-precision math actually run?
//       Cortex-M7 has a VFPv5 FPU with hardware doubles, which is why core/
//       is double throughout and needs no float rewrite.
//
//   pio run -e core_check              (WSL) build, then flash from Windows
//
// Compare the printed sizeofs against the host:
//   cmake --build build && ./build/cube_balancer --dry-run

#include <Arduino.h>

#include "core/BalancingController.hpp"
#include "core/StateEstimator.hpp"
#include "core/Types.hpp"

namespace {

// Report a sizeof against what the host build produces.  These two structs
// are the ABI contract between the builds -- they are what the estimator
// and controller read every cycle.  Record the numbers the first time this
// runs; every later phase assumes they have not moved.
void reportSize(const char* name, size_t actual) {
  Serial.printf("  sizeof(%-12s) = %u bytes\r\n", name, (unsigned)actual);
}

// The gate itself.  firstUnsetField() returns the name of the first
// sentinel still in place, or nullptr once every tunable is set.  On a
// freshly built binary it MUST return a name -- the gains cannot be guessed
// without measuring the rig, so nothing should be configured yet.
void reportGate(const char* which, const char* unset_field) {
  if (unset_field != nullptr) {
    Serial.printf("  %-20s gate ACTIVE, first unset: %s\r\n", which,
                  unset_field);
  } else {
    Serial.printf("  %-20s gate OPEN -- fully configured\r\n", which);
  }
}

}  // namespace

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println();
  Serial.println("=== core_check -- S1 build wiring ===");
  Serial.println();

  // --- [1] ABI ---------------------------------------------------------
  //
  // If sizeof(double) reads 4, stop here: the FPU assumption is wrong and
  // every number in core/ is software-emulated at a cost the 5 ms control
  // budget cannot absorb.
  Serial.println("Shared type layout (must match the host build):");
  reportSize("ImuData", sizeof(cube::ImuData));
  reportSize("MotorState", sizeof(cube::MotorState));
  reportSize("BodyState", sizeof(cube::BodyState));
  reportSize("double", sizeof(double));
  Serial.println();

  // --- [2] the NaN gate ------------------------------------------------
  //
  // Default-constructed, so every tunable is still its sentinel.  Both
  // lines below are EXPECTED to report an unset field.  A gate reading
  // "OPEN" on a default construction would mean the sentinels were
  // replaced by plausible defaults somewhere -- which is exactly the
  // failure the gate exists to prevent.
  Serial.println("Configuration gate (both MUST report an unset field):");
  {
    const cube::StateEstimator estimator;
    const cube::BalancingController controller;
    reportGate("StateEstimator", estimator.firstUnsetField());
    reportGate("BalancingController", controller.firstUnsetField());
  }
  Serial.println();

  // --- [3] the math ----------------------------------------------------
  //
  // accelAngle() is one of the user-written scaffolds and currently
  // returns NAN by design, so this does not check a control law -- it
  // checks that calling into cube_core's compiled code from firmware links
  // and executes at all.
  Serial.println("Linkage:");
  {
    cube::StateEstimator estimator;
    cube::ImuData sample;
    sample.accel_x = 0.0;
    sample.accel_y = 0.0;
    sample.accel_z = 9.80665;
    sample.valid = true;

    const double angle = estimator.accelAngle(sample);
    Serial.printf("  accelAngle() returned %s\r\n",
                  isnan(angle) ? "NaN (expected -- scaffold unwritten)"
                               : "a value");

    // Proves the enum-to-string helper links too; it is what the balancer
    // prints on a safety trip.
    Serial.printf("  ToString(kUnconfigured) = %s\r\n",
                  cube::ToString(cube::SafetyState::kUnconfigured));
  }

  Serial.println();
  Serial.println("PASS if: sizeofs match the host, both gates report ACTIVE,");
  Serial.println("         and sizeof(double) is 8.");
}

void loop() {
  // Slow heartbeat only.  Everything this sketch has to say was said in
  // setup(); the blink is here so a blank serial console can be told apart
  // from a board that never booted.
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1900);
}
