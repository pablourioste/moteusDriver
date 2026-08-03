#pragma once

#include <string>
#include <vector>

#include "moteus.h"
#include "testing/CsvLogger.hpp"

namespace testing {

enum class Status { kPass, kFail, kWarn, kSkip };

struct TestResult {
  std::string name;
  Status status = Status::kSkip;
  // Shown next to the verdict in the summary table: the measured number that
  // decided it, so a failure is diagnosable without reopening the CSV.
  std::string detail;
};

// Thresholds every check is judged against.  Gathered into one struct so the
// limits are visible and overridable in one place rather than scattered as
// literals through the test bodies.
struct ValidatorLimits {
  double min_bus_voltage = 10.0;    // V
  double max_bus_voltage = 30.0;    // V
  double max_temperature_c = 60.0;  // board temperature ceiling

  // Test 2 nudges the motor just hard enough to see which way it goes.
  double direction_torque_nm = 0.05;
  int direction_duration_ms = 500;
  // Below this the motor is treated as locked rather than as having moved --
  // encoder dither on a 65536 CPR encoder is well under this.
  double direction_min_travel_rev = 0.002;

  // Test 3 ramp profile.
  double ramp_peak_torque_nm = 0.5;
  int ramp_up_ms = 2000;
  int ramp_hold_ms = 1000;

  // Test 5 watchdog.
  double watchdog_torque_nm = 0.1;
  double watchdog_timeout_s = 0.1;
  int watchdog_silence_ms = 300;

  // Shared loop period.  100 Hz matches the driver's own cadence.
  int sample_period_ms = 10;

  // Test 6 (velocity hold).  Unlike the torque tests this runs closed-loop on
  // the board's position PID, which is what the real control loop uses.
  double velocity_target_rev_s = 1.0;
  int velocity_spinup_ms = 1000;
  int velocity_hold_ms = 3000;
  double velocity_max_torque_nm = 0.5;
  // Fraction of the target the measured speed must reach to pass.  Generous,
  // because an untuned position PID tracks speed loosely even when the drive
  // stage is perfectly healthy.
  double velocity_tolerance = 0.30;
};

// Runs the health checks against one controller.
//
// Every test that applies torque is written to return the motor to rest and
// leave the board stopped, including on the failure paths -- an aborted check
// that leaves the controller energised is worse than no check at all.
class MotorValidator {
 public:
  MotorValidator(mjbots::moteus::Controller* controller, CsvLogger* logger,
                 const ValidatorLimits& limits = {});

  // Individual checks, in the order they are meant to run.  Each records its
  // own row(s) to the CSV and prints live progress.
  TestResult TestConnectivity();
  TestResult TestDirection();
  TestResult TestTorqueRamp();
  TestResult TestThermal();
  TestResult TestWatchdog();

  // Closed-loop constant-speed run at `limits_.velocity_target_rev_s`.  This
  // is the same command shape main.cpp uses, so a pass here means the driver
  // should work; `override_rev_s` retargets it without rebuilding.
  TestResult TestVelocityHold(double override_rev_s = 0.0);

  // Runs all five in sequence.  Stops early if connectivity fails, since
  // every later test would only produce noise against a board that is not
  // talking.  Returns every result gathered, including the ones skipped.
  std::vector<TestResult> RunAll();

  // Commands a stop and confirms the board reports itself unpowered.  Safe to
  // call at any point, including from a signal handler path.
  bool Stop();

 private:
  // One command/response round trip at `torque_nm` in pure-torque mode,
  // logging the reply.  Returns false if the controller did not answer.
  bool Step(double torque_nm, double watchdog_s,
            mjbots::moteus::Query::Result* out);

  // Holds `torque_nm` for `duration_ms`, logging throughout.  `peak_temp` and
  // `peak_abs_torque`, when non-null, accumulate across the hold.
  bool Hold(double torque_nm, int duration_ms, double* peak_temp,
            double* peak_abs_torque);

  mjbots::moteus::Controller* const controller_;
  CsvLogger* const logger_;
  const ValidatorLimits limits_;
};

// Renders the final table.  Returns true if no test failed.
bool PrintSummary(const std::vector<TestResult>& results,
                  const std::string& log_path);

}  // namespace testing
