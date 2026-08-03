#include "testing/MotorValidator.hpp"

#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>

namespace testing {
namespace {

namespace moteus = mjbots::moteus;

// Colour is emitted only to a terminal; piping the run into a file should not
// bury the verdicts in escape sequences.
bool UseColour() {
  static const bool tty = ::isatty(STDOUT_FILENO) != 0;
  return tty;
}

const char* Colour(const char* code) { return UseColour() ? code : ""; }

const char* kReset = "\033[0m";
const char* kGreen = "\033[32m";
const char* kRed = "\033[31m";
const char* kYellow = "\033[33m";
const char* kGrey = "\033[90m";
const char* kBold = "\033[1m";

std::string Tag(Status s) {
  switch (s) {
    case Status::kPass:
      return std::string(Colour(kGreen)) + "[ PASS ]" + Colour(kReset);
    case Status::kFail:
      return std::string(Colour(kRed)) + "[ FAIL ]" + Colour(kReset);
    case Status::kWarn:
      return std::string(Colour(kYellow)) + "[ WARN ]" + Colour(kReset);
    case Status::kSkip:
      return std::string(Colour(kGrey)) + "[ SKIP ]" + Colour(kReset);
  }
  return "[ ???? ]";
}

std::string Fmt(double v, int precision = 3) {
  if (std::isnan(v)) { return "n/a"; }
  std::ostringstream os;
  os << std::fixed << std::setprecision(precision) << v;
  return os.str();
}

void Header(int index, const std::string& name) {
  std::cout << "\n" << Colour(kBold) << "Test " << index << " - " << name
            << Colour(kReset) << "\n"
            << "  " << std::string(60, '-') << "\n";
}

void Line(const std::string& text) { std::cout << "  " << text << std::endl; }

// Overwrites one line in place, for the live readouts during a ramp.
void Live(const std::string& text) {
  if (UseColour()) {
    std::cout << "\r  " << text << "\033[K" << std::flush;
  } else {
    std::cout << "  " << text << std::endl;
  }
}

void EndLive() {
  if (UseColour()) { std::cout << std::endl; }
}

void SleepMs(int ms) { ::usleep(static_cast<useconds_t>(ms) * 1000); }

}  // namespace

MotorValidator::MotorValidator(moteus::Controller* controller,
                               CsvLogger* logger,
                               const ValidatorLimits& limits)
    : controller_(controller), logger_(logger), limits_(limits) {}

bool MotorValidator::Step(double torque_nm, double watchdog_s,
                          moteus::Query::Result* out) {
  moteus::PositionMode::Command cmd;
  // Pure torque: no position target, and both gains scaled out.  Leaving
  // kp_scale at its default would make the board chase whatever position it
  // happened to latch, fighting the feedforward term we are trying to test.
  cmd.position = std::numeric_limits<double>::quiet_NaN();
  cmd.velocity = 0.0;
  cmd.kp_scale = 0.0;
  cmd.kd_scale = 0.0;
  cmd.feedforward_torque = torque_nm;
  // A ceiling slightly above the request, so the board still clamps runaway
  // current but does not clip the very torque under test.
  cmd.maximum_torque = std::abs(torque_nm) + 0.05;
  if (!std::isnan(watchdog_s)) { cmd.watchdog_timeout = watchdog_s; }

  const auto reply = controller_->SetPosition(cmd);
  if (!reply) { return false; }

  logger_->Log(reply->values, torque_nm);
  if (out) { *out = reply->values; }
  return true;
}

bool MotorValidator::Hold(double torque_nm, int duration_ms, double* peak_temp,
                          double* peak_abs_torque) {
  const int steps = duration_ms / limits_.sample_period_ms;
  moteus::Query::Result state;

  for (int i = 0; i < steps; i++) {
    if (!Step(torque_nm, std::numeric_limits<double>::quiet_NaN(), &state)) {
      return false;
    }
    if (peak_temp && !std::isnan(state.temperature)) {
      *peak_temp = std::max(*peak_temp, state.temperature);
    }
    if (peak_abs_torque && !std::isnan(state.torque)) {
      *peak_abs_torque = std::max(*peak_abs_torque, std::abs(state.torque));
    }
    SleepMs(limits_.sample_period_ms);
  }
  return true;
}

bool MotorValidator::Stop() {
  const auto reply = controller_->SetStop();
  if (!reply) { return false; }
  logger_->Log(reply->values, 0.0);
  return true;
}

TestResult MotorValidator::TestConnectivity() {
  TestResult r{"Connectivity & Voltage Sanity", Status::kSkip, ""};
  Header(1, r.name);

  // Clear any latched fault first, so a stale fault from a previous aborted
  // run is not reported as this run's startup state.
  controller_->SetStop();
  SleepMs(50);

  const auto reply = controller_->SetQuery();
  if (!reply) {
    r.status = Status::kFail;
    r.detail = "no reply from controller";
    Line(Tag(r.status) + " no reply -- check transport and CAN ID");
    return r;
  }

  const auto& v = reply->values;
  logger_->Log(v, 0.0);

  Line("bus voltage  : " + Fmt(v.voltage, 2) + " V");
  Line("temperature  : " + Fmt(v.temperature, 1) + " C");
  Line("mode / fault : " + std::to_string(static_cast<int>(v.mode)) + " / " +
       std::to_string(static_cast<int>(v.fault)));

  std::ostringstream detail;
  detail << Fmt(v.voltage, 2) << " V, fault " << static_cast<int>(v.fault);
  r.detail = detail.str();

  if (v.fault != 0) {
    r.status = Status::kFail;
    Line(Tag(r.status) + " controller reports fault code " +
         std::to_string(static_cast<int>(v.fault)));
    return r;
  }
  if (std::isnan(v.voltage) || v.voltage < limits_.min_bus_voltage ||
      v.voltage > limits_.max_bus_voltage) {
    r.status = Status::kFail;
    Line(Tag(r.status) + " bus voltage outside " +
         Fmt(limits_.min_bus_voltage, 1) + "-" +
         Fmt(limits_.max_bus_voltage, 1) + " V");
    return r;
  }

  r.status = Status::kPass;
  Line(Tag(r.status) + " connected, voltage in range, no faults");
  return r;
}

TestResult MotorValidator::TestDirection() {
  TestResult r{"Direction & Encoder Feedback", Status::kSkip, ""};
  Header(2, r.name);

  moteus::Query::Result start;
  if (!Step(0.0, std::numeric_limits<double>::quiet_NaN(), &start)) {
    r.status = Status::kFail;
    r.detail = "no reply";
    Line(Tag(r.status) + " no reply from controller");
    return r;
  }

  Line("applying +" + Fmt(limits_.direction_torque_nm) + " Nm for " +
       std::to_string(limits_.direction_duration_ms) + " ms");

  const int steps = limits_.direction_duration_ms / limits_.sample_period_ms;
  moteus::Query::Result state = start;
  double velocity_sum = 0.0;
  int velocity_n = 0;

  for (int i = 0; i < steps; i++) {
    if (!Step(limits_.direction_torque_nm,
              std::numeric_limits<double>::quiet_NaN(), &state)) {
      Stop();
      r.status = Status::kFail;
      r.detail = "lost contact mid-test";
      Line(Tag(r.status) + " lost contact with controller");
      return r;
    }
    if (!std::isnan(state.velocity)) {
      velocity_sum += state.velocity;
      velocity_n++;
    }
    Live("pos " + Fmt(state.position) + " rev   vel " + Fmt(state.velocity) +
         " rev/s");
    SleepMs(limits_.sample_period_ms);
  }
  EndLive();

  Stop();

  const double travel = state.position - start.position;
  const double mean_velocity = velocity_n ? velocity_sum / velocity_n : 0.0;

  Line("delta position: " + Fmt(travel) + " rev");
  Line("mean velocity : " + Fmt(mean_velocity) + " rev/s");

  std::ostringstream detail;
  detail << "dpos " << Fmt(travel) << " rev";
  r.detail = detail.str();

  if (std::abs(travel) < limits_.direction_min_travel_rev) {
    r.status = Status::kFail;
    Line(Tag(r.status) +
         " motor did not move -- locked, unpowered, or torque too low");
    return r;
  }
  if (travel < 0.0 || mean_velocity < 0.0) {
    // Not a hardware fault: the encoder or phase order is simply mirrored
    // relative to the command sign.  Flagged rather than failed, because the
    // fix is a config change and the rig is otherwise healthy.
    r.status = Status::kWarn;
    Line(Tag(r.status) +
         " SIGN MISMATCH: positive torque produced negative motion");
    Line("        fix with motor_position.output.sign or "
         "motor_position.rotor_to_output_ratio sign");
    return r;
  }

  r.status = Status::kPass;
  Line(Tag(r.status) + " positive torque produced positive motion");
  return r;
}

TestResult MotorValidator::TestTorqueRamp() {
  TestResult r{"Torque Ramp & Step Response", Status::kSkip, ""};
  Header(3, r.name);

  double peak_temp = -std::numeric_limits<double>::infinity();
  double peak_torque = 0.0;
  // Worst absolute gap between what we asked for and what came back, sampled
  // only during the holds where the command is steady -- comparing during a
  // ramp would charge the board for our own slew rate.
  double worst_tracking_error = 0.0;

  const int ramp_steps = limits_.ramp_up_ms / limits_.sample_period_ms;

  for (const double sign : {1.0, -1.0}) {
    const double peak = sign * limits_.ramp_peak_torque_nm;
    Line((sign > 0 ? std::string("forward") : std::string("reverse")) +
         " ramp 0 -> " + Fmt(peak) + " Nm");

    moteus::Query::Result state;

    // Ramp up.
    for (int i = 0; i <= ramp_steps; i++) {
      const double t = peak * (static_cast<double>(i) / ramp_steps);
      if (!Step(t, std::numeric_limits<double>::quiet_NaN(), &state)) {
        Stop();
        r.status = Status::kFail;
        r.detail = "lost contact during ramp";
        Line(Tag(r.status) + " lost contact with controller");
        return r;
      }
      if (state.fault != 0) {
        Stop();
        r.status = Status::kFail;
        r.detail = "fault " + std::to_string(static_cast<int>(state.fault)) +
                   " at " + Fmt(t) + " Nm";
        EndLive();
        Line(Tag(r.status) + " fault " +
             std::to_string(static_cast<int>(state.fault)) + " raised at " +
             Fmt(t) + " Nm");
        return r;
      }
      if (!std::isnan(state.temperature)) {
        peak_temp = std::max(peak_temp, state.temperature);
      }
      if (!std::isnan(state.torque)) {
        peak_torque = std::max(peak_torque, std::abs(state.torque));
      }
      Live("cmd " + Fmt(t) + " Nm   meas " + Fmt(state.torque) +
           " Nm   Iq " + Fmt(state.q_current, 2) + " A   " +
           Fmt(state.temperature, 1) + " C");
      SleepMs(limits_.sample_period_ms);
    }
    EndLive();

    // Hold at peak, measuring tracking where the command is constant.
    const int hold_steps = limits_.ramp_hold_ms / limits_.sample_period_ms;
    for (int i = 0; i < hold_steps; i++) {
      if (!Step(peak, std::numeric_limits<double>::quiet_NaN(), &state)) {
        Stop();
        r.status = Status::kFail;
        r.detail = "lost contact during hold";
        Line(Tag(r.status) + " lost contact with controller");
        return r;
      }
      if (state.fault != 0) {
        Stop();
        r.status = Status::kFail;
        r.detail = "fault " + std::to_string(static_cast<int>(state.fault)) +
                   " at hold";
        EndLive();
        Line(Tag(r.status) + " fault " +
             std::to_string(static_cast<int>(state.fault)) + " during hold");
        return r;
      }
      if (!std::isnan(state.torque)) {
        worst_tracking_error =
            std::max(worst_tracking_error, std::abs(peak - state.torque));
        peak_torque = std::max(peak_torque, std::abs(state.torque));
      }
      if (!std::isnan(state.temperature)) {
        peak_temp = std::max(peak_temp, state.temperature);
      }
      Live("hold " + Fmt(peak) + " Nm   meas " + Fmt(state.torque) +
           " Nm   Iq " + Fmt(state.q_current, 2) + " A   " +
           Fmt(state.temperature, 1) + " C");
      SleepMs(limits_.sample_period_ms);
    }
    EndLive();

    // Ramp back down.
    for (int i = ramp_steps; i >= 0; i--) {
      const double t = peak * (static_cast<double>(i) / ramp_steps);
      if (!Step(t, std::numeric_limits<double>::quiet_NaN(), &state)) {
        Stop();
        r.status = Status::kFail;
        r.detail = "lost contact during ramp-down";
        Line(Tag(r.status) + " lost contact with controller");
        return r;
      }
      SleepMs(limits_.sample_period_ms);
    }
    Stop();
    SleepMs(200);  // let it coast to rest before reversing
  }

  std::ostringstream detail;
  detail << "peak " << Fmt(peak_torque) << " Nm, err "
         << Fmt(worst_tracking_error) << " Nm";
  r.detail = detail.str();

  Line("peak measured torque : " + Fmt(peak_torque) + " Nm");
  Line("worst tracking error : " + Fmt(worst_tracking_error) + " Nm");
  Line("peak temperature     : " + Fmt(peak_temp, 1) + " C");

  if (peak_temp > limits_.max_temperature_c) {
    r.status = Status::kFail;
    Line(Tag(r.status) + " exceeded thermal ceiling during ramp");
    return r;
  }
  // A quarter of the commanded peak is a generous bound: it catches a board
  // that is clipping or not producing torque at all, without failing a rig
  // whose torque constant is merely a little off from the calibrated kv.
  if (worst_tracking_error > limits_.ramp_peak_torque_nm * 0.25) {
    r.status = Status::kWarn;
    Line(Tag(r.status) +
         " measured torque tracks commanded poorly -- check kv / calibration");
    return r;
  }

  r.status = Status::kPass;
  Line(Tag(r.status) + " torque tracked through both directions, no trips");
  return r;
}

TestResult MotorValidator::TestThermal() {
  TestResult r{"Thermal & Fault Log Inspection", Status::kSkip, ""};
  Header(4, r.name);

  const auto reply = controller_->SetQuery();
  if (!reply) {
    r.status = Status::kFail;
    r.detail = "no reply";
    Line(Tag(r.status) + " no reply from controller");
    return r;
  }
  const auto& v = reply->values;
  logger_->Log(v, 0.0);

  Line("board temperature : " + Fmt(v.temperature, 1) + " C  (limit " +
       Fmt(limits_.max_temperature_c, 1) + " C)");
  Line("motor temperature : " + Fmt(v.motor_temperature, 1) + " C");
  Line("fault code        : " + std::to_string(static_cast<int>(v.fault)));

  std::ostringstream detail;
  detail << Fmt(v.temperature, 1) << " C, fault "
         << static_cast<int>(v.fault);
  r.detail = detail.str();

  if (v.fault != 0) {
    r.status = Status::kFail;
    Line(Tag(r.status) + " active fault after load");
    return r;
  }
  if (!std::isnan(v.temperature) && v.temperature > limits_.max_temperature_c) {
    r.status = Status::kFail;
    Line(Tag(r.status) + " board over thermal ceiling");
    return r;
  }

  r.status = Status::kPass;
  Line(Tag(r.status) + " thermals within limits, no active faults");
  return r;
}

TestResult MotorValidator::TestWatchdog() {
  TestResult r{"Watchdog & Emergency Stop", Status::kSkip, ""};
  Header(5, r.name);

  Line("commanding " + Fmt(limits_.watchdog_torque_nm) + " Nm with " +
       Fmt(limits_.watchdog_timeout_s, 2) + " s watchdog");

  moteus::Query::Result state;
  // A few cycles so the board is genuinely armed and running before we go
  // quiet; a single frame could time out before it ever entered position mode.
  for (int i = 0; i < 10; i++) {
    if (!Step(limits_.watchdog_torque_nm, limits_.watchdog_timeout_s, &state)) {
      r.status = Status::kFail;
      r.detail = "no reply";
      Line(Tag(r.status) + " no reply from controller");
      return r;
    }
    SleepMs(limits_.sample_period_ms);
  }
  Line("armed, mode " + std::to_string(static_cast<int>(state.mode)));

  Line("going silent for " + std::to_string(limits_.watchdog_silence_ms) +
       " ms (no commands sent)");
  SleepMs(limits_.watchdog_silence_ms);

  const auto reply = controller_->SetQuery();
  if (!reply) {
    r.status = Status::kFail;
    r.detail = "no reply after silence";
    Line(Tag(r.status) + " no reply after the silent window");
    return r;
  }
  logger_->Log(reply->values, 0.0);
  const int mode = static_cast<int>(reply->values.mode);
  Line("mode after silence: " + std::to_string(mode) + "  fault: " +
       std::to_string(static_cast<int>(reply->values.fault)));

  // Mode 11 is Position; anything at or below 3 is stopped/fault territory.
  // The board should have disarmed itself rather than still be driving.
  const bool disarmed = mode <= 3;

  std::ostringstream detail;
  detail << "mode " << mode << " after " << limits_.watchdog_silence_ms
         << " ms silence";
  r.detail = detail.str();

  if (!disarmed) {
    Stop();
    r.status = Status::kFail;
    Line(Tag(r.status) +
         " board still armed -- watchdog did not fire (is "
         "servo.default_timeout_s set?)");
    return r;
  }
  Line("watchdog fired, board disarmed");

  // Explicit stop, then confirm it draws nothing.
  if (!Stop()) {
    r.status = Status::kFail;
    r.detail = "stop command unanswered";
    Line(Tag(r.status) + " explicit stop was not acknowledged");
    return r;
  }
  SleepMs(50);
  const auto after = controller_->SetQuery();
  if (!after) {
    r.status = Status::kFail;
    r.detail = "no reply after stop";
    Line(Tag(r.status) + " no reply after explicit stop");
    return r;
  }
  logger_->Log(after->values, 0.0);
  Line("after stop: mode " +
       std::to_string(static_cast<int>(after->values.mode)) + "  Iq " +
       Fmt(after->values.q_current, 3) + " A  power " +
       Fmt(after->values.power, 2) + " W");

  if (!std::isnan(after->values.q_current) &&
      std::abs(after->values.q_current) > 0.5) {
    r.status = Status::kWarn;
    Line(Tag(r.status) + " still drawing current after stop");
    return r;
  }

  r.status = Status::kPass;
  Line(Tag(r.status) + " watchdog disarmed the board and stop draws no power");
  return r;
}

TestResult MotorValidator::TestVelocityHold(double override_rev_s) {
  TestResult r{"Velocity Hold (closed loop)", Status::kSkip, ""};
  Header(6, r.name);

  const double target = (override_rev_s != 0.0)
                            ? override_rev_s
                            : limits_.velocity_target_rev_s;

  Line("target " + Fmt(target) + " rev/s (" + Fmt(target * 60.0, 0) +
       " RPM), max torque " + Fmt(limits_.velocity_max_torque_nm) + " Nm");

  // Deliberately NOT the pure-torque shape Step() uses: kp_scale and kd_scale
  // stay at 1.0 so the board's own position PID regulates the speed.  That is
  // what tview's `d pos` does, and what main.cpp does.
  moteus::PositionMode::Command cmd;
  cmd.position = std::numeric_limits<double>::quiet_NaN();
  cmd.velocity = target;
  cmd.maximum_torque = limits_.velocity_max_torque_nm;

  const int spinup_steps = limits_.velocity_spinup_ms / limits_.sample_period_ms;
  const int hold_steps = limits_.velocity_hold_ms / limits_.sample_period_ms;

  double velocity_sum = 0.0;
  int velocity_n = 0;
  double peak_temp = -std::numeric_limits<double>::infinity();
  moteus::Query::Result state;

  for (int i = 0; i < spinup_steps + hold_steps; i++) {
    const auto reply = controller_->SetPosition(cmd);
    if (!reply) {
      Stop();
      r.status = Status::kFail;
      r.detail = "no reply";
      EndLive();
      Line(Tag(r.status) + " lost contact with controller");
      return r;
    }
    state = reply->values;
    logger_->Log(state, std::numeric_limits<double>::quiet_NaN());

    if (state.fault != 0) {
      Stop();
      r.status = Status::kFail;
      r.detail = "fault " + std::to_string(static_cast<int>(state.fault));
      EndLive();
      Line(Tag(r.status) + " fault " +
           std::to_string(static_cast<int>(state.fault)) + " during run");
      return r;
    }

    // Only the steady-state window counts toward the verdict; averaging the
    // spin-up ramp in would drag the mean below target for no good reason.
    if (i >= spinup_steps && !std::isnan(state.velocity)) {
      velocity_sum += state.velocity;
      velocity_n++;
    }
    if (!std::isnan(state.temperature)) {
      peak_temp = std::max(peak_temp, state.temperature);
    }

    Live(std::string(i < spinup_steps ? "spin-up" : "hold   ") + "  vel " +
         Fmt(state.velocity) + " rev/s   torque " + Fmt(state.torque) +
         " Nm   " + Fmt(state.temperature, 1) + " C");
    SleepMs(limits_.sample_period_ms);
  }
  EndLive();

  Stop();

  const double mean_velocity = velocity_n ? velocity_sum / velocity_n : 0.0;
  const double error = std::abs(mean_velocity - target);

  Line("mean held velocity : " + Fmt(mean_velocity) + " rev/s");
  Line("error vs target    : " + Fmt(error) + " rev/s");
  Line("peak temperature   : " + Fmt(peak_temp, 1) + " C");

  std::ostringstream detail;
  detail << Fmt(mean_velocity) << " / " << Fmt(target) << " rev/s";
  r.detail = detail.str();

  // A motor that never turned is a different failure from one that turned at
  // the wrong speed, and the fix is different too, so say which happened.
  if (std::abs(mean_velocity) < std::abs(target) * 0.05) {
    r.status = Status::kFail;
    Line(Tag(r.status) +
         " motor did not turn -- no torque reaching the motor");
    Line("        check servo.pid_position gains and phase wiring");
    return r;
  }
  if (mean_velocity * target < 0.0) {
    r.status = Status::kWarn;
    Line(Tag(r.status) + " motor turned the WRONG WAY");
    return r;
  }
  if (error > std::abs(target) * limits_.velocity_tolerance) {
    r.status = Status::kWarn;
    Line(Tag(r.status) + " speed held but outside " +
         Fmt(limits_.velocity_tolerance * 100.0, 0) +
         "% tolerance -- PID likely untuned");
    return r;
  }

  r.status = Status::kPass;
  Line(Tag(r.status) + " held commanded speed within tolerance");
  return r;
}

std::vector<TestResult> MotorValidator::RunAll() {
  std::vector<TestResult> results;

  results.push_back(TestConnectivity());
  if (results.back().status == Status::kFail) {
    // Everything downstream needs a live, fault-free board.  Record the
    // remaining tests as skipped so the summary is honest about what ran.
    results.push_back({"Direction & Encoder Feedback", Status::kSkip,
                       "skipped: connectivity failed"});
    results.push_back({"Torque Ramp & Step Response", Status::kSkip,
                       "skipped: connectivity failed"});
    results.push_back({"Thermal & Fault Log Inspection", Status::kSkip,
                       "skipped: connectivity failed"});
    results.push_back({"Watchdog & Emergency Stop", Status::kSkip,
                       "skipped: connectivity failed"});
    results.push_back({"Velocity Hold (closed loop)", Status::kSkip,
                       "skipped: connectivity failed"});
    return results;
  }

  results.push_back(TestDirection());
  results.push_back(TestTorqueRamp());
  results.push_back(TestThermal());
  results.push_back(TestWatchdog());
  results.push_back(TestVelocityHold());

  Stop();
  return results;
}

bool PrintSummary(const std::vector<TestResult>& results,
                  const std::string& log_path) {
  std::cout << "\n"
            << Colour(kBold) << "  Summary" << Colour(kReset) << "\n"
            << "  " << std::string(72, '=') << "\n";

  bool all_ok = true;
  int index = 0;
  for (const auto& r : results) {
    index++;
    if (r.status == Status::kFail) { all_ok = false; }

    std::ostringstream row;
    row << "  " << index << ". " << std::left << std::setw(34) << r.name;
    std::cout << row.str() << Tag(r.status);
    if (!r.detail.empty()) {
      std::cout << "  " << Colour(kGrey) << r.detail << Colour(kReset);
    }
    std::cout << "\n";
  }

  std::cout << "  " << std::string(72, '=') << "\n"
            << "  log: " << log_path << "\n\n";

  if (all_ok) {
    std::cout << "  " << Colour(kGreen) << Colour(kBold)
              << "All checks passed." << Colour(kReset)
              << "  Motor is cleared for control-loop use.\n\n";
  } else {
    std::cout << "  " << Colour(kRed) << Colour(kBold)
              << "One or more checks FAILED." << Colour(kReset)
              << "  Do not run the balance loop until resolved.\n\n";
  }
  return all_ok;
}

}  // namespace testing
