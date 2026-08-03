#include "testing/CsvLogger.hpp"

#include <sys/stat.h>

#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace testing {
namespace {

long long NowMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

// Local time, formatted the same way moteus_tool names its calibration dumps
// (moteus-cal-<serial>-20260802T091153.log), so the two sit together sensibly
// in a directory listing.
std::string Timestamp() {
  const auto now = std::chrono::system_clock::now();
  const auto secs = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  ::localtime_r(&secs, &tm);

  std::ostringstream os;
  os << std::put_time(&tm, "%Y%m%dT%H%M%S");
  return os.str();
}

// The controller reports an unset/unavailable register as NaN.  Writing a
// bare "nan" makes downstream parsers guess; an empty cell is unambiguous in
// every CSV reader we are likely to point at this.
void WriteCell(std::ostream& out, double value) {
  if (std::isnan(value)) { return; }
  out << value;
}

}  // namespace

CsvLogger::CsvLogger(const std::string& directory) {
  // mkdir rather than <filesystem>: this has to build under the same
  // CMAKE_CXX_STANDARD 17 as the rest of the project, and older libstdc++
  // still wants -lstdc++fs for std::filesystem.  One syscall avoids that.
  ::mkdir(directory.c_str(), 0755);  // EEXIST is fine and expected.

  path_ = directory + "/motor_test_" + Timestamp() + ".csv";
  out_.open(path_);
  if (!out_) {
    throw std::runtime_error("cannot open log file for writing: " + path_);
  }

  out_ << "timestamp_ms,bus_voltage,temperature_c,mode,fault_code,"
       << "position_rev,velocity_rev_s,commanded_torque_nm,"
       << "measured_torque_nm,q_current_a\n";
  out_.flush();

  start_ms_ = NowMs();
}

CsvLogger::~CsvLogger() {
  if (out_.is_open()) { out_.close(); }
}

void CsvLogger::Log(const mjbots::moteus::Query::Result& state,
                    double commanded_torque_nm) {
  // Enough digits to distinguish encoder counts on a 65536 CPR encoder
  // (~1.5e-5 rev) without dumping full double noise into every cell.
  out_ << std::fixed << std::setprecision(6);

  out_ << (NowMs() - start_ms_) << ',';
  WriteCell(out_, state.voltage);            out_ << ',';
  WriteCell(out_, state.temperature);        out_ << ',';
  out_ << static_cast<int>(state.mode) << ',';
  out_ << static_cast<int>(state.fault) << ',';
  WriteCell(out_, state.position);           out_ << ',';
  WriteCell(out_, state.velocity);           out_ << ',';
  WriteCell(out_, commanded_torque_nm);      out_ << ',';
  WriteCell(out_, state.torque);             out_ << ',';
  WriteCell(out_, state.q_current);          out_ << '\n';

  // Flushed per row deliberately -- see the class comment.  At 100 Hz this is
  // a trivial cost next to the CAN round-trip that produced the sample.
  out_.flush();
  rows_++;
}

}  // namespace testing
