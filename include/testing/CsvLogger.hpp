#pragma once

#include <fstream>
#include <string>

#include "moteus.h"

namespace testing {

// Appends one row per sampled controller state to a timestamped CSV.
//
// The file is opened once and held open for the life of the object: the
// health check samples at 100 Hz and reopening per row would dominate the
// loop.  Rows are flushed on every write anyway, because the whole point of
// this log is to survive whatever fault ends the run -- a buffered tail lost
// to a crash is exactly the data worth having.
class CsvLogger {
 public:
  // Creates logs/motor_test_<timestamp>.csv under `directory` and writes the
  // header row.  Throws std::runtime_error if the file cannot be opened.
  explicit CsvLogger(const std::string& directory = "logs");

  ~CsvLogger();

  CsvLogger(const CsvLogger&) = delete;
  CsvLogger& operator=(const CsvLogger&) = delete;

  // Records one sample.  `commanded_torque_nm` is not readable back from the
  // controller -- the board reports what it measured, not what we asked for --
  // so it is passed in by the caller that issued the command.
  void Log(const mjbots::moteus::Query::Result& state,
           double commanded_torque_nm);

  // Path of the file being written, for the closing summary.
  const std::string& path() const { return path_; }

  // Number of rows written so far, excluding the header.
  long rows() const { return rows_; }

 private:
  std::string path_;
  std::ofstream out_;
  long rows_ = 0;
  // Milliseconds since construction, so the timestamp column starts at 0 for
  // each run rather than carrying a meaningless absolute epoch.
  long long start_ms_ = 0;
};

}  // namespace testing
