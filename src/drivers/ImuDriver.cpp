#include "drivers/ImuDriver.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <vector>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

// The BMI270 configuration blob.
//
// This is Bosch's binary microcode array (~8 KB), distributed with their
// Apache-2.0 licensed BMI270-Sensor-API.  It is NOT reproduced here: it is
// several thousand hex constants, and a transcription error would produce an
// IMU that initialises cleanly and then returns subtly wrong data.
//
// To supply it, extract `bmi270_config_file[]` from
//   https://github.com/boschsensortec/BMI270-Sensor-API/blob/master/bmi270.c
// into third_party/bmi270_config.h as:
//
//   static const uint8_t bmi270_config_file[] = { 0xc8, 0x2e, ... };
//
// then configure with -DBMI270_CONFIG_HEADER=ON.  See
// docs/3d_scaling/README.md for the exact steps.
#if defined(HAVE_BMI270_CONFIG_BLOB)
#include "bmi270_config.h"
#endif

namespace cube {
namespace {

constexpr double kGravity = 9.80665;
constexpr double kDegToRad = 0.017453292519943295;

// Accelerometer full-scale in g, indexed by the range register value.
constexpr double kAccelRangeG[4] = {2.0, 4.0, 8.0, 16.0};

// Gyroscope full-scale in degrees/s, indexed by the range register value.
constexpr double kGyroRangeDps[5] = {2000.0, 1000.0, 500.0, 250.0, 125.0};

void SleepMicroseconds(long micros) {
  struct timespec ts;
  ts.tv_sec = micros / 1000000;
  ts.tv_nsec = (micros % 1000000) * 1000;
  ::nanosleep(&ts, nullptr);
}

double MonotonicSeconds() {
  struct timespec ts = {};
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) +
         static_cast<double>(ts.tv_nsec) / 1e9;
}

// Reassemble a little-endian signed 16-bit value from two bytes.
std::int16_t ToInt16(const std::uint8_t* p) {
  return static_cast<std::int16_t>(
      static_cast<std::uint16_t>(p[0]) |
      (static_cast<std::uint16_t>(p[1]) << 8));
}

}  // namespace

// Each field is guarded independently: a build that defines only some of
// the macros still gets the identity transform for the rest, rather than
// failing to compile or silently zeroing a calibrated axis.
ImuDriver::Config ImuDriver::Config::FromBuildDefaults() {
  Config config;
#ifdef DEFAULT_IMU_GYRO_BIAS_X
  config.gyro_bias_x = DEFAULT_IMU_GYRO_BIAS_X;
#endif
#ifdef DEFAULT_IMU_GYRO_BIAS_Y
  config.gyro_bias_y = DEFAULT_IMU_GYRO_BIAS_Y;
#endif
#ifdef DEFAULT_IMU_GYRO_BIAS_Z
  config.gyro_bias_z = DEFAULT_IMU_GYRO_BIAS_Z;
#endif
#ifdef DEFAULT_IMU_ACCEL_OFFSET_X
  config.accel_offset_x = DEFAULT_IMU_ACCEL_OFFSET_X;
#endif
#ifdef DEFAULT_IMU_ACCEL_OFFSET_Y
  config.accel_offset_y = DEFAULT_IMU_ACCEL_OFFSET_Y;
#endif
#ifdef DEFAULT_IMU_ACCEL_OFFSET_Z
  config.accel_offset_z = DEFAULT_IMU_ACCEL_OFFSET_Z;
#endif
#ifdef DEFAULT_IMU_ACCEL_SCALE_X
  config.accel_scale_x = DEFAULT_IMU_ACCEL_SCALE_X;
#endif
#ifdef DEFAULT_IMU_ACCEL_SCALE_Y
  config.accel_scale_y = DEFAULT_IMU_ACCEL_SCALE_Y;
#endif
#ifdef DEFAULT_IMU_ACCEL_SCALE_Z
  config.accel_scale_z = DEFAULT_IMU_ACCEL_SCALE_Z;
#endif
  return config;
}

ImuDriver::ImuDriver() = default;

ImuDriver::ImuDriver(const Config& config) : config_(config) {}

ImuDriver::~ImuDriver() {
  if (fd_ >= 0) { ::close(fd_); }
}

bool ImuDriver::writeRegister(std::uint8_t reg, std::uint8_t value) {
  const std::uint8_t buffer[2] = {reg, value};
  return ::write(fd_, buffer, 2) == 2;
}

bool ImuDriver::readRegisters(std::uint8_t reg, std::uint8_t* buffer,
                              std::size_t len) {
  // Write the register address, then read.  Two transfers rather than a
  // combined ioctl message: simpler, and the BMI270 handles a repeated start
  // or a stop-start identically at these rates.
  if (::write(fd_, &reg, 1) != 1) { return false; }
  return ::read(fd_, buffer, len) == static_cast<ssize_t>(len);
}

bool ImuDriver::uploadConfigBlob(std::string* error) {
#if !defined(HAVE_BMI270_CONFIG_BLOB)
  if (error) {
    *error =
        "BMI270 configuration blob not compiled in.  The accelerometer and "
        "gyroscope stay disabled until Bosch's ~8 KB config file is uploaded, "
        "so this driver cannot produce data without it.\n"
        "Extract bmi270_config_file[] from the BMI270-Sensor-API repo into "
        "third_party/bmi270_config.h and reconfigure with "
        "-DBMI270_CONFIG_HEADER=ON.  See docs/3d_scaling/README.md.";
  }
  return false;
#else
  // Disable advanced power save; the config upload requires the device
  // awake.  The datasheet asks for 450 us of settling afterwards.
  if (!writeRegister(kRegPwrConf, 0x00)) {
    if (error) { *error = "failed to write PWR_CONF"; }
    return false;
  }
  SleepMicroseconds(450);

  // Prepare for the upload.
  if (!writeRegister(kRegInitCtrl, 0x00)) {
    if (error) { *error = "failed to write INIT_CTRL=0"; }
    return false;
  }

  // Burst-write the blob.  The internal write pointer is set through
  // INIT_ADDR_0/1, which hold the address in half-word units split across
  // nibbles: ADDR_0 is bits [3:0], ADDR_1 is bits [11:4].
  const std::size_t blob_size = sizeof(bmi270_config_file);
  constexpr std::size_t kChunk = 32;  // bytes per burst; must be even

  std::vector<std::uint8_t> packet;
  packet.reserve(kChunk + 1);

  for (std::size_t offset = 0; offset < blob_size; offset += kChunk) {
    const std::size_t len =
        (offset + kChunk <= blob_size) ? kChunk : (blob_size - offset);

    const std::size_t half_word = offset / 2;
    const std::uint8_t addr_0 = static_cast<std::uint8_t>(half_word & 0x0F);
    const std::uint8_t addr_1 = static_cast<std::uint8_t>((half_word >> 4) & 0xFF);

    if (!writeRegister(kRegInitAddr0, addr_0) ||
        !writeRegister(kRegInitAddr1, addr_1)) {
      if (error) { *error = "failed to set INIT_ADDR during config upload"; }
      return false;
    }

    packet.clear();
    packet.push_back(kRegInitData);
    packet.insert(packet.end(), bmi270_config_file + offset,
                  bmi270_config_file + offset + len);

    if (::write(fd_, packet.data(), packet.size()) !=
        static_cast<ssize_t>(packet.size())) {
      if (error) { *error = "failed to write INIT_DATA during config upload"; }
      return false;
    }
  }

  // Signal that the upload is complete.
  if (!writeRegister(kRegInitCtrl, 0x01)) {
    if (error) { *error = "failed to write INIT_CTRL=1"; }
    return false;
  }

  // The datasheet allows up to ~20 ms for the core to digest the config.
  // Poll rather than sleeping blindly so a fast part is not penalised.
  for (int attempt = 0; attempt < 40; attempt++) {
    SleepMicroseconds(1000);
    std::uint8_t status = 0;
    if (readRegisters(kRegInternalStatus, &status, 1) &&
        (status & kInternalStatusMask) == kInternalStatusReady) {
      return true;
    }
  }

  if (error) {
    *error =
        "BMI270 config upload did not complete: INTERNAL_STATUS never "
        "reported ready.  Check I2C wiring and that the blob is the correct "
        "bmi270_config_file for this part.";
  }
  return false;
#endif
}

bool ImuDriver::initialize(std::string* error) {
  const auto fail = [this, error](const std::string& message) {
    if (error) { *error = message; }
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    return false;
  };

  // Validate the accel scales before touching the bus.  read() divides by
  // these, so a zero left in by a mis-transcribed build flag would produce
  // infinities that propagate into the estimator -- and unlike the NaN
  // sentinels elsewhere, that failure is silent: an infinite accel angle
  // just makes theta useless without ever tripping a comparison.  A scale
  // far from 1.0 means the fit was taken at the wrong range and is a
  // transcription error, not a real part-to-part variation.
  {
    const double scales[3] = {config_.accel_scale_x, config_.accel_scale_y,
                              config_.accel_scale_z};
    const char* axis_names[3] = {"x", "y", "z"};
    for (int i = 0; i < 3; i++) {
      if (!std::isfinite(scales[i]) || std::fabs(scales[i] - 1.0) > 0.5) {
        return fail("accel_scale_" + std::string(axis_names[i]) + " is " +
                    std::to_string(scales[i]) +
                    ", which is not a plausible calibration gain (expected "
                    "near 1.0).\nRe-run firmware/imu/imu_calibrate menu [f] and "
                    "check the transcribed build flags.");
      }
    }
  }

  fd_ = ::open(config_.i2c_device.c_str(), O_RDWR);
  if (fd_ < 0) {
    return fail("cannot open " + config_.i2c_device + ": " +
                std::strerror(errno) +
                "\nOn WSL2 there is no I2C bus -- this driver must run on the "
                "target SBC.");
  }

  if (::ioctl(fd_, I2C_SLAVE, config_.i2c_address) < 0) {
    return fail("cannot select I2C address: " + std::string(std::strerror(errno)));
  }

  // A dummy read is the documented way to put the BMI270 into I2C mode after
  // power-on; the part comes up expecting either bus and latches on first
  // access.
  std::uint8_t chip_id = 0;
  readRegisters(kRegChipId, &chip_id, 1);
  SleepMicroseconds(1000);

  // Soft reset to a known state, then the datasheet's 2 ms settle.
  if (!writeRegister(kRegCmd, kCmdSoftReset)) {
    return fail("no response writing CMD soft reset -- check wiring/address");
  }
  SleepMicroseconds(2000);

  readRegisters(kRegChipId, &chip_id, 1);
  SleepMicroseconds(1000);

  if (!readRegisters(kRegChipId, &chip_id, 1)) {
    return fail("cannot read CHIP_ID");
  }
  if (chip_id != kChipIdBmi270) {
    char buffer[128];
    std::snprintf(buffer, sizeof(buffer),
                  "unexpected CHIP_ID 0x%02X (expected 0x%02X for BMI270)",
                  chip_id, kChipIdBmi270);
    return fail(buffer);
  }

  if (!uploadConfigBlob(error)) {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    return false;
  }

  // Enable accelerometer, gyroscope and temperature sensor.
  // PWR_CTRL bits: 0=aux, 1=gyr, 2=acc, 3=temp.
  if (!writeRegister(kRegPwrCtrl, 0x0E)) {
    return fail("failed to write PWR_CTRL");
  }
  SleepMicroseconds(5000);

  // ACC_CONF: [3:0] ODR, [6:4] bandwidth/averaging, [7] filter performance.
  // 0xA0 | odr selects normal-mode filtering with the performance-optimised
  // path, which is what a control loop wants -- the low-power path adds
  // several milliseconds of group delay.
  if (!writeRegister(kRegAccConf, static_cast<std::uint8_t>(0xA0 | config_.odr))) {
    return fail("failed to write ACC_CONF");
  }
  if (!writeRegister(kRegAccRange, config_.accel_range)) {
    return fail("failed to write ACC_RANGE");
  }

  // GYR_CONF: same layout, plus [6] noise performance and [7] filter
  // performance, both set high for the lowest latency and noise.
  if (!writeRegister(kRegGyrConf, static_cast<std::uint8_t>(0xE0 | config_.odr))) {
    return fail("failed to write GYR_CONF");
  }
  if (!writeRegister(kRegGyrRange, config_.gyro_range)) {
    return fail("failed to write GYR_RANGE");
  }

  SleepMicroseconds(10000);

  // Precompute LSB->SI scale factors.  Both sensors are 16-bit signed, so
  // full scale maps to 32768 counts.
  const int accel_idx = (config_.accel_range < 4) ? config_.accel_range : 0;
  const int gyro_idx = (config_.gyro_range < 5) ? config_.gyro_range : 2;
  accel_scale_ = kAccelRangeG[accel_idx] * kGravity / 32768.0;
  gyro_scale_ = kGyroRangeDps[gyro_idx] * kDegToRad / 32768.0;

  return true;
}

ImuData ImuDriver::read() {
  ImuData data;
  if (fd_ < 0) { return data; }

  // Accelerometer and gyroscope data are contiguous from 0x0C: 6 bytes of
  // accel then 6 of gyro, so one burst gets a coherent sample pair.  Reading
  // them separately risks straddling an internal update.
  std::uint8_t buffer[12] = {};
  if (!readRegisters(kRegAccXLsb, buffer, sizeof(buffer))) {
    return data;
  }

  // Two different corrections apply here, in a fixed order:
  //
  //   raw counts --> x accel_scale_ --> - offset --> / scale --> m/s^2
  //                  (LSB->SI, from    (six-position fit, from
  //                   the range)        imu_calibrate [f])
  //
  // accel_scale_ and config_.accel_scale_* are NOT the same thing despite
  // the similar names: the first converts counts to SI using the configured
  // range, the second corrects this individual part's gain error and is
  // near 1.0.  Applying them in the other order would scale the offset too,
  // and the offset is in m/s^2, not counts.
  //
  // Bmi270SpiDriver::read() must do this identically -- the two drivers
  // consume the same calibration numbers, so a divergence here would mean
  // the host simulation and the Teensy disagreed about gravity.
  data.accel_x = (ToInt16(&buffer[0]) * accel_scale_ - config_.accel_offset_x) /
                 config_.accel_scale_x;
  data.accel_y = (ToInt16(&buffer[2]) * accel_scale_ - config_.accel_offset_y) /
                 config_.accel_scale_y;
  data.accel_z = (ToInt16(&buffer[4]) * accel_scale_ - config_.accel_offset_z) /
                 config_.accel_scale_z;

  data.gyro_x = ToInt16(&buffer[6]) * gyro_scale_ - config_.gyro_bias_x;
  data.gyro_y = ToInt16(&buffer[8]) * gyro_scale_ - config_.gyro_bias_y;
  data.gyro_z = ToInt16(&buffer[10]) * gyro_scale_ - config_.gyro_bias_z;

  // Temperature is a separate 16-bit register pair; 0x8000 means "invalid".
  std::uint8_t temp_buffer[2] = {};
  if (readRegisters(kRegTemperature, temp_buffer, 2)) {
    const std::int16_t raw = ToInt16(temp_buffer);
    data.temperature_c = (raw == static_cast<std::int16_t>(0x8000))
        ? 0.0
        : (raw / 512.0) + 23.0;
  }

  data.timestamp = MonotonicSeconds();
  data.valid = true;
  return data;
}

bool ImuDriver::calibrateGyroBias(Seconds duration, std::string* error) {
  if (fd_ < 0) {
    if (error) { *error = "IMU not initialized"; }
    return false;
  }

  // Clear the existing bias first, otherwise this measures the residual
  // after the previous correction rather than the true zero-rate offset.
  config_.gyro_bias_x = 0.0;
  config_.gyro_bias_y = 0.0;
  config_.gyro_bias_z = 0.0;

  double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
  double sum_sq_x = 0.0, sum_sq_y = 0.0, sum_sq_z = 0.0;
  long count = 0;

  const double start = MonotonicSeconds();
  while (MonotonicSeconds() - start < duration) {
    const ImuData sample = read();
    if (sample.valid) {
      sum_x += sample.gyro_x;
      sum_y += sample.gyro_y;
      sum_z += sample.gyro_z;
      sum_sq_x += sample.gyro_x * sample.gyro_x;
      sum_sq_y += sample.gyro_y * sample.gyro_y;
      sum_sq_z += sample.gyro_z * sample.gyro_z;
      count++;
    }
    SleepMicroseconds(2500);  // ~400 Hz, matching the configured ODR
  }

  if (count < 100) {
    if (error) {
      *error = "too few valid samples (" + std::to_string(count) +
               ") during bias calibration -- is the IMU responding?";
    }
    return false;
  }

  const double mean_x = sum_x / count;
  const double mean_y = sum_y / count;
  const double mean_z = sum_z / count;

  // If the rig moved during the measurement the mean is not a bias, it is a
  // bias plus whatever rotation happened.  Standard deviation catches that:
  // a stationary BMI270 at 500 dps sits well under 0.01 rad/s of noise.
  const double var_x = sum_sq_x / count - mean_x * mean_x;
  const double var_y = sum_sq_y / count - mean_y * mean_y;
  const double var_z = sum_sq_z / count - mean_z * mean_z;
  const double worst_sd =
      std::sqrt(std::fmax(var_x, std::fmax(var_y, var_z)));

  constexpr double kMaxStationarySd = 0.02;  // rad/s
  if (worst_sd > kMaxStationarySd) {
    if (error) {
      *error = "gyro moved during calibration (sd " +
               std::to_string(worst_sd) + " rad/s > " +
               std::to_string(kMaxStationarySd) +
               ") -- hold the rig still and retry";
    }
    return false;
  }

  config_.gyro_bias_x = mean_x;
  config_.gyro_bias_y = mean_y;
  config_.gyro_bias_z = mean_z;
  return true;
}

}  // namespace cube
