// BMI270 over Teensy SPI.  See the header for the four SPI-vs-I2C
// differences; this file is where each one is actually enforced.

#include "embedded/Bmi270SpiDriver.hpp"

#include <Arduino.h>
#include <SPI.h>

#include <cmath>

// Bosch's ~8 KB configuration blob.  Unconditional here, unlike the host
// driver's HAVE_BMI270_CONFIG_BLOB guard: platformio.ini puts -I third_party
// in the shared build_flags, and a Teensy build with no blob would be a
// driver that initialises cleanly and then reads zeros forever.
#include "bmi270_config.h"

namespace cube {
namespace {

namespace reg = ::cube::bmi270;

// Chunk size for the blob upload.  MUST BE EVEN -- INIT_ADDR is expressed
// in half-words, so an odd boundary is not addressable.
//
// 256 rather than the host driver's 32: at 8192 bytes that is 32 SPI
// transactions instead of 256.  The upper bound is not the part but CS
// hold time, and 256 is well inside it.
constexpr std::size_t kChunk = 256;

// Sampling interval during bias calibration, ~400 Hz to match the ODR.
constexpr std::uint32_t kSamplePeriodUs = 2500;

// Rejection threshold for "was it actually still?".  A stationary BMI270 at
// 500 dps sits well under 0.01 rad/s of noise, so 0.02 is a motion
// detector, not a noise floor.  Same value as ImuDriver -- the two drivers
// must agree about what counts as still, or a bias measured on one platform
// would be accepted by the other under conditions it rejects.
constexpr double kMaxStationarySd = 0.02;  // rad/s

}  // namespace

Bmi270SpiDriver::Config Bmi270SpiDriver::Config::FromBuildDefaults() {
  Config config;
  // Guarded per field: a build defining only some flags still gets the
  // identity transform for the rest, rather than a half-calibrated sensor.
#ifdef GYRO_BIAS_X
  config.gyro_bias_x = GYRO_BIAS_X;
#endif
#ifdef GYRO_BIAS_Y
  config.gyro_bias_y = GYRO_BIAS_Y;
#endif
#ifdef GYRO_BIAS_Z
  config.gyro_bias_z = GYRO_BIAS_Z;
#endif
#ifdef ACCEL_OFFSET_X
  config.accel_offset_x = ACCEL_OFFSET_X;
#endif
#ifdef ACCEL_OFFSET_Y
  config.accel_offset_y = ACCEL_OFFSET_Y;
#endif
#ifdef ACCEL_OFFSET_Z
  config.accel_offset_z = ACCEL_OFFSET_Z;
#endif
#ifdef ACCEL_SCALE_X
  config.accel_scale_x = ACCEL_SCALE_X;
#endif
#ifdef ACCEL_SCALE_Y
  config.accel_scale_y = ACCEL_SCALE_Y;
#endif
#ifdef ACCEL_SCALE_Z
  config.accel_scale_z = ACCEL_SCALE_Z;
#endif
  return config;
}

Bmi270SpiDriver::Bmi270SpiDriver() = default;

Bmi270SpiDriver::Bmi270SpiDriver(const Config& config) : config_(config) {}

// ---------------------------------------------------------------------- bus

// Difference [1]: read address carries bit 7, and the byte immediately
// after the address is a dummy that MUST be discarded.  Dropping the
// discard shifts every subsequent byte by one, which produces plausible
// nonsense rather than an obvious failure.
std::uint8_t Bmi270SpiDriver::readRegister(std::uint8_t reg) {
  SPI.beginTransaction(SPISettings(spi_hz_, MSBFIRST, SPI_MODE0));
  digitalWrite(config_.cs_pin, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(0x00);  // dummy -- DISCARD
  const std::uint8_t value = SPI.transfer(0x00);
  digitalWrite(config_.cs_pin, HIGH);
  SPI.endTransaction();
  return value;
}

// Writes take (reg & 0x7F) and NO dummy byte -- the asymmetry with reads is
// deliberate and comes straight from the datasheet.
void Bmi270SpiDriver::writeRegister(std::uint8_t reg, std::uint8_t value) {
  SPI.beginTransaction(SPISettings(spi_hz_, MSBFIRST, SPI_MODE0));
  digitalWrite(config_.cs_pin, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(value);
  digitalWrite(config_.cs_pin, HIGH);
  SPI.endTransaction();
}

// Difference [2]: CS stays LOW for the whole payload.  One transaction also
// keeps the six axes coherent -- two reads can straddle an internal update
// and mix samples from different instants.
void Bmi270SpiDriver::readBurst(std::uint8_t reg, std::uint8_t* buffer,
                                std::size_t len) {
  SPI.beginTransaction(SPISettings(spi_hz_, MSBFIRST, SPI_MODE0));
  digitalWrite(config_.cs_pin, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(0x00);  // dummy -- DISCARD
  for (std::size_t i = 0; i < len; i++) { buffer[i] = SPI.transfer(0x00); }
  digitalWrite(config_.cs_pin, HIGH);
  SPI.endTransaction();
}

// The internal write pointer auto-increments only within one CS assertion,
// so raising CS mid-chunk silently corrupts the blob rather than erroring.
void Bmi270SpiDriver::writeBurst(std::uint8_t reg, const std::uint8_t* data,
                                 std::size_t len) {
  SPI.beginTransaction(SPISettings(spi_hz_, MSBFIRST, SPI_MODE0));
  digitalWrite(config_.cs_pin, LOW);
  SPI.transfer(reg & 0x7F);
  for (std::size_t i = 0; i < len; i++) { SPI.transfer(data[i]); }
  digitalWrite(config_.cs_pin, HIGH);
  SPI.endTransaction();
}

// --------------------------------------------------------------------- init

bool Bmi270SpiDriver::uploadConfigBlob(std::string* error) {
  const std::size_t blob_size = sizeof(bmi270_config_file);

  writeRegister(reg::kRegPwrConf, 0x00);  // disable advanced power save
  delayMicroseconds(450);                 // datasheet settle
  writeRegister(reg::kRegInitCtrl, 0x00); // announce the upload

  for (std::size_t offset = 0; offset < blob_size; offset += kChunk) {
    const std::size_t len =
        (offset + kChunk <= blob_size) ? kChunk : (blob_size - offset);

    // INIT_ADDR is in HALF-WORDS, split across two registers as a low
    // nibble and a high byte.  Feeding it a byte offset would place each
    // chunk at twice its intended address.
    const std::size_t half_word = offset / 2;
    writeRegister(reg::kRegInitAddr0,
                  static_cast<std::uint8_t>(half_word & 0x0F));
    writeRegister(reg::kRegInitAddr1,
                  static_cast<std::uint8_t>((half_word >> 4) & 0xFF));
    writeBurst(reg::kRegInitData, bmi270_config_file + offset, len);
  }

  writeRegister(reg::kRegInitCtrl, 0x01);  // upload complete, start the load

  for (int attempt = 0; attempt < reg::kInitStatusPollAttempts; attempt++) {
    delay(1);
    if ((readRegister(reg::kRegInternalStatus) & reg::kInternalStatusMask) ==
        reg::kInternalStatusReady) {
      return true;
    }
  }

  if (error) {
    *error =
        "BMI270 config upload did not complete: INTERNAL_STATUS never "
        "reported ready.\nUsual causes, in order of likelihood:\n"
        "  * SPI clock too fast during upload (must be <= ~2 MHz)\n"
        "  * CS released mid-chunk, corrupting the blob\n"
        "  * wrong bmi270_config_file for this part";
  }
  return false;
}

void Bmi270SpiDriver::configureSensors() {
  writeRegister(reg::kRegPwrCtrl, reg::kPwrCtrlAccGyrTemp);
  delay(5);
  writeRegister(reg::kRegAccConf,
                static_cast<std::uint8_t>(reg::kAccConfHighBits | config_.odr));
  writeRegister(reg::kRegAccRange, config_.accel_range);
  writeRegister(reg::kRegGyrConf,
                static_cast<std::uint8_t>(reg::kGyrConfHighBits | config_.odr));
  writeRegister(reg::kRegGyrRange, config_.gyro_range);
  delay(10);

  // Range indices are clamped rather than trusted: an out-of-range value
  // would read past these tables.
  const int accel_idx = (config_.accel_range < 4) ? config_.accel_range : 0;
  const int gyro_idx = (config_.gyro_range < 5) ? config_.gyro_range : 2;
  accel_scale_ =
      reg::kAccelRangeG[accel_idx] * reg::kGravity / reg::kFullScaleCounts;
  gyro_scale_ =
      reg::kGyroRangeDps[gyro_idx] * reg::kDegToRad / reg::kFullScaleCounts;
}

bool Bmi270SpiDriver::initialize(std::string* error) {
  const auto fail = [error](const char* message) {
    if (error) { *error = message; }
    return false;
  };

  // Validate the accel scales before touching the bus.  read() divides by
  // these, so a zero from a mis-transcribed build flag would produce
  // infinities that propagate into the estimator -- and that failure is
  // silent, since an infinite angle never trips a comparison.
  {
    const double scales[3] = {config_.accel_scale_x, config_.accel_scale_y,
                              config_.accel_scale_z};
    for (int i = 0; i < 3; i++) {
      if (!std::isfinite(scales[i]) || std::fabs(scales[i] - 1.0) > 0.5) {
        return fail(
            "accel_scale_* is not a plausible calibration gain (expected near "
            "1.0).\nRe-run the imu_calibrate sketch, menu [f], and check the "
            "transcribed build flags.");
      }
    }
  }

  spi_hz_ = config_.spi_hz_upload;

  pinMode(config_.cs_pin, OUTPUT);
  digitalWrite(config_.cs_pin, HIGH);
  SPI.begin();
  delay(10);

  // Difference [3]: the part boots in I2C mode and latches to SPI on the
  // first CS falling edge, so this read returns garbage BY DESIGN.  It is
  // the mode select, not a real query -- discard it.
  readRegister(reg::kRegChipId);
  delay(1);

  const std::uint8_t chip_id = readRegister(reg::kRegChipId);
  if (chip_id != reg::kChipIdBmi270) {
    // 0x00 usually means MISO is not connected or the part is unpowered;
    // 0xFF usually means MISO is floating high with no part responding.
    return fail(
        "BMI270 CHIP_ID mismatch (expected 0x24).\n"
        "0x00 -> check MISO/pin 12 and the 3.3V rail.\n"
        "0xFF -> nothing is driving MISO; check CS/pin 10 and wiring.\n"
        "Re-run the imu_spi_test sketch to isolate sensor from driver.");
  }

  if (!uploadConfigBlob(error)) { return false; }

  configureSensors();

  // Only now is it safe to raise the clock -- see difference [4].
  spi_hz_ = config_.spi_hz_data;

  initialized_ = true;
  return true;
}

// --------------------------------------------------------------------- read

ImuData Bmi270SpiDriver::read() {
  ImuData data;
  if (!initialized_) { return data; }

  // One burst for all six axes: 6 bytes of accel then 6 of gyro, contiguous
  // from 0x0C.  ~15 us at 10 MHz.
  std::uint8_t buffer[12] = {};
  readBurst(reg::kRegAccXLsb, buffer, sizeof(buffer));

  // Correction order is fixed and MUST match ImuDriver::read():
  //
  //   raw counts --> x accel_scale_ --> - offset --> / scale --> m/s^2
  //                  (LSB->SI, from    (six-position fit)
  //                   the range)
  //
  // accel_scale_ and config_.accel_scale_* are different things despite the
  // names: the first converts counts to SI, the second corrects this
  // individual part's gain error and sits near 1.0.  Reversing the order
  // would scale the offset, which is in m/s^2 and not in counts.
  data.accel_x =
      (reg::ToInt16(&buffer[0]) * accel_scale_ - config_.accel_offset_x) /
      config_.accel_scale_x;
  data.accel_y =
      (reg::ToInt16(&buffer[2]) * accel_scale_ - config_.accel_offset_y) /
      config_.accel_scale_y;
  data.accel_z =
      (reg::ToInt16(&buffer[4]) * accel_scale_ - config_.accel_offset_z) /
      config_.accel_scale_z;

  data.gyro_x = reg::ToInt16(&buffer[6]) * gyro_scale_ - config_.gyro_bias_x;
  data.gyro_y = reg::ToInt16(&buffer[8]) * gyro_scale_ - config_.gyro_bias_y;
  data.gyro_z = reg::ToInt16(&buffer[10]) * gyro_scale_ - config_.gyro_bias_z;

  std::uint8_t temp_buffer[2] = {};
  readBurst(reg::kRegTemperature, temp_buffer, sizeof(temp_buffer));
  const std::int16_t raw_temp = reg::ToInt16(temp_buffer);
  data.temperature_c = (raw_temp == reg::kTemperatureInvalid)
                           ? 0.0
                           : (raw_temp / reg::kTemperatureLsbPerC) +
                                 reg::kTemperatureOffsetC;

  // 64-bit clock, not micros(): a raw uint32 wraps every 71 minutes and
  // takes the timestamp backwards with it.  See TeensyClock.hpp.
  data.timestamp = clock_.seconds();
  data.valid = true;
  return data;
}

// -------------------------------------------------------------- calibration

bool Bmi270SpiDriver::calibrateGyroBias(Seconds duration, std::string* error) {
  if (!initialized_) {
    if (error) { *error = "IMU not initialized"; }
    return false;
  }

  // Clear the existing bias first, or this measures the residual of the
  // last correction rather than the true zero-rate offset.
  config_.gyro_bias_x = 0.0;
  config_.gyro_bias_y = 0.0;
  config_.gyro_bias_z = 0.0;

  double sum[3] = {0.0, 0.0, 0.0};
  double sum_sq[3] = {0.0, 0.0, 0.0};
  long count = 0;

  const Seconds start = clock_.seconds();
  while (clock_.seconds() - start < duration) {
    const ImuData sample = read();
    if (sample.valid) {
      const double g[3] = {sample.gyro_x, sample.gyro_y, sample.gyro_z};
      for (int i = 0; i < 3; i++) {
        sum[i] += g[i];
        sum_sq[i] += g[i] * g[i];
      }
      count++;
    }
    delayMicroseconds(kSamplePeriodUs);
  }

  if (count < 100) {
    if (error) {
      *error = "too few valid samples during bias calibration -- is the IMU "
               "responding?";
    }
    return false;
  }

  double mean[3];
  double worst_sd = 0.0;
  for (int i = 0; i < 3; i++) {
    mean[i] = sum[i] / count;
    // Guard the subtraction: catastrophic cancellation can push this a hair
    // below zero, and sqrt of a negative is NaN.
    const double variance =
        std::fmax(0.0, sum_sq[i] / count - mean[i] * mean[i]);
    const double sd = std::sqrt(variance);
    if (sd > worst_sd) { worst_sd = sd; }
  }

  // If the rig moved during the measurement, the mean is not a bias -- it
  // is a bias plus whatever rotation happened.  Accepting it would bake a
  // permanent phantom rotation into every future sample.
  if (worst_sd > kMaxStationarySd) {
    if (error) {
      *error = "IMU moved during bias calibration; the mean would be bias "
               "PLUS real rotation.  Retry on a still surface.";
    }
    return false;
  }

  config_.gyro_bias_x = mean[0];
  config_.gyro_bias_y = mean[1];
  config_.gyro_bias_z = mean[2];
  return true;
}

}  // namespace cube
