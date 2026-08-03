// Bosch BMI270 IMU over Linux I2C (i2c-dev ioctl).
//
// The BMI270 is not a register-and-go part.  After power-on it holds the
// accelerometer and gyroscope disabled until an ~8 KB configuration blob has
// been uploaded into its internal core; until that upload completes and
// INTERNAL_STATUS reports 0b0001, every data register reads zero.  This trips
// people up constantly, because the chip ID reads back correctly the whole
// time and nothing looks wrong.
//
// That blob is Bosch's binary and is not reproduced here.  See the note on
// kConfigBlobRequired below and docs/3d_scaling/README.md for how to obtain
// it.  If it is absent, initialize() fails loudly rather than handing back
// an IMU that silently returns zeros.
//
// Register addresses and constants below are from the Bosch BMI270 Sensor
// API headers (bmi2_defs.h) and the BMI270 datasheet rev 1.4+.
#pragma once

#include <cstdint>
#include <string>

#include "core/Types.hpp"
#include "core/interfaces/IImuSensor.hpp"

namespace cube {

class ImuDriver : public IImuSensor {
 public:
  // --- BMI270 register map (bmi2_defs.h) --------------------------------
  static constexpr std::uint8_t kRegChipId = 0x00;
  static constexpr std::uint8_t kRegInternalStatus = 0x21;
  static constexpr std::uint8_t kRegAccXLsb = 0x0C;  // 12 bytes: acc then gyr
  static constexpr std::uint8_t kRegTemperature = 0x22;
  static constexpr std::uint8_t kRegAccConf = 0x40;
  static constexpr std::uint8_t kRegAccRange = 0x41;
  static constexpr std::uint8_t kRegGyrConf = 0x42;
  static constexpr std::uint8_t kRegGyrRange = 0x43;
  static constexpr std::uint8_t kRegInitCtrl = 0x59;
  static constexpr std::uint8_t kRegInitAddr0 = 0x5B;
  static constexpr std::uint8_t kRegInitAddr1 = 0x5C;
  static constexpr std::uint8_t kRegInitData = 0x5E;
  static constexpr std::uint8_t kRegPwrConf = 0x7C;
  static constexpr std::uint8_t kRegPwrCtrl = 0x7D;
  static constexpr std::uint8_t kRegCmd = 0x7E;

  static constexpr std::uint8_t kChipIdBmi270 = 0x24;
  static constexpr std::uint8_t kCmdSoftReset = 0xB6;

  // INTERNAL_STATUS.message field; 0b0001 means the config load succeeded.
  static constexpr std::uint8_t kInternalStatusMask = 0x0F;
  static constexpr std::uint8_t kInternalStatusReady = 0x01;

  // Default I2C addresses.  0x68 with SDO low, 0x69 with SDO high.
  static constexpr std::uint8_t kI2cAddrPrimary = 0x68;
  static constexpr std::uint8_t kI2cAddrSecondary = 0x69;

  struct Config {
    // I2C bus device node.  On a Raspberry Pi this is /dev/i2c-1.
    std::string i2c_device = "/dev/i2c-1";
    std::uint8_t i2c_address = kI2cAddrPrimary;

    // Accelerometer range: 0=+/-2g, 1=+/-4g, 2=+/-8g, 3=+/-16g.  A balancing
    // cube never sees more than a couple of g, and the smallest range gives
    // the finest resolution, so 2g is the right default.
    std::uint8_t accel_range = 0;

    // Gyro range: 0=2000, 1=1000, 2=500, 3=250, 4=125 dps.  A cube catching
    // itself can exceed 250 dps briefly, so 500 dps is a safe compromise
    // between headroom and resolution.
    std::uint8_t gyro_range = 2;

    // ODR code for both sensors.  0x0A = 400 Hz, comfortably above the
    // 200 Hz control loop so every cycle gets a fresh sample.
    std::uint8_t odr = 0x0A;

    // Gyro bias, rad/s, subtracted from every sample.  Populated by
    // calibrateGyroBias() or loaded from a previous run.
    double gyro_bias_x = 0.0;
    double gyro_bias_y = 0.0;
    double gyro_bias_z = 0.0;
  };

  ImuDriver();
  explicit ImuDriver(const Config& config);
  ~ImuDriver() override;

  ImuDriver(const ImuDriver&) = delete;
  ImuDriver& operator=(const ImuDriver&) = delete;

  bool initialize(std::string* error) override;
  ImuData read() override;
  bool calibrateGyroBias(Seconds duration, std::string* error) override;
  std::string name() const override { return "BMI270"; }

  const Config& config() const { return config_; }
  Config& mutableConfig() { return config_; }

 private:
  // Raw I2C register access.  Return false on any ioctl/read/write failure.
  bool writeRegister(std::uint8_t reg, std::uint8_t value);
  bool readRegisters(std::uint8_t reg, std::uint8_t* buffer, std::size_t len);

  // Uploads the Bosch configuration blob and waits for INTERNAL_STATUS to
  // report success.  This is the step that makes the sensors produce data.
  bool uploadConfigBlob(std::string* error);

  Config config_;
  int fd_ = -1;

  // Scale factors derived from the configured ranges at init, so the hot
  // read path is a multiply rather than a switch.
  double accel_scale_ = 0.0;  // LSB -> m/s^2
  double gyro_scale_ = 0.0;   // LSB -> rad/s
};

}  // namespace cube
