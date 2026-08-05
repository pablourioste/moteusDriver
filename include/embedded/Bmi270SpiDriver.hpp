// Bosch BMI270 over Teensy 4.1 SPI -- the IImuSensor the control loop uses.
//
// This is the SPI counterpart to drivers/ImuDriver (Linux I2C).  Same part,
// same register map (drivers/Bmi270Registers.hpp), same ImuData out; only
// the four bus primitives differ.  core/ cannot tell them apart, which is
// the entire point of the interface seam.
//
// PROVENANCE: the logic here is not new.  It is the code from
// firmware/imu_read/main.cpp and firmware/imu_calibrate/main.cpp -- both
// verified on this hardware -- gathered into a class the control loop can
// call through.  Those sketches remain in the repo and untouched, so if the
// IMU misbehaves later, re-flashing imu_spi_test isolates sensor from
// driver in one step.  A bug found here should be checked against them.
//
// WIRING (IMU_SETUP.md step 1):
//   BMI270  VDD->3.3V  GND->GND  SCK->13  SDI->11  SDO->12  CSB->10
//
// THE FOUR SPI DIFFERENCES vs I2C, each of which has bitten someone:
//
//   [1] Reads send (reg | 0x80) and then DISCARD ONE DUMMY BYTE before the
//       data.  Writes send (reg & 0x7F) with NO dummy.  That asymmetry is
//       the most common BMI270 SPI bug and has no I2C equivalent -- get it
//       wrong and every value is shifted by one byte, which looks like
//       plausible-but-wrong data rather than an obvious failure.
//
//   [2] Bursts must hold CS LOW across the register byte AND the whole
//       payload.  The internal address pointer auto-increments only within
//       one CS assertion, so a burst built from repeated single writes
//       silently corrupts the config blob.
//
//   [3] The part powers up in I2C mode and latches to SPI on the FIRST CS
//       falling edge.  So the first read returns garbage BY DESIGN and must
//       be thrown away.  ImuDriver's defensive triple-read happens to be
//       exactly right for SPI; it is kept here for that reason.
//
//   [4] The blob upload has a hard clock ceiling around 2 MHz.  Above it,
//       INTERNAL_STATUS never reaches 0x01 and the failure mimics a wiring
//       fault.  Data reads then run happily at 10 MHz, so the clock is
//       raised after initialisation.
#pragma once

#include <cstdint>
#include <string>

#include "core/Types.hpp"
#include "core/interfaces/IImuSensor.hpp"
#include "drivers/Bmi270Registers.hpp"
#include "embedded/TeensyClock.hpp"

namespace cube {

class Bmi270SpiDriver : public IImuSensor {
 public:
  struct Config {
    // Chip select.  Driven manually rather than by the SPI peripheral's
    // hardware CS, because the blob upload needs CS held low across a
    // 256-byte payload -- see difference [2] above.
    int cs_pin = 10;

    // Blob upload clock.  MUST stay at or below ~2 MHz: see difference [4].
    std::uint32_t spi_hz_upload = 1000000;

    // Normal data clock.  10 MHz puts a 12-byte burst at ~15 us, which is
    // 0.3% of a 5 ms control cycle.  The Teensy's LPSPI tops out at 30 MHz
    // and the BMI270 at 10, so the sensor is the binding limit.
    std::uint32_t spi_hz_data = 10000000;

    // Accelerometer range: 0=+/-2g, 1=+/-4g, 2=+/-8g, 3=+/-16g.  A
    // balancing cube never sees more than a couple of g, and the smallest
    // range gives the finest resolution.
    std::uint8_t accel_range = 0;

    // Gyro range index -- see the INVERTED-index warning in
    // Bmi270Registers.hpp.  2 selects 500 dps, enough headroom for a cube
    // catching itself without wasting resolution.
    std::uint8_t gyro_range = 2;

    // ODR code for both sensors.  0x0A = 400 Hz, twice the 200 Hz control
    // loop so every cycle gets a fresh sample.
    std::uint8_t odr = 0x0A;

    // Gyro bias, rad/s, subtracted from every sample.
    double gyro_bias_x = 0.0;
    double gyro_bias_y = 0.0;
    double gyro_bias_z = 0.0;

    // Accel correction, applied as (a_raw - offset) / scale.  From the
    // six-position fit in firmware/imu_calibrate, menu [1]..[6] then [f].
    //
    // Defaults are the identity transform, so an uncalibrated build behaves
    // as though these did not exist.  Scale defaults to 1.0, not 0.0 --
    // read() divides by it.
    //
    // Must match drivers/ImuDriver::Config field for field, and be applied
    // in the SAME ORDER: the host and the Teensy consume the same measured
    // numbers, so a divergence means the two builds disagree about gravity.
    double accel_offset_x = 0.0;
    double accel_offset_y = 0.0;
    double accel_offset_z = 0.0;
    double accel_scale_x = 1.0;
    double accel_scale_y = 1.0;
    double accel_scale_z = 1.0;

    // Config carrying the calibration baked in by platformio.ini
    // build_flags (GYRO_BIAS_*, ACCEL_OFFSET_*, ACCEL_SCALE_*) -- the flags
    // firmware/imu_calibrate prints paste-ready.  Mirrors
    // ImuDriver::Config::FromBuildDefaults() on the host side.
    static Config FromBuildDefaults();
  };

  Bmi270SpiDriver();
  explicit Bmi270SpiDriver(const Config& config);

  Bmi270SpiDriver(const Bmi270SpiDriver&) = delete;
  Bmi270SpiDriver& operator=(const Bmi270SpiDriver&) = delete;

  bool initialize(std::string* error) override;
  ImuData read() override;
  bool calibrateGyroBias(Seconds duration, std::string* error) override;
  std::string name() const override { return "BMI270/SPI"; }

  const Config& config() const { return config_; }
  Config& mutableConfig() { return config_; }

 private:
  // --- bus primitives -- the ONLY part that differs from ImuDriver -------
  std::uint8_t readRegister(std::uint8_t reg);
  void writeRegister(std::uint8_t reg, std::uint8_t value);
  void readBurst(std::uint8_t reg, std::uint8_t* buffer, std::size_t len);
  void writeBurst(std::uint8_t reg, const std::uint8_t* data, std::size_t len);

  // Uploads Bosch's ~8 KB blob and waits for INTERNAL_STATUS.  Until this
  // succeeds the sensors stay disabled and every data register reads zero,
  // while CHIP_ID keeps answering correctly the whole time.
  bool uploadConfigBlob(std::string* error);

  // Enables the sensors and precomputes the LSB->SI scale factors.
  void configureSensors();

  Config config_;
  TeensyClock clock_;

  bool initialized_ = false;

  // Derived at init so the hot read path is a multiply, not a switch.
  double accel_scale_ = 0.0;  // LSB -> m/s^2
  double gyro_scale_ = 0.0;   // LSB -> rad/s

  // Current SPI clock.  Starts at spi_hz_upload and is raised to
  // spi_hz_data once the blob is in -- see difference [4].
  std::uint32_t spi_hz_ = 1000000;
};

}  // namespace cube
