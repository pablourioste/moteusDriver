// BMI270 register map and conversion constants, shared by both IMU drivers.
//
// Two drivers read this part: ImuDriver (Linux I2C, host) and
// Bmi270SpiDriver (Teensy SPI).  The bus differs; the silicon does not.
// Keeping the register numbers in one header means a datasheet correction
// lands in both drivers at once, instead of being fixed on whichever
// platform happened to show the bug.
//
// Deliberately free of <string>, allocation and bus code so the Teensy
// build can include it without dragging in anything hosted.
//
// Sources: Bosch BMI270 Sensor API headers (bmi2_defs.h) and the BMI270
// datasheet rev 1.4+.
#pragma once

#include <cstdint>

namespace cube {
namespace bmi270 {

// --- Register map ---------------------------------------------------------
constexpr std::uint8_t kRegChipId = 0x00;

// 12 contiguous bytes: 6 of accelerometer then 6 of gyroscope.  One burst
// from here gets a coherent sample pair -- two separate reads can straddle
// an internal update and mix samples from different instants.
constexpr std::uint8_t kRegAccXLsb = 0x0C;

constexpr std::uint8_t kRegInternalStatus = 0x21;
constexpr std::uint8_t kRegTemperature = 0x22;
constexpr std::uint8_t kRegAccConf = 0x40;
constexpr std::uint8_t kRegAccRange = 0x41;
constexpr std::uint8_t kRegGyrConf = 0x42;
constexpr std::uint8_t kRegGyrRange = 0x43;
constexpr std::uint8_t kRegInitCtrl = 0x59;
constexpr std::uint8_t kRegInitAddr0 = 0x5B;
constexpr std::uint8_t kRegInitAddr1 = 0x5C;
constexpr std::uint8_t kRegInitData = 0x5E;
constexpr std::uint8_t kRegPwrConf = 0x7C;
constexpr std::uint8_t kRegPwrCtrl = 0x7D;
constexpr std::uint8_t kRegCmd = 0x7E;

// --- Values ---------------------------------------------------------------
constexpr std::uint8_t kChipIdBmi270 = 0x24;
constexpr std::uint8_t kCmdSoftReset = 0xB6;

// INTERNAL_STATUS.message; 0b0001 means the config blob loaded successfully.
// Until it reads this, every data register returns zero while CHIP_ID keeps
// answering correctly -- which is why a blob failure looks like working
// hardware producing no motion.
constexpr std::uint8_t kInternalStatusMask = 0x0F;
constexpr std::uint8_t kInternalStatusReady = 0x01;

// PWR_CTRL: accelerometer | gyroscope | temperature enabled.
constexpr std::uint8_t kPwrCtrlAccGyrTemp = 0x0E;

// High bits OR'd into the ODR nibble when writing ACC_CONF / GYR_CONF.
// 0xA0 sets acc_filter_perf and the default bandwidth; 0xE0 additionally
// sets gyr_noise_perf, which the gyro has and the accelerometer does not.
constexpr std::uint8_t kAccConfHighBits = 0xA0;
constexpr std::uint8_t kGyrConfHighBits = 0xE0;

// I2C addresses.  0x68 with SDO low, 0x69 with SDO high.  Unused on SPI,
// where the part is selected by CS instead.
constexpr std::uint8_t kI2cAddrPrimary = 0x68;
constexpr std::uint8_t kI2cAddrSecondary = 0x69;

// --- Conversion -----------------------------------------------------------
constexpr double kGravity = 9.80665;
constexpr double kDegToRad = 0.017453292519943295;

// Both sensors are 16-bit signed, so full scale maps to 32768 counts.
constexpr double kFullScaleCounts = 32768.0;

// Accelerometer full-scale in g, indexed by the ACC_RANGE register value.
constexpr double kAccelRangeG[4] = {2.0, 4.0, 8.0, 16.0};

// Gyroscope full-scale in deg/s, indexed by the GYR_RANGE register value.
//
// NOTE the index is INVERTED relative to intuition: 0 is the WIDEST range,
// not the narrowest.  Writing 500 into GYR_RANGE expecting 500 dps would
// select index 4 (125 dps) and produce a silent 4x scale error on every
// gyro sample.  Always index this table; never write a dps value directly.
constexpr double kGyroRangeDps[5] = {2000.0, 1000.0, 500.0, 250.0, 125.0};

// Temperature: 0x8000 is the "invalid" sentinel, so a reading of exactly
// 0 deserves suspicion rather than trust.
constexpr std::int16_t kTemperatureInvalid = static_cast<std::int16_t>(0x8000);
constexpr double kTemperatureLsbPerC = 512.0;
constexpr double kTemperatureOffsetC = 23.0;

// --- Blob upload ----------------------------------------------------------
// The configuration blob is 8192 bytes and must be written in EVEN-sized
// chunks: INIT_ADDR is expressed in half-words, so an odd boundary cannot
// be addressed.
constexpr std::size_t kConfigBlobSize = 8192;

// INTERNAL_STATUS poll budget after INIT_CTRL=1.  The load takes ~20 ms in
// practice; 40 attempts at 1 ms is generous without hanging forever.
constexpr int kInitStatusPollAttempts = 40;

// Reassemble a little-endian 16-bit sample.  LSB first, per the datasheet.
inline std::int16_t ToInt16(const std::uint8_t* p) {
  return static_cast<std::int16_t>(static_cast<std::uint16_t>(p[0]) |
                                   (static_cast<std::uint16_t>(p[1]) << 8));
}

}  // namespace bmi270
}  // namespace cube
