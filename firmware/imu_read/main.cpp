// BMI270 live data read -- IMU_SETUP.md step 4.
//
// Uploads the config blob (step 3), enables the sensors, then streams
// real accelerometer and gyroscope numbers.  This is the first sketch
// whose output is a physical measurement rather than a pass/fail.
//
// The blob is re-uploaded here on purpose: it lives in volatile memory
// and does NOT survive the reset that flashing causes.  Every sketch
// that wants data must upload it first -- there is no persistent state
// to inherit from imu_config_upload.
//
// Conversions and register semantics are ported from
// src/drivers/ImuDriver.cpp:260-296, so the numbers here and the ones
// the host driver produces are the same numbers.
//
//   pio run -e imu_read                  (WSL) build, then flash the
//                                        .hex with Teensy Loader
//
// Wiring unchanged (IMU_SETUP.md step 1):
//   BMI270  VDD->3.3V  GND->GND  SCK->13  SDI->11  SDO->12  CSB->10
//
// NOTE: unlike the earlier sketches this one streams from loop(), so a
// late serial attach is harmless -- you only miss the banner.

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

#include "bmi270_config.h"

namespace {

constexpr int kCsPin = 10;

// The blob upload has a hard 2 MHz ceiling; normal data reads do not.
// Upload slow, then raise the clock for streaming.
constexpr uint32_t kSpiHzUpload = 1000000;
constexpr uint32_t kSpiHzData = 10000000;

constexpr uint8_t kRegChipId = 0x00;
constexpr uint8_t kRegAccXLsb = 0x0C;       // 12 bytes: acc XYZ then gyr XYZ
constexpr uint8_t kRegInternalStatus = 0x21;
constexpr uint8_t kRegTemperature = 0x22;
constexpr uint8_t kRegAccConf = 0x40;
constexpr uint8_t kRegAccRange = 0x41;
constexpr uint8_t kRegGyrConf = 0x42;
constexpr uint8_t kRegGyrRange = 0x43;
constexpr uint8_t kRegInitCtrl = 0x59;
constexpr uint8_t kRegInitAddr0 = 0x5B;
constexpr uint8_t kRegInitAddr1 = 0x5C;
constexpr uint8_t kRegInitData = 0x5E;
constexpr uint8_t kRegPwrConf = 0x7C;
constexpr uint8_t kRegPwrCtrl = 0x7D;

constexpr uint8_t kChipIdBmi270 = 0x24;
constexpr uint8_t kInternalStatusMask = 0x0F;
constexpr uint8_t kInternalStatusReady = 0x01;

// Register VALUES, not physical units -- the mapping is not the number.
// ACC_RANGE 0 = +-2 g.  GYR_RANGE is INVERTED: 0 = 2000 dps, so +-500
// dps is register value 2.  Writing 500 here would be a silent 4x scale
// error that still produces plausible-looking output.
constexpr uint8_t kAccelRangeReg = 0;       // +-2 g
constexpr uint8_t kGyroRangeReg = 2;        // +-500 dps
constexpr uint8_t kOdrReg = 0x0A;           // 400 Hz

constexpr double kAccelRangeG = 2.0;        // must match kAccelRangeReg
constexpr double kGyroRangeDps = 500.0;     // must match kGyroRangeReg

constexpr double kGravity = 9.80665;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

// Precomputed once at init, never per sample.  Both sensors are 16-bit
// signed, so full scale maps to 32768 counts.
double accel_scale = 0.0;                   // LSB -> m/s^2
double gyro_scale = 0.0;                    // LSB -> rad/s

constexpr size_t kChunk = 256;              // must be EVEN (half-word addr)

SPISettings settings(kSpiHzUpload, MSBFIRST, SPI_MODE0);

// SPI read: send `reg | 0x80`, discard ONE dummy byte, then read.
// Datasheet lines 11749 / 11815: the first byte on SDO is garbage.
uint8_t readReg(uint8_t reg) {
  SPI.beginTransaction(settings);
  digitalWrite(kCsPin, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(0x00);                       // dummy -- DISCARD
  const uint8_t value = SPI.transfer(0x00);
  digitalWrite(kCsPin, HIGH);
  SPI.endTransaction();
  return value;
}

// Burst read: ONE transaction, CS held low throughout.  This is what
// keeps the six axes coherent -- the address auto-increments internally
// only while CS is asserted, and reading the pair as two transactions
// can straddle an internal update and mix samples from two instants.
void readBurst(uint8_t reg, uint8_t* buffer, size_t len) {
  SPI.beginTransaction(settings);
  digitalWrite(kCsPin, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(0x00);                       // dummy -- DISCARD
  for (size_t i = 0; i < len; i++) {
    buffer[i] = SPI.transfer(0x00);
  }
  digitalWrite(kCsPin, HIGH);
  SPI.endTransaction();
}

void writeReg(uint8_t reg, uint8_t value) {
  SPI.beginTransaction(settings);
  digitalWrite(kCsPin, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(value);
  digitalWrite(kCsPin, HIGH);
  SPI.endTransaction();
}

// CS MUST stay low for the whole chunk: the internal write pointer
// auto-increments only within one CS assertion.
void writeBurst(uint8_t reg, const uint8_t* data, size_t len) {
  SPI.beginTransaction(settings);
  digitalWrite(kCsPin, LOW);
  SPI.transfer(reg & 0x7F);
  for (size_t i = 0; i < len; i++) {
    SPI.transfer(data[i]);
  }
  digitalWrite(kCsPin, HIGH);
  SPI.endTransaction();
}

// Little-endian: LSB first, then MSB.  Getting this backwards survives
// every earlier check and only shows up as nonsense in step 4.
int16_t toInt16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}

bool uploadConfigBlob() {
  const size_t blob_size = sizeof(bmi270_config_file);

  writeReg(kRegPwrConf, 0x00);              // disable adv. power save
  delayMicroseconds(450);
  writeReg(kRegInitCtrl, 0x00);             // announce upload

  for (size_t offset = 0; offset < blob_size; offset += kChunk) {
    const size_t len =
        (offset + kChunk <= blob_size) ? kChunk : (blob_size - offset);

    // INIT_ADDR holds the write pointer in HALF-WORD units, split
    // across nibbles: ADDR_0 = bits [3:0], ADDR_1 = bits [11:4].
    const size_t half_word = offset / 2;
    writeReg(kRegInitAddr0, static_cast<uint8_t>(half_word & 0x0F));
    writeReg(kRegInitAddr1, static_cast<uint8_t>((half_word >> 4) & 0xFF));

    writeBurst(kRegInitData, bmi270_config_file + offset, len);
  }

  writeReg(kRegInitCtrl, 0x01);             // upload complete

  for (int attempt = 0; attempt < 40; attempt++) {
    delay(1);
    const uint8_t status = readReg(kRegInternalStatus);
    if ((status & kInternalStatusMask) == kInternalStatusReady) {
      return true;
    }
  }
  return false;
}

void configureSensors() {
  // PWR_CTRL bits: 0=aux, 1=gyr, 2=acc, 3=temp.  0x0E enables all three
  // we care about.  Until this write, every data register reads zero.
  writeReg(kRegPwrCtrl, 0x0E);
  delay(5);

  // ACC_CONF: [3:0] ODR, [6:4] averaging, [7] filter performance.
  // 0xA0 selects the performance-optimised path -- the low-power path
  // adds several ms of group delay, which a control loop cannot afford.
  writeReg(kRegAccConf, static_cast<uint8_t>(0xA0 | kOdrReg));
  writeReg(kRegAccRange, kAccelRangeReg);

  // GYR_CONF: same layout, plus [6] noise and [7] filter performance.
  writeReg(kRegGyrConf, static_cast<uint8_t>(0xE0 | kOdrReg));
  writeReg(kRegGyrRange, kGyroRangeReg);

  delay(10);

  accel_scale = kAccelRangeG * kGravity / 32768.0;
  gyro_scale = kGyroRangeDps * kDegToRad / 32768.0;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println();
  Serial.println("=== BMI270 live data read ===");

  pinMode(kCsPin, OUTPUT);
  digitalWrite(kCsPin, HIGH);               // idle high before SPI.begin
  SPI.begin();
  delay(10);

  // Powers up in I2C mode, switches to SPI on the first CS falling
  // edge, so this read is the mode-select and returns garbage BY DESIGN.
  readReg(kRegChipId);
  delay(1);

  const uint8_t id = readReg(kRegChipId);
  Serial.print("CHIP_ID = 0x");
  Serial.println(id, HEX);
  if (id != kChipIdBmi270) {
    Serial.println("FAIL -- expected 0x24.  Fix the bus; re-run imu_spi_test.");
    while (true) { delay(1000); }
  }

  Serial.println("uploading config blob (8192 bytes)...");
  if (!uploadConfigBlob()) {
    Serial.println("FAIL -- INTERNAL_STATUS never reached 0x01.");
    Serial.println("  See imu_config_upload for the diagnosis table.");
    while (true) { delay(1000); }
  }
  Serial.println("blob accepted");

  configureSensors();

  // Raise the clock only now.  The upload ceiling does not apply to
  // normal register access.
  settings = SPISettings(kSpiHzData, MSBFIRST, SPI_MODE0);

  Serial.println("config: accel +-2g, gyro +-500dps, ODR 400 Hz");
  Serial.println();
  Serial.println("Rest the board flat and still.  |a| should read 9.81.");
  Serial.println();
  Serial.println("      ax      ay      az   |     gx      gy      gz   |"
                 "    |a|   temp");
}

void loop() {
  // ONE transaction for all 12 bytes.  See readBurst().
  uint8_t buffer[12];
  readBurst(kRegAccXLsb, buffer, sizeof(buffer));

  const double ax = toInt16(&buffer[0]) * accel_scale;
  const double ay = toInt16(&buffer[2]) * accel_scale;
  const double az = toInt16(&buffer[4]) * accel_scale;

  // Raw gyro -- no bias removal.  That is step 5, and subtracting an
  // unmeasured bias here would hide the very offset step 5 measures.
  const double gx = toInt16(&buffer[6]) * gyro_scale;
  const double gy = toInt16(&buffer[8]) * gyro_scale;
  const double gz = toInt16(&buffer[10]) * gyro_scale;

  const double a_mag = sqrt(ax * ax + ay * ay + az * az);

  // Temperature is a separate register pair; 0x8000 is the "invalid"
  // sentinel, which is why a reading of exactly 0 is suspicious.
  uint8_t temp_buffer[2];
  readBurst(kRegTemperature, temp_buffer, sizeof(temp_buffer));
  const int16_t temp_raw = toInt16(temp_buffer);
  const double temp_c =
      (temp_raw == static_cast<int16_t>(0x8000)) ? NAN
                                                 : (temp_raw / 512.0) + 23.0;

  Serial.printf("%8.3f%8.3f%8.3f | %7.3f%7.3f%7.3f | %7.3f%7.1f\n",
                ax, ay, az, gx, gy, gz, a_mag, temp_c);

  delay(100);                               // 10 Hz -- readable by eye
}
