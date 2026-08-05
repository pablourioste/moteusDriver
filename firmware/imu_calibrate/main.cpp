// BMI270 calibration -- IMU_SETUP.md steps 4b, 5 and 6.
//
// Interactive, menu-driven.  Produces three things the rest of the
// project needs and cannot derive from code:
//
//   [1] axis mapping     which axis is +9.81 in which orientation
//   [2] gyro bias        zero-rate offset, per axis
//   [3] accel offset+scale   six-position fit
//
// Output is a paste-ready block of build flags.  Nothing is persisted
// on the Teensy -- there is no filesystem, and the config blob does not
// survive a reset anyway, so this sketch re-uploads it at boot like
// imu_read does.
//
// Type a menu key in the serial monitor and press Enter.
//
//   pio run -e imu_calibrate             (WSL) build, then flash with
//                                        Teensy Loader
//
// Wiring unchanged (IMU_SETUP.md step 1):
//   BMI270  VDD->3.3V  GND->GND  SCK->13  SDI->11  SDO->12  CSB->10

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

#include "bmi270_config.h"

namespace {

constexpr int kCsPin = 10;
constexpr uint32_t kSpiHzUpload = 1000000;   // hard ceiling for the blob
constexpr uint32_t kSpiHzData = 10000000;

constexpr uint8_t kRegChipId = 0x00;
constexpr uint8_t kRegAccXLsb = 0x0C;
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

// GYR_RANGE is an INVERTED index: 0=2000, 1=1000, 2=500 dps.  Writing
// 500 here would be a silent 4x scale error.
constexpr uint8_t kAccelRangeReg = 0;        // +-2 g
constexpr uint8_t kGyroRangeReg = 2;         // +-500 dps
constexpr uint8_t kOdrReg = 0x0A;            // 400 Hz

constexpr double kAccelRangeG = 2.0;
constexpr double kGyroRangeDps = 500.0;
constexpr double kGravity = 9.80665;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

constexpr uint32_t kSamplePeriodUs = 2500;   // 400 Hz, matching the ODR

// Rejection threshold for "was it actually still?".  A stationary
// BMI270 at 500 dps sits well under 0.01 rad/s of noise, so 0.02 is a
// motion detector, not a noise floor.  From ImuDriver.cpp:356.
constexpr double kMaxStationarySd = 0.02;    // rad/s

double accel_scale = 0.0;
double gyro_scale = 0.0;

SPISettings settings(kSpiHzUpload, MSBFIRST, SPI_MODE0);

// Six-position accel results, indexed by orientation.  NAN = not yet
// captured, so a partial run cannot silently produce a bad fit.
double axis_max[3] = {NAN, NAN, NAN};
double axis_min[3] = {NAN, NAN, NAN};

double gyro_bias[3] = {0.0, 0.0, 0.0};
bool gyro_bias_valid = false;

// ---------------------------------------------------------------- bus

uint8_t readReg(uint8_t reg) {
  SPI.beginTransaction(settings);
  digitalWrite(kCsPin, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(0x00);                        // dummy -- DISCARD
  const uint8_t value = SPI.transfer(0x00);
  digitalWrite(kCsPin, HIGH);
  SPI.endTransaction();
  return value;
}

// ONE transaction: keeps the six axes coherent.  Two reads can straddle
// an internal update and mix samples from different instants.
void readBurst(uint8_t reg, uint8_t* buffer, size_t len) {
  SPI.beginTransaction(settings);
  digitalWrite(kCsPin, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(0x00);                        // dummy -- DISCARD
  for (size_t i = 0; i < len; i++) { buffer[i] = SPI.transfer(0x00); }
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

// CS held LOW across register byte AND payload -- the internal write
// pointer auto-increments only within one CS assertion.
void writeBurst(uint8_t reg, const uint8_t* data, size_t len) {
  SPI.beginTransaction(settings);
  digitalWrite(kCsPin, LOW);
  SPI.transfer(reg & 0x7F);
  for (size_t i = 0; i < len; i++) { SPI.transfer(data[i]); }
  digitalWrite(kCsPin, HIGH);
  SPI.endTransaction();
}

int16_t toInt16(const uint8_t* p) {          // little-endian: LSB first
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}

// --------------------------------------------------------------- init

bool uploadConfigBlob() {
  const size_t blob_size = sizeof(bmi270_config_file);
  constexpr size_t kChunk = 256;             // must be EVEN

  writeReg(kRegPwrConf, 0x00);
  delayMicroseconds(450);
  writeReg(kRegInitCtrl, 0x00);

  for (size_t offset = 0; offset < blob_size; offset += kChunk) {
    const size_t len =
        (offset + kChunk <= blob_size) ? kChunk : (blob_size - offset);
    const size_t half_word = offset / 2;     // INIT_ADDR is in half-words
    writeReg(kRegInitAddr0, static_cast<uint8_t>(half_word & 0x0F));
    writeReg(kRegInitAddr1, static_cast<uint8_t>((half_word >> 4) & 0xFF));
    writeBurst(kRegInitData, bmi270_config_file + offset, len);
  }

  writeReg(kRegInitCtrl, 0x01);

  for (int attempt = 0; attempt < 40; attempt++) {
    delay(1);
    if ((readReg(kRegInternalStatus) & kInternalStatusMask) ==
        kInternalStatusReady) {
      return true;
    }
  }
  return false;
}

void configureSensors() {
  writeReg(kRegPwrCtrl, 0x0E);               // acc | gyr | temp
  delay(5);
  writeReg(kRegAccConf, static_cast<uint8_t>(0xA0 | kOdrReg));
  writeReg(kRegAccRange, kAccelRangeReg);
  writeReg(kRegGyrConf, static_cast<uint8_t>(0xE0 | kOdrReg));
  writeReg(kRegGyrRange, kGyroRangeReg);
  delay(10);

  accel_scale = kAccelRangeG * kGravity / 32768.0;
  gyro_scale = kGyroRangeDps * kDegToRad / 32768.0;
}

// ------------------------------------------------------------ sampling

struct Sample {
  double a[3];
  double g[3];
};

Sample readSample() {
  uint8_t buffer[12];
  readBurst(kRegAccXLsb, buffer, sizeof(buffer));
  Sample s;
  for (int i = 0; i < 3; i++) {
    s.a[i] = toInt16(&buffer[i * 2]) * accel_scale;
    s.g[i] = toInt16(&buffer[6 + i * 2]) * gyro_scale;
  }
  return s;
}

double readTemperature() {
  uint8_t buffer[2];
  readBurst(kRegTemperature, buffer, sizeof(buffer));
  const int16_t raw = toInt16(buffer);
  // 0x8000 is the "invalid" sentinel -- which is why a reading of
  // exactly 0 deserves suspicion rather than trust.
  return (raw == static_cast<int16_t>(0x8000)) ? NAN : (raw / 512.0) + 23.0;
}

// Mean and standard deviation over `seconds`, for all six channels.
// The sd is the motion detector: it is what distinguishes a bias from a
// bias plus whatever rotation happened while you were holding it.
struct Stats {
  double mean_a[3];
  double mean_g[3];
  double sd_a[3];
  double sd_g[3];
  long count;
};

Stats collect(double seconds) {
  double sum_a[3] = {0, 0, 0}, sum_g[3] = {0, 0, 0};
  double sq_a[3] = {0, 0, 0}, sq_g[3] = {0, 0, 0};
  long count = 0;

  const uint32_t start = millis();
  const uint32_t duration_ms = static_cast<uint32_t>(seconds * 1000.0);
  while (millis() - start < duration_ms) {
    const Sample s = readSample();
    for (int i = 0; i < 3; i++) {
      sum_a[i] += s.a[i];  sq_a[i] += s.a[i] * s.a[i];
      sum_g[i] += s.g[i];  sq_g[i] += s.g[i] * s.g[i];
    }
    count++;
    delayMicroseconds(kSamplePeriodUs);
  }

  Stats st;
  st.count = count;
  for (int i = 0; i < 3; i++) {
    st.mean_a[i] = sum_a[i] / count;
    st.mean_g[i] = sum_g[i] / count;
    // Guard the subtraction: catastrophic cancellation can push this a
    // hair below zero, and sqrt of a negative is NaN.
    const double var_a = fmax(0.0, sq_a[i] / count - st.mean_a[i] * st.mean_a[i]);
    const double var_g = fmax(0.0, sq_g[i] / count - st.mean_g[i] * st.mean_g[i]);
    st.sd_a[i] = sqrt(var_a);
    st.sd_g[i] = sqrt(var_g);
  }
  return st;
}

void countdown(const char* what, int seconds) {
  Serial.print("  ");
  Serial.print(what);
  Serial.print(" -- hold still: ");
  for (int i = seconds; i > 0; i--) {
    Serial.print(i);
    Serial.print(" ");
    delay(1000);
  }
  Serial.println("measuring...");
}

// ------------------------------------------------------------ actions

void showLive() {
  Serial.println();
  Serial.println("Live stream -- send any key to stop.");
  Serial.println("      ax      ay      az   |     gx      gy      gz   |    |a|");
  while (!Serial.available()) {
    const Sample s = readSample();
    const double mag =
        sqrt(s.a[0] * s.a[0] + s.a[1] * s.a[1] + s.a[2] * s.a[2]);
    Serial.printf("%8.3f%8.3f%8.3f | %7.3f%7.3f%7.3f | %7.3f\n",
                  s.a[0], s.a[1], s.a[2], s.g[0], s.g[1], s.g[2], mag);
    delay(100);
  }
  while (Serial.available()) { Serial.read(); }
}

// Step 4b -- axis mapping.  Reports which axis gravity is on and with
// what sign, so you can write the orientation down.
void showAxisMapping() {
  Serial.println();
  Serial.println("--- Axis mapping ---");
  countdown("current orientation", 3);
  const Stats st = collect(1.0);

  int dominant = 0;
  for (int i = 1; i < 3; i++) {
    if (fabs(st.mean_a[i]) > fabs(st.mean_a[dominant])) { dominant = i; }
  }
  const double mag = sqrt(st.mean_a[0] * st.mean_a[0] +
                          st.mean_a[1] * st.mean_a[1] +
                          st.mean_a[2] * st.mean_a[2]);

  Serial.printf("  accel = %.3f %.3f %.3f   |a| = %.3f\n",
                st.mean_a[0], st.mean_a[1], st.mean_a[2], mag);
  Serial.printf("  gravity is on %c%c   (%.3f m/s^2)\n",
                st.mean_a[dominant] > 0 ? '+' : '-', 'X' + dominant,
                st.mean_a[dominant]);

  if (fabs(mag - kGravity) > 0.3) {
    Serial.printf("  WARNING |a| is %.3f, expected 9.81 +-0.3\n", mag);
    Serial.println("  ~19.6 or ~4.9 means the wrong range in the scale factor.");
  }
  Serial.println("  >> Write this down: orientation -> axis and sign.");
}

// Step 5 -- gyro bias.
void calibrateGyro() {
  Serial.println();
  Serial.println("--- Gyro bias (step 5) ---");
  Serial.println("  Solid, level, still surface.  Not your hand.");

  // Clear any previous bias first, or this measures the residual of the
  // last correction rather than the true zero-rate offset.
  gyro_bias[0] = gyro_bias[1] = gyro_bias[2] = 0.0;
  gyro_bias_valid = false;

  countdown("gyro", 3);
  const Stats st = collect(2.0);

  if (st.count < 100) {
    Serial.printf("  FAIL -- only %ld samples; is the IMU responding?\n",
                  st.count);
    return;
  }

  const double worst_sd = fmax(st.sd_g[0], fmax(st.sd_g[1], st.sd_g[2]));
  Serial.printf("  samples=%ld  worst sd=%.5f rad/s\n", st.count, worst_sd);

  if (worst_sd > kMaxStationarySd) {
    Serial.printf("  FAIL -- moved during calibration (sd %.5f > %.5f)\n",
                  worst_sd, kMaxStationarySd);
    Serial.println("  The mean would be bias PLUS real rotation.  Retry still.");
    return;
  }

  for (int i = 0; i < 3; i++) { gyro_bias[i] = st.mean_g[i]; }
  gyro_bias_valid = true;

  Serial.printf("  bias = %+.6f %+.6f %+.6f rad/s\n",
                gyro_bias[0], gyro_bias[1], gyro_bias[2]);
  Serial.printf("  temperature = %.1f C  <-- record this; bias moves with it\n",
                readTemperature());
  Serial.println("  PASS");
}

// Step 6 -- one of the six accelerometer positions.
void captureAccelPosition(int axis, bool positive) {
  Serial.println();
  Serial.printf("--- Accel position: %c%c up ---\n",
                positive ? '+' : '-', 'X' + axis);
  countdown("position", 3);
  const Stats st = collect(2.0);

  // Reject motion here too -- a six-position fit built from samples
  // taken while the board was moving is worse than no correction.
  const double worst_sd = fmax(st.sd_a[0], fmax(st.sd_a[1], st.sd_a[2]));
  if (worst_sd > 0.5) {
    Serial.printf("  FAIL -- moved (accel sd %.3f m/s^2).  Retry.\n", worst_sd);
    return;
  }

  const double value = st.mean_a[axis];
  Serial.printf("  %c axis reads %+.4f m/s^2 (sd %.4f)\n",
                'X' + axis, value, st.sd_a[axis]);

  if (positive) {
    if (value < 5.0) {
      Serial.println("  FAIL -- expected about +9.81.  Wrong orientation?");
      return;
    }
    axis_max[axis] = value;
  } else {
    if (value > -5.0) {
      Serial.println("  FAIL -- expected about -9.81.  Wrong orientation?");
      return;
    }
    axis_min[axis] = value;
  }
  Serial.println("  captured");
}

void showAccelFit() {
  Serial.println();
  Serial.println("--- Accel offset/scale fit ---");

  bool complete = true;
  for (int i = 0; i < 3; i++) {
    if (isnan(axis_max[i]) || isnan(axis_min[i])) {
      Serial.printf("  %c: MISSING (%s%s)\n", 'X' + i,
                    isnan(axis_max[i]) ? "+ " : "",
                    isnan(axis_min[i]) ? "-" : "");
      complete = false;
    }
  }
  if (!complete) {
    Serial.println("  Capture all six positions first.");
    return;
  }

  Serial.println();
  Serial.println("  ; paste into platformio.ini build_flags");
  for (int i = 0; i < 3; i++) {
    const double offset = (axis_max[i] + axis_min[i]) / 2.0;
    const double scale = (axis_max[i] - axis_min[i]) / (2.0 * kGravity);
    Serial.printf("    -D ACCEL_OFFSET_%c=%.6f\n", 'X' + i, offset);
    Serial.printf("    -D ACCEL_SCALE_%c=%.6f\n", 'X' + i, scale);
    if (fabs(scale - 1.0) > 0.05) {
      Serial.printf("  ; WARNING %c scale %.4f is far from 1.0\n",
                    'X' + i, scale);
    }
  }
  Serial.println("  ; apply as: a_corrected = (a_raw - offset) / scale");
}

void showSummary() {
  Serial.println();
  Serial.println("=== Calibration summary ===");
  Serial.println("; paste into platformio.ini build_flags");

  if (gyro_bias_valid) {
    Serial.printf("    -D GYRO_BIAS_X=%.6f\n", gyro_bias[0]);
    Serial.printf("    -D GYRO_BIAS_Y=%.6f\n", gyro_bias[1]);
    Serial.printf("    -D GYRO_BIAS_Z=%.6f\n", gyro_bias[2]);
  } else {
    Serial.println("  ; gyro NOT calibrated -- run [2]");
  }

  bool accel_complete = true;
  for (int i = 0; i < 3; i++) {
    if (isnan(axis_max[i]) || isnan(axis_min[i])) { accel_complete = false; }
  }
  if (accel_complete) {
    showAccelFit();
  } else {
    Serial.println("  ; accel NOT calibrated -- run [4]..[9]");
  }

  Serial.println();
  Serial.println("NOTE: mounting misalignment is NOT covered here.  If the");
  Serial.println("IMU sits a fraction of a degree off the body frame, that");
  Serial.println("is a ROTATION, not an offset -- measure it at assembly and");
  Serial.println("apply it after this correction.  Folding it into these");
  Serial.println("offsets corrupts both.");
}

void printMenu() {
  Serial.println();
  Serial.println("=== BMI270 calibration ===");
  Serial.println("  l  live stream");
  Serial.println("  m  axis mapping        (step 4b)");
  Serial.println("  g  gyro bias           (step 5)");
  Serial.println("  v  verify gyro at rest");
  Serial.println("  1  accel  +X up        (step 6)");
  Serial.println("  2  accel  -X up");
  Serial.println("  3  accel  +Y up");
  Serial.println("  4  accel  -Y up");
  Serial.println("  5  accel  +Z up (flat)");
  Serial.println("  6  accel  -Z up (upside down)");
  Serial.println("  f  accel fit");
  Serial.println("  s  summary -- paste-ready flags");
  Serial.println("  ?  this menu");
}

// Check 5's third bullet: after correction the gyro should read ~0 at
// rest, and return to ~0 after a rotation rather than settling offset.
void verifyGyro() {
  Serial.println();
  Serial.println("--- Verify gyro at rest ---");
  if (!gyro_bias_valid) {
    Serial.println("  No bias measured yet -- run [g] first.");
    return;
  }
  countdown("verification", 3);
  const Stats st = collect(2.0);

  Serial.println("  corrected means (should be < 0.002 rad/s):");
  bool pass = true;
  for (int i = 0; i < 3; i++) {
    const double corrected = st.mean_g[i] - gyro_bias[i];
    Serial.printf("    %c: %+.6f%s\n", 'X' + i, corrected,
                  fabs(corrected) < 0.002 ? "" : "   <-- HIGH");
    if (fabs(corrected) >= 0.002) { pass = false; }
  }
  Serial.println(pass ? "  PASS" : "  FAIL -- re-run [g] on a stiller surface");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println();
  Serial.println("=== BMI270 calibration ===");

  pinMode(kCsPin, OUTPUT);
  digitalWrite(kCsPin, HIGH);
  SPI.begin();
  delay(10);

  readReg(kRegChipId);                       // mode-select: garbage BY DESIGN
  delay(1);

  const uint8_t id = readReg(kRegChipId);
  Serial.printf("CHIP_ID = 0x%02X\n", id);
  if (id != kChipIdBmi270) {
    Serial.println("FAIL -- expected 0x24.  Re-run imu_spi_test.");
    while (true) { delay(1000); }
  }

  Serial.println("uploading config blob...");
  if (!uploadConfigBlob()) {
    Serial.println("FAIL -- INTERNAL_STATUS never reached 0x01.");
    while (true) { delay(1000); }
  }

  configureSensors();
  settings = SPISettings(kSpiHzData, MSBFIRST, SPI_MODE0);

  Serial.printf("ready -- accel +-2g, gyro +-500dps, 400 Hz, %.1f C\n",
                readTemperature());
  printMenu();
}

void loop() {
  if (!Serial.available()) { return; }

  const char c = Serial.read();
  while (Serial.available()) { Serial.read(); }   // flush CR/LF
  if (c == '\n' || c == '\r') { return; }

  switch (c) {
    case 'l': showLive(); break;
    case 'm': showAxisMapping(); break;
    case 'g': calibrateGyro(); break;
    case 'v': verifyGyro(); break;
    case '1': captureAccelPosition(0, true); break;
    case '2': captureAccelPosition(0, false); break;
    case '3': captureAccelPosition(1, true); break;
    case '4': captureAccelPosition(1, false); break;
    case '5': captureAccelPosition(2, true); break;
    case '6': captureAccelPosition(2, false); break;
    case 'f': showAccelFit(); break;
    case 's': showSummary(); break;
    case '?': printMenu(); break;
    default:
      Serial.print("unknown: ");
      Serial.println(c);
      break;
  }
}
