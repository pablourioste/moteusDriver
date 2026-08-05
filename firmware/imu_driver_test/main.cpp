// imu_driver_test -- the cross-validation gate for Bmi270SpiDriver.
//
// Bmi270SpiDriver is the class the control loop will trust for its tilt
// estimate.  It was assembled from firmware/imu_read and
// firmware/imu_calibrate, which are already verified on this hardware --
// but "assembled from verified code" is not the same as verified, and a
// transcription slip in the burst logic or the correction order would
// produce plausible-looking numbers that are quietly wrong.
//
// So this sketch does not ask "does the driver work?".  It asks "does the
// driver agree with the sketches that already work?".  Run it on the same
// bench, in the same orientations, and compare against imu_read's stream
// and imu_calibrate's [g] result.  They must match.
//
//   pio run -e imu_driver_test         (WSL) build, then flash from Windows
//
// Wiring unchanged (IMU_SETUP.md step 1):
//   BMI270  VDD->3.3V  GND->GND  SCK->13  SDI->11  SDO->12  CSB->10
//
// Menu:
//   l  live stream          compare against imu_read
//   g  gyro bias            compare against imu_calibrate [g]
//   t  burst timing         must be < 20 us to fit the control budget
//   c  show calibration     what the build flags actually baked in
//   ?  menu

#include <Arduino.h>

#include <cmath>

#include "core/Types.hpp"
#include "embedded/Bmi270SpiDriver.hpp"
#include "embedded/TeensyClock.hpp"

namespace {

cube::Bmi270SpiDriver* g_imu = nullptr;
cube::TeensyClock g_clock;

// Compare against imu_read's stream in the same orientation.  Gravity
// magnitude is the headline number: it must read 9.81 regardless of how the
// board is turned, because rotating a vector does not change its length.
// A magnitude near 19.6 or 4.9 means the range and the scale factor
// disagree by a factor of two.
void showLive() {
  Serial.println();
  Serial.println("Live stream -- send any key to stop.");
  Serial.println("Compare against `pio run -e imu_read` in the SAME pose.");
  Serial.println("      ax      ay      az   |     gx      gy      gz   |    |a|");

  while (!Serial.available()) {
    const cube::ImuData s = g_imu->read();
    if (!s.valid) {
      Serial.println("  read() returned valid=false -- bus error");
      delay(500);
      continue;
    }
    const double mag = std::sqrt(s.accel_x * s.accel_x +
                                 s.accel_y * s.accel_y +
                                 s.accel_z * s.accel_z);
    Serial.printf("%8.3f%8.3f%8.3f | %7.3f%7.3f%7.3f | %7.3f\n",
                  s.accel_x, s.accel_y, s.accel_z,
                  s.gyro_x, s.gyro_y, s.gyro_z, mag);
    delay(100);
  }
  while (Serial.available()) { Serial.read(); }
}

// Compare against imu_calibrate menu [g].  Same part, same stillness, same
// 0.02 rad/s motion threshold -- so the biases should agree to within the
// noise, and a systematic difference means the two read paths differ.
void calibrateGyro() {
  Serial.println();
  Serial.println("--- Gyro bias via the driver ---");
  Serial.println("  Solid, level, still surface.  Not your hand.");
  Serial.print("  measuring 2 s... ");

  std::string error;
  if (!g_imu->calibrateGyroBias(2.0, &error)) {
    Serial.println("FAIL");
    Serial.printf("  %s\n", error.c_str());
    return;
  }

  const auto& c = g_imu->config();
  Serial.println("OK");
  Serial.printf("  bias = %+.6f %+.6f %+.6f rad/s\n",
                c.gyro_bias_x, c.gyro_bias_y, c.gyro_bias_z);
  Serial.println("  >> Compare against imu_calibrate [g].  They must agree.");
}

// The read path runs every control cycle, so its cost is a hard budget
// item: 5 ms at 200 Hz.  A 12-byte burst at 10 MHz should be ~15 us, i.e.
// 0.3% of the cycle.  Anything near a millisecond means the clock never got
// raised past the 1 MHz upload ceiling.
void showTiming() {
  Serial.println();
  Serial.println("--- read() timing ---");

  constexpr int kIterations = 200;
  const std::uint64_t start = g_clock.micros64();
  for (int i = 0; i < kIterations; i++) {
    const cube::ImuData s = g_imu->read();
    (void)s;
  }
  const std::uint64_t elapsed = g_clock.micros64() - start;

  const double per_read = static_cast<double>(elapsed) / kIterations;
  Serial.printf("  %d reads in %lu us -> %.1f us each\n", kIterations,
                (unsigned long)elapsed, per_read);
  Serial.printf("  %.2f%% of a 5 ms control cycle\n", per_read / 5000.0 * 100.0);

  // read() does two bursts (sample + temperature), so the budget here is
  // looser than the 20 us quoted for a bare 12-byte burst.
  if (per_read > 50.0) {
    Serial.println("  HIGH -- expected well under 50 us.");
    Serial.println("  Is the SPI clock still at the 1 MHz upload ceiling?");
  } else {
    Serial.println("  OK");
  }
}

// What the build flags actually baked in.  Worth checking explicitly:
// a flag that never reached the compiler leaves the identity transform,
// which is an uncalibrated sensor that looks exactly like a calibrated one.
void showCalibration() {
  const auto& c = g_imu->config();
  Serial.println();
  Serial.println("--- Calibration compiled into this binary ---");
  Serial.printf("  gyro bias    %+.6f %+.6f %+.6f rad/s\n",
                c.gyro_bias_x, c.gyro_bias_y, c.gyro_bias_z);
  Serial.printf("  accel offset %+.6f %+.6f %+.6f m/s^2\n",
                c.accel_offset_x, c.accel_offset_y, c.accel_offset_z);
  Serial.printf("  accel scale  %+.6f %+.6f %+.6f\n",
                c.accel_scale_x, c.accel_scale_y, c.accel_scale_z);

  const bool identity =
      c.gyro_bias_x == 0.0 && c.gyro_bias_y == 0.0 && c.gyro_bias_z == 0.0 &&
      c.accel_offset_x == 0.0 && c.accel_offset_y == 0.0 &&
      c.accel_offset_z == 0.0 && c.accel_scale_x == 1.0 &&
      c.accel_scale_y == 1.0 && c.accel_scale_z == 1.0;
  if (identity) {
    Serial.println("  >> ALL IDENTITY -- this build is UNCALIBRATED.");
    Serial.println("     Paste imu_calibrate's [s] output into");
    Serial.println("     platformio.ini [env:imu_driver_test] build_flags.");
  }
}

void printMenu() {
  Serial.println();
  Serial.println("=== Bmi270SpiDriver cross-validation ===");
  Serial.println("  l  live stream      -> compare vs imu_read");
  Serial.println("  g  gyro bias        -> compare vs imu_calibrate [g]");
  Serial.println("  t  read() timing");
  Serial.println("  c  show compiled calibration");
  Serial.println("  ?  this menu");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println();
  Serial.println("=== Bmi270SpiDriver cross-validation ===");

  // FromBuildDefaults() rather than a bare Config: it carries whatever
  // calibration the build flags supplied.
  static cube::Bmi270SpiDriver imu(
      cube::Bmi270SpiDriver::Config::FromBuildDefaults());
  g_imu = &imu;

  Serial.print("initialising... ");
  std::string error;
  if (!imu.initialize(&error)) {
    Serial.println("FAIL");
    Serial.println(error.c_str());
    while (true) { delay(1000); }
  }
  Serial.println("OK");
  Serial.printf("driver reports name: %s\n", imu.name().c_str());

  showCalibration();
  printMenu();
}

void loop() {
  if (!Serial.available()) { return; }

  const char c = Serial.read();
  while (Serial.available()) { Serial.read(); }  // flush CR/LF
  if (c == '\n' || c == '\r') { return; }

  switch (c) {
    case 'l': showLive(); break;
    case 'g': calibrateGyro(); break;
    case 't': showTiming(); break;
    case 'c': showCalibration(); break;
    case '?': printMenu(); break;
    default:
      Serial.print("unknown: ");
      Serial.println(c);
      break;
  }
}
