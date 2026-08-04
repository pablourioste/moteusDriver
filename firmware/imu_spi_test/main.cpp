// BMI270 SPI identity test -- IMU_SETUP.md step 2.
//
// Asks the IMU for its CHIP_ID and nothing else.  Register 0x00 is
// read-only, always reads 0x24, and needs no prior configuration, so it
// exercises the entire SPI round trip -- CS, clock, both data
// directions, logic levels -- while depending on nothing.
//
// Run this BEFORE anything more complex.  A correct 0x24 means the bus
// works; every later failure is then about the sensor, not the wiring.
//
//   pio run -e imu_spi_test              (WSL)     build
//   pio run -e imu_spi_test -t upload    (Windows) flash
//   pio device monitor                   (Windows) read
//
// Wiring (IMU_SETUP.md step 1):
//   BMI270  VDD->3.3V  GND->GND  SCK->13  SDI->11  SDO->12  CSB->10
//
// NOTE: the result prints ONCE, from setup(); loop() is empty.  Attach
// the serial monitor BEFORE resetting, or you will see a blank screen
// and mistake it for a dead board.  The LED cannot help you here either
// -- LED_BUILTIN is pin 13, which SPI.begin() takes over as SCK.

#include <Arduino.h>
#include <SPI.h>

namespace {

constexpr int kCsPin = 10;

// Start at 1 MHz.  The BMI270 tolerates 10 MHz, but a slow clock first
// separates signal-integrity problems from logic bugs -- raise it only
// once the identity read is reliable.
constexpr uint32_t kSpiHz = 1000000;

constexpr uint8_t kRegChipId = 0x00;
constexpr uint8_t kChipIdBmi270 = 0x24;

// SPI read: send `reg | 0x80`, discard ONE dummy byte, then read.
//
// The dummy byte is not optional and has no I2C equivalent -- the
// datasheet (lines 11749, 11815) states the first byte returned on SDO
// is garbage.  Forgetting it shifts every subsequent byte by one and is
// the most common BMI270 SPI bug.
uint8_t readReg(uint8_t reg) {
  SPI.beginTransaction(SPISettings(kSpiHz, MSBFIRST, SPI_MODE0));
  digitalWrite(kCsPin, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(0x00);                       // dummy -- DISCARD
  const uint8_t value = SPI.transfer(0x00);
  digitalWrite(kCsPin, HIGH);
  SPI.endTransaction();
  return value;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println();
  Serial.println("=== BMI270 SPI identity test ===");

  pinMode(kCsPin, OUTPUT);
  digitalWrite(kCsPin, HIGH);               // idle high before SPI.begin
  SPI.begin();
  delay(10);

  // The part powers up in I2C mode and switches to SPI on the first CS
  // falling edge, so this first read returns garbage BY DESIGN.
  // Discard it -- it is the mode-select, not a measurement.
  readReg(kRegChipId);
  delay(1);

  const uint8_t id = readReg(kRegChipId);

  Serial.printf("CHIP_ID = 0x%02X   (expect 0x%02X)\n", id, kChipIdBmi270);

  if (id == kChipIdBmi270) {
    Serial.println("PASS -- BMI270 responding over SPI");
  } else {
    Serial.println("FAIL");
    switch (id) {
      case 0x00:
        Serial.println("  0x00: MISO stuck low -- check SDO->12, check power");
        break;
      case 0xFF:
        Serial.println("  0xFF: MISO floating -- check CSB->10, check wiring");
        break;
      default:
        Serial.println("  Unexpected ID.  A different Bosch part (BMI160 etc),");
        Serial.println("  or bytes shifted -- is the dummy byte being discarded?");
        break;
    }
    Serial.println("  Values that change between runs mean marginal signal:");
    Serial.println("  shorten the leads or drop kSpiHz to 500000.");
  }
}

void loop() {}
