// SPI peripheral loopback -- IMU_SETUP.md step 0.6.
//
// Proves the Teensy's SPI hardware works with NO sensor attached, so a
// later CHIP_ID failure cannot be blamed on the peripheral.
//
// Wiring: a single jumper from pin 11 (MOSI) to pin 12 (MISO).
//         Nothing else.  Remove it before wiring the IMU.
//
// Whatever is transmitted comes straight back.  Remove the jumper and
// the test must FAIL -- that is what proves the test is real rather than
// reporting success from a disconnected bus.
//
//   pio run -e spi_loopback -t upload
//   pio device monitor

#include <Arduino.h>
#include <SPI.h>

namespace {

constexpr uint32_t kSpiHz = 1000000;

// Distinctive alternating bit pattern.  0x00 or 0xFF would be
// indistinguishable from a stuck-low or floating line.
constexpr uint8_t kPattern[] = {0xA5, 0x5A, 0x0F, 0xF0, 0x42};

}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println();
  Serial.println("=== SPI loopback test ===");
  Serial.println("Jumper pin 11 (MOSI) -> pin 12 (MISO)");
  Serial.println();

  SPI.begin();
  delay(10);

  int passed = 0;
  const int total = sizeof(kPattern) / sizeof(kPattern[0]);

  for (int i = 0; i < total; i++) {
    SPI.beginTransaction(SPISettings(kSpiHz, MSBFIRST, SPI_MODE0));
    const uint8_t got = SPI.transfer(kPattern[i]);
    SPI.endTransaction();

    const bool ok = (got == kPattern[i]);
    if (ok) { passed++; }
    Serial.printf("  sent 0x%02X  got 0x%02X  %s\n",
                  kPattern[i], got, ok ? "ok" : "MISMATCH");
  }

  Serial.println();
  if (passed == total) {
    Serial.printf("PASS -- %d/%d.  SPI peripheral works.\n", passed, total);
    Serial.println("Now REMOVE the jumper and re-run: it must FAIL.");
    Serial.println("A test that passes either way proves nothing.");
  } else {
    Serial.printf("FAIL -- %d/%d matched.\n", passed, total);
    Serial.println("  All 0x00 -> jumper missing, or MISO shorted low");
    Serial.println("  All 0xFF -> jumper missing, MISO floating high");
    Serial.println("  Otherwise -> check the jumper is 11 to 12");
  }
}

void loop() {}
