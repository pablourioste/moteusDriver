// BMI270 config blob upload -- IMU_SETUP.md step 3.
//
// The BMI270 ships with its accelerometer and gyroscope DISABLED.  They
// stay disabled until Bosch's 8 KB configuration blob is uploaded into
// the sensor's internal core.  Until then every data register reads
// zero -- while CHIP_ID reads a perfect 0x24.  That combination is the
// trap this step exists to avoid: the bus looks healthy and the sensor
// looks dead.
//
// Ported from src/drivers/ImuDriver.cpp:91-178 (the I2C version).  The
// state machine is unchanged; only the two bus functions differ:
//   - reads gain the SPI dummy byte
//   - the burst write holds CS low for the whole chunk
// Chunk size goes 32 -> 256 bytes, since SPI has no I2C length limit:
// 32 transactions instead of 256.
//
// Run AFTER imu_spi_test prints 0x24.  This pushes 8 KB over the bus; a
// marginal link fails here in a way that mimics a wiring fault.
//
//   pio run -e imu_config_upload         (WSL) build, then flash the
//                                        .hex with Teensy Loader
//
// Wiring is unchanged from imu_spi_test (IMU_SETUP.md step 1):
//   BMI270  VDD->3.3V  GND->GND  SCK->13  SDI->11  SDO->12  CSB->10
//
// NOTE: prints once from setup(); loop() is empty.  Attach the serial
// monitor BEFORE resetting or you will see a blank screen.  The LED is
// unavailable -- LED_BUILTIN is pin 13, which SPI.begin() takes as SCK.

#include <Arduino.h>
#include <SPI.h>

#include "bmi270_config.h"

namespace {

constexpr int kCsPin = 10;

// 1 MHz.  The upload is the one operation with a HARD clock ceiling:
// above ~2 MHz INTERNAL_STATUS never reaches 0x01 and it looks exactly
// like a wiring fault.  The normal data path may run at 10 MHz later --
// this constant governs the upload only.
constexpr uint32_t kSpiHz = 1000000;

constexpr uint8_t kRegChipId = 0x00;
constexpr uint8_t kRegInternalStatus = 0x21;
constexpr uint8_t kRegInitCtrl = 0x59;
constexpr uint8_t kRegInitAddr0 = 0x5B;
constexpr uint8_t kRegInitAddr1 = 0x5C;
constexpr uint8_t kRegInitData = 0x5E;
constexpr uint8_t kRegPwrConf = 0x7C;
constexpr uint8_t kRegPwrCtrl = 0x7D;

constexpr uint8_t kChipIdBmi270 = 0x24;
constexpr uint8_t kInternalStatusMask = 0x0F;
constexpr uint8_t kInternalStatusReady = 0x01;

// Bytes per burst.  Must be EVEN: INIT_ADDR addresses the internal
// write pointer in half-words, so an odd chunk would desynchronise the
// address arithmetic from the byte offset.
constexpr size_t kChunk = 256;

const SPISettings kSettings(kSpiHz, MSBFIRST, SPI_MODE0);

// SPI read: send `reg | 0x80`, discard ONE dummy byte, then read.
// Datasheet lines 11749 / 11815: the first byte on SDO is garbage.
uint8_t readReg(uint8_t reg) {
  SPI.beginTransaction(kSettings);
  digitalWrite(kCsPin, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(0x00);                       // dummy -- DISCARD
  const uint8_t value = SPI.transfer(0x00);
  digitalWrite(kCsPin, HIGH);
  SPI.endTransaction();
  return value;
}

// SPI write: register address with bit 7 CLEAR, then the byte.  No
// dummy byte on the write path -- that asymmetry is easy to get wrong.
void writeReg(uint8_t reg, uint8_t value) {
  SPI.beginTransaction(kSettings);
  digitalWrite(kCsPin, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(value);
  digitalWrite(kCsPin, HIGH);
  SPI.endTransaction();
}

// Burst write: register byte + payload in ONE CS assertion.
//
// CS MUST stay low for the whole chunk.  The BMI270's internal write
// pointer auto-increments only while CS is asserted; releasing it
// mid-chunk restarts the pointer and silently corrupts the blob.  This
// is why the payload cannot be sent with repeated writeReg() calls.
void writeBurst(uint8_t reg, const uint8_t* data, size_t len) {
  SPI.beginTransaction(kSettings);
  digitalWrite(kCsPin, LOW);
  SPI.transfer(reg & 0x7F);
  for (size_t i = 0; i < len; i++) {
    SPI.transfer(data[i]);
  }
  digitalWrite(kCsPin, HIGH);
  SPI.endTransaction();
}

bool uploadConfigBlob() {
  const size_t blob_size = sizeof(bmi270_config_file);

  // 1. Disable advanced power save -- the upload requires the device
  //    awake.  The datasheet asks for 450 us of settling afterwards.
  writeReg(kRegPwrConf, 0x00);
  delayMicroseconds(450);

  // 2. Announce the upload.
  writeReg(kRegInitCtrl, 0x00);

  // 3. Push the blob.  INIT_ADDR_0/1 set the internal write pointer in
  //    HALF-WORD units, split across nibbles: ADDR_0 is bits [3:0],
  //    ADDR_1 is bits [11:4].
  Serial.print("  uploading ");
  Serial.print(blob_size);
  Serial.print(" bytes in ");
  Serial.print((blob_size + kChunk - 1) / kChunk);
  Serial.println(" chunks");

  for (size_t offset = 0; offset < blob_size; offset += kChunk) {
    const size_t len =
        (offset + kChunk <= blob_size) ? kChunk : (blob_size - offset);

    const size_t half_word = offset / 2;
    writeReg(kRegInitAddr0, static_cast<uint8_t>(half_word & 0x0F));
    writeReg(kRegInitAddr1, static_cast<uint8_t>((half_word >> 4) & 0xFF));

    writeBurst(kRegInitData, bmi270_config_file + offset, len);
  }

  // 4. Signal completion.
  writeReg(kRegInitCtrl, 0x01);

  // 5. The datasheet allows ~20 ms for the core to digest the config.
  //    Poll rather than sleeping blindly so a fast part is not
  //    penalised.  40 ms of headroom.
  for (int attempt = 0; attempt < 40; attempt++) {
    delay(1);
    const uint8_t status = readReg(kRegInternalStatus);
    if ((status & kInternalStatusMask) == kInternalStatusReady) {
      Serial.print("  INTERNAL_STATUS = 0x");
      Serial.print(status, HEX);
      Serial.print(" after ");
      Serial.print(attempt + 1);
      Serial.println(" ms");
      return true;
    }
  }

  Serial.print("  INTERNAL_STATUS = 0x");
  Serial.print(readReg(kRegInternalStatus), HEX);
  Serial.println(" -- never reached 0x01");
  return false;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println();
  Serial.println("=== BMI270 config blob upload ===");

  pinMode(kCsPin, OUTPUT);
  digitalWrite(kCsPin, HIGH);               // idle high before SPI.begin
  SPI.begin();
  delay(10);

  // The part powers up in I2C mode and switches to SPI on the first CS
  // falling edge, so this first read returns garbage BY DESIGN.
  readReg(kRegChipId);
  delay(1);

  // Re-verify identity before pushing 8 KB.  If the bus is not healthy
  // now, every failure below would be misattributed to the blob.
  const uint8_t id = readReg(kRegChipId);
  Serial.print("CHIP_ID = 0x");
  Serial.println(id, HEX);
  if (id != kChipIdBmi270) {
    Serial.println("FAIL -- expected 0x24.  Fix the bus first; re-run");
    Serial.println("  imu_spi_test.  Not uploading the blob.");
    return;
  }

  if (!uploadConfigBlob()) {
    Serial.println("FAIL -- config upload did not complete");
    Serial.println("  never 0x01  -> SPI clock too fast (kSpiHz > 2 MHz)");
    Serial.println("  never 0x01  -> CS released mid-chunk");
    Serial.println("  reads 0x00  -> blob never arrived; check writeBurst");
    Serial.println("  was working -> a soft reset wipes the blob; it must");
    Serial.println("                 be re-uploaded after every reset");
    return;
  }

  Serial.println("PASS -- config accepted, sensor core initialised");

  // Enable the accelerometer and gyroscope.  Until now they have been
  // powered down; this is what makes the data registers non-zero.
  // 0x0E = acc_en | gyr_en | temp_en.
  writeReg(kRegPwrCtrl, 0x0E);
  delay(10);
  Serial.print("PWR_CTRL = 0x");
  Serial.println(readReg(kRegPwrCtrl), HEX);

  Serial.println();
  Serial.println("NOTE: the blob lives in volatile memory.  Any reset or");
  Serial.println("power cycle clears it and this must run again.");
}

void loop() {}
