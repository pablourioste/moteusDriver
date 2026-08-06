// Binary telemetry over USB CDC -- the first consumer of TelemetryFrame.
//
// Streams real BMI270 samples as 118-byte binary records instead of
// printf'd text.  This is step 3 of the telemetry build order: it proves
// the frame layout, the ring, and the Python decoder against hardware that
// already works, over a link (USB) that does not drop bytes.  Once this is
// clean, swapping the sink for a UART to the Xiao is a one-line change and
// any remaining corruption is unambiguously the radio's fault.
//
//   pio run -e telemetry_test          (WSL) build, then flash from Windows
//
// Wiring is unchanged from imu_read / imu_calibrate (IMU_SETUP.md step 1):
//   BMI270  VDD->3.3V  GND->GND  SCK->13  SDI->11  SDO->12  CSB->10
//
// THIS SKETCH EMITS BINARY, NOT TEXT.  Do not open it in a serial monitor
// expecting to read it -- you will see garbage, and a monitor that echoes
// or line-buffers can interfere with the capture.  Use:
//
//   python.exe tools/telemetry/capture.py --port COM9 --out logs/t.bin --live
//
// The estimator / controller / motor fields are zero here: those modules
// are still scaffolds and there is no CAN link on this sketch.  The frame
// is nevertheless emitted at FULL SIZE from day one, so the schema -- and
// therefore every log recorded with it -- stays valid when they are filled
// in.  Adding fields later would invalidate every capture taken before.
//
// Interactive commands (single key, no Enter needed):
//   s  print a human-readable status line to the SECOND USB serial port
//   r  reset counters
//
// Note the status text goes to SerialUSB1 when built with USB_DUAL_SERIAL,
// never to the binary stream -- mixing text into the frame stream is
// exactly the resync problem the format exists to avoid.

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

#include "bmi270_config.h"
#include "core/TelemetryFrame.hpp"
#include "core/TelemetryRing.hpp"
#include "core/TelemetrySink.hpp"

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

// 400 Hz, matching the configured ODR.  Sampling faster than the ODR just
// re-reads the same register contents and inflates the log with duplicates.
constexpr uint32_t kSamplePeriodUs = 2500;

// Datasheet sensitivities, for the raw<->SI cross-check the host decoder
// performs.  BMI270 datasheet: 16384 LSB/g at +-2 g, 65.536 LSB/dps at
// +-500 dps.  Equivalent to range*unit/32768 -- expressed that way here to
// match the existing driver, and as explicit LSB constants in scale.py so
// the two derivations can be compared.
double accel_scale = 0.0;
double gyro_scale = 0.0;

SPISettings settings(kSpiHzUpload, MSBFIRST, SPI_MODE0);

// 64 frames ~= 7.5 kB, ~160 ms of slack at 400 Hz.  Sized to ride out a
// host that pauses briefly, not to buffer a disconnected link.
cube::TelemetryRing<64> g_ring;

uint32_t g_dropped = 0;
uint32_t g_written = 0;
uint32_t g_loop_late = 0;

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

// Unlike imu_read's readSample(), this KEEPS the raw counts.  That is the
// one substantive difference: the raw int16 is what lets a log be
// re-processed with better calibration later, and scaling discards it
// irreversibly.
struct RawSample {
  int16_t raw_accel[3];
  int16_t raw_gyro[3];
  float accel_si[3];
  float gyro_si[3];
};

RawSample readSample() {
  uint8_t buffer[12];
  readBurst(kRegAccXLsb, buffer, sizeof(buffer));

  RawSample s;
  for (int i = 0; i < 3; i++) {
    s.raw_accel[i] = toInt16(&buffer[i * 2]);
    s.raw_gyro[i] = toInt16(&buffer[6 + i * 2]);
    s.accel_si[i] = static_cast<float>(s.raw_accel[i] * accel_scale);
    // No bias subtraction here: gyro bias is a calibration output from
    // imu_calibrate, and this sketch deliberately logs uncorrected values
    // so the raw and SI columns differ only by the datasheet scale factor.
    // That makes the host-side cross-check unambiguous.
    s.gyro_si[i] = static_cast<float>(s.raw_gyro[i] * gyro_scale);
  }
  return s;
}

float readTemperature() {
  uint8_t buffer[2];
  readBurst(kRegTemperature, buffer, sizeof(buffer));
  const int16_t raw = toInt16(buffer);
  // 0x8000 is the "invalid" sentinel -- which is why a reading of exactly
  // 0 deserves suspicion rather than trust.
  return (raw == static_cast<int16_t>(0x8000))
             ? NAN
             : static_cast<float>((raw / 512.0) + 23.0);
}

// ------------------------------------------------------------ transport

// USB CDC sink.
//
// availableForWrite() is the whole reason this is safe: it reports how
// many bytes the endpoint will take right now, so the drain can decline to
// write instead of blocking.  Without it, Serial.write() blocks whenever
// the host stops reading, and the sample loop dies with it -- on the real
// balancer that is a fall, not a lost log.
class UsbCdcSink : public cube::ITelemetrySink {
 public:
  int space() override { return Serial.availableForWrite(); }

  size_t write(const uint8_t* data, size_t length) override {
    return Serial.write(data, length);
  }

  const char* name() const override { return "usb_cdc"; }
};

#ifdef TELEMETRY_SINK_UART1

// The Xiao seam.  Same frames, same ring, same drain policy -- only the
// pipe underneath changes, which is precisely what ITelemetrySink exists
// to make possible.  Built ONLY by -e telemetry_test_uart; the default
// env compiles none of this and its USB behaviour is unchanged.
//
// BAUD IS 921600, NOT 115200, AND THAT IS ARITHMETIC:
//   118 bytes x 400 Hz x 10 bits/byte (8N1) = 472 000 baud minimum.
// 115200 carries 11.5 kB/s -- 97 frames/s, a quarter of what this sketch
// produces.  The overflow does not announce itself; it appears as frames
// the host cannot decode.  Keep this in step with UART_BAUD in
// firmware/xiao/xiao_bridge/main.cpp.
constexpr uint32_t kUartBaud = 921600;

// ** THE NON-OBVIOUS ONE. **
//
// Serial1's default TX buffer on Teensy 4.x is SERIAL1_TX_BUFFER_SIZE = 64
// bytes (cores/teensy4/HardwareSerial1.cpp).  A telemetry frame is 118.
// DrainTelemetry() refuses to write unless space() >= sizeof(frame), so
// with the stock buffer availableForWrite() never once reaches 118 and
// NOT A SINGLE FRAME IS EVER SENT -- silently, with no error anywhere.
// The symptom is a bridge whose uart_rx counter stays at zero while this
// sketch looks perfectly healthy.
//
// addMemoryForWrite() extends the buffer; 4 kB is ~34 frames, enough to
// absorb a burst without the drain ever seeing a full buffer.
uint8_t g_uart_tx_buffer[4096];

class Uart1Sink : public cube::ITelemetrySink {
 public:
  int space() override { return Serial1.availableForWrite(); }

  size_t write(const uint8_t* data, size_t length) override {
    return Serial1.write(data, length);
  }

  const char* name() const override { return "uart1_xiao"; }
};

Uart1Sink g_sink;

#else

UsbCdcSink g_sink;

#endif  // TELEMETRY_SINK_UART1

}  // namespace

void setup() {
  Serial.begin(115200);        // rate ignored: Teensy USB CDC runs at
                               // 480 Mbit/s regardless.  This is why 118
                               // bytes at 400 Hz (47 kB/s) fits here but
                               // would NOT fit a real 115200 UART.
  while (!Serial && millis() < 3000) {}

#ifdef TELEMETRY_SINK_UART1
  // Extend the TX buffer BEFORE begin(), for the reason spelled out at
  // g_uart_tx_buffer: 64 bytes < 118, so without this the drain never
  // fires and the sketch emits nothing at all.
  Serial1.addMemoryForWrite(g_uart_tx_buffer, sizeof(g_uart_tx_buffer));
  Serial1.begin(kUartBaud);
#endif

  pinMode(kCsPin, OUTPUT);
  digitalWrite(kCsPin, HIGH);
  SPI.begin();
  delay(10);

  readReg(kRegChipId);                       // mode-select: garbage BY DESIGN
  delay(1);

  const uint8_t id = readReg(kRegChipId);
  if (id != kChipIdBmi270) {
    // No frames will ever be emitted, so a text complaint on the binary
    // port is harmless here and is the only way to report this.
    while (true) {
      Serial.printf("FAIL CHIP_ID=0x%02X expected 0x24 -- run imu_spi_test\n",
                    id);
      delay(1000);
    }
  }

  if (!uploadConfigBlob()) {
    while (true) {
      Serial.println("FAIL INTERNAL_STATUS never reached 0x01");
      delay(1000);
    }
  }

  configureSensors();
  settings = SPISettings(kSpiHzData, MSBFIRST, SPI_MODE0);
}

void loop() {
  static uint32_t next_deadline_us = micros();
  static uint32_t last_sample_us = micros();

  const uint32_t now_us = micros();

  // Unsigned subtraction, so this stays correct across the 71.6 minute
  // micros() rollover -- the same reason the host decoder unwraps t_us
  // rather than trusting raw differences.
  const uint32_t elapsed = now_us - last_sample_us;
  last_sample_us = now_us;

  const RawSample sample = readSample();
  const float temp_c = readTemperature();

  cube::TelemetryFrame frame{};
  for (int i = 0; i < 3; i++) {
    frame.raw_accel[i] = sample.raw_accel[i];
    frame.raw_gyro[i] = sample.raw_gyro[i];
    frame.accel_si[i] = sample.accel_si[i];
    frame.gyro_si[i] = sample.gyro_si[i];
  }
  frame.imu_temp_c = temp_c;
  frame.t_us = now_us;
  frame.dt_s = static_cast<float>(elapsed) * 1e-6f;

  // Estimator / controller / motor fields stay zero: those modules are
  // scaffolds and this sketch has no CAN link.  Full-size frame anyway,
  // so the schema does not change when they are implemented.
  frame.flags = cube::kFlagImuValid | cube::kFlagDryRun;

  const uint32_t before_push = micros();
  if (!g_ring.push(&frame)) { g_dropped++; }
  const uint32_t push_us = micros() - before_push;
  // push() must be constant-time.  If it ever exceeds a quarter of the
  // sample period something is very wrong -- most likely the CRC ended up
  // in an unoptimised build.
  if (push_us > kSamplePeriodUs / 4) { g_loop_late++; }

  // Drain OUTSIDE the sampling deadline, with a budget so a backlog is
  // paid down over several cycles rather than in one burst that itself
  // causes an overrun.
  g_written += cube::DrainTelemetry(&g_ring, &g_sink, 4);

  // Absolute-deadline scheduling, matching cube_balancer.cpp:300-313.
  next_deadline_us += kSamplePeriodUs;
  const int32_t slack = static_cast<int32_t>(next_deadline_us - micros());
  if (slack > 0) {
    delayMicroseconds(static_cast<uint32_t>(slack));
  } else {
    g_loop_late++;
    // Too far behind to catch up -- resynchronise rather than spin trying
    // to make up an unbounded backlog of missed deadlines.
    if (-slack > static_cast<int32_t>(kSamplePeriodUs)) {
      next_deadline_us = micros() + kSamplePeriodUs;
    }
  }
}
