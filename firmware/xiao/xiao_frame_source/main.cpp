// Step X3 -- does the WiFi path carry REAL telemetry frames intact?
//
// A synthetic telemetry source: the Xiao itself generates valid 118-byte
// cube::TelemetryFrame records and sends them over UDP, with no Teensy, no
// UART and no IMU anywhere in the picture.
//
//   pio run -e xiao_frame_source -t upload
//   ./moteus-venv/bin/python tools/telemetry/capture.py
//       --udp 0.0.0.0:5006 --out logs/wifi_synth.bin -s 60 --live
//   ./moteus-venv/bin/python tools/telemetry/decode.py logs/wifi_synth.bin
//
// WHY THIS SKETCH EXISTS.  The full chain is IMU -> Teensy -> UART -> Xiao
// -> WiFi -> laptop, and when the first end-to-end run shows corrupt
// frames there are five candidate culprits.  This one removes four of
// them: the frames are generated with a known sequence and a known
// waveform, so anything the laptop fails to decode was lost or mangled by
// the radio and by nothing else.  Run it BEFORE wiring the UART, and the
// later bring-up has one unknown instead of five.
//
// It doubles as the acceptance test for the host tooling.  capture.py's
// --udp path, decode.py's CRC and sequence accounting, and the raw-vs-SI
// scale cross-check are all exercised here without a single wire.
//
// THE FRAME FORMAT IS REUSED, NOT REIMPLEMENTED.  This compiles
// include/core/TelemetryFrame.hpp and src/core/TelemetryFrame.cpp -- the
// exact files the Teensy firmware and the Linux host build compile.  The
// static_assert on sizeof() therefore now holds across THREE toolchains
// (Cortex-M7, RISC-V, x86-64), which is the only real proof that the
// packed layout is portable rather than a compiler coincidence.  A copied
// struct definition here would test nothing.
//
// BATCHING.  Several frames go per datagram.  One 118-byte frame per
// packet wastes 42 bytes of IP+UDP header plus a full WiFi frame exchange
// on every record; at 200 Hz that is 200 packets/s of avoidable airtime.
// Frames are self-delimiting (magic + length + CRC), so the receiver
// neither knows nor cares how they were packetised -- capture.py writes
// datagram payloads to disk back-to-back and decode.py resynchronises on
// the magic.  See the comment in capture.py::capture_udp.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include <math.h>
#include <string.h>

#include "core/TelemetryFrame.hpp"

namespace {

#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID "CubliNet"
#endif
#ifndef WIFI_AP_PASSWORD
#define WIFI_AP_PASSWORD "cubli1234"
#endif

// Matches cube_balancer's control rate, so the packet rate and bandwidth
// measured here are the ones the real system will produce.
#ifndef FRAME_RATE_HZ
#define FRAME_RATE_HZ 200
#endif

constexpr uint16_t kControlPort = 5005;
constexpr uint16_t kTelemetryPort = 5006;

constexpr uint32_t kFramePeriodUs = 1000000UL / FRAME_RATE_HZ;

// 8 x 118 = 944 bytes, comfortably inside the 1500-byte Ethernet MTU that
// the WiFi stack fragments at.  Exceeding it does not fail loudly -- it
// fragments, and a lost fragment costs the WHOLE datagram, which turns a
// 1-frame loss into an 8-frame loss.
constexpr size_t kFramesPerDatagram = 8;
constexpr size_t kDatagramSize =
    kFramesPerDatagram * cube::kTelemetryFrameSize;

// Datasheet scale factors, spelled exactly as tools/telemetry/scale.py
// spells them.  The synthetic raw counts and SI values must agree to
// within float32 round-off or decode.py's cross-check warns -- which is
// the point: this sketch proves the check works before real data depends
// on it.
constexpr double kAccelLsbPerG = 16384.0;      // +-2 g,   ACC_RANGE 0
constexpr double kGyroLsbPerDps = 65.536;      // +-500 dps, GYR_RANGE 2
constexpr double kGravity = 9.80665;
constexpr double kDegToRad = 0.017453292519943295;

WiFiUDP udp;

uint8_t g_datagram[kDatagramSize];
size_t g_datagram_used = 0;

IPAddress g_laptop_ip;
bool g_laptop_known = false;

uint32_t g_seq = 0;
uint32_t g_frames_sent = 0;
uint32_t g_datagrams_sent = 0;
uint32_t g_send_failures = 0;
uint32_t g_frames_dropped_no_peer = 0;
uint32_t g_late_cycles = 0;

// Flush whatever is buffered.  Called when the buffer fills; there is no
// timeout flush because at 200 Hz a full datagram accumulates in 40 ms and
// the stream never goes idle.
void flushDatagram() {
  if (g_datagram_used == 0 || !g_laptop_known) { return; }

  udp.beginPacket(g_laptop_ip, kTelemetryPort);
  const size_t written = udp.write(g_datagram, g_datagram_used);
  const bool ok = udp.endPacket() == 1 && written == g_datagram_used;

  if (ok) {
    g_datagrams_sent++;
    g_frames_sent += g_datagram_used / cube::kTelemetryFrameSize;
  } else {
    // Counted, not retried.  A retry would delay the NEXT frame to resend
    // a stale one, which on a real-time link is strictly worse than the
    // loss -- and the sequence gap tells the host exactly what went
    // missing anyway.
    g_send_failures++;
  }
  g_datagram_used = 0;
}

// A frame whose contents are predictable, so the host can tell corruption
// from noise.  Two incommensurate frequencies (0.5 Hz and 1.3 Hz) mean a
// misordered or duplicated frame is visible in a plot rather than hiding
// inside a repeating pattern.
void buildFrame(cube::TelemetryFrame* frame, uint32_t now_us, float dt_s) {
  memset(frame, 0, sizeof(*frame));

  const double t = now_us * 1e-6;

  // Generate the RAW counts first and derive SI from them, never the other
  // way round.  That is the same direction the real firmware works in, and
  // it is what makes decode.py's cross-check meaningful instead of
  // circular.
  const int16_t raw_ax = static_cast<int16_t>(8000.0 * sin(2 * PI * 0.5 * t));
  const int16_t raw_ay = static_cast<int16_t>(8000.0 * cos(2 * PI * 0.5 * t));
  const int16_t raw_az = static_cast<int16_t>(kAccelLsbPerG);   // 1 g on Z
  const int16_t raw_gx = static_cast<int16_t>(4000.0 * sin(2 * PI * 1.3 * t));
  const int16_t raw_gy = static_cast<int16_t>(4000.0 * cos(2 * PI * 1.3 * t));
  const int16_t raw_gz = 0;

  frame->raw_accel[0] = raw_ax;
  frame->raw_accel[1] = raw_ay;
  frame->raw_accel[2] = raw_az;
  frame->raw_gyro[0] = raw_gx;
  frame->raw_gyro[1] = raw_gy;
  frame->raw_gyro[2] = raw_gz;

  for (int i = 0; i < 3; i++) {
    frame->accel_si[i] =
        static_cast<float>(frame->raw_accel[i] * kGravity / kAccelLsbPerG);
    frame->gyro_si[i] =
        static_cast<float>(frame->raw_gyro[i] * kDegToRad / kGyroLsbPerDps);
  }

  frame->imu_temp_c = 25.0f;
  frame->seq = g_seq++;
  frame->t_us = now_us;
  frame->dt_s = dt_s;
  frame->slack_s = 0.0f;

  // Estimator / controller / motor fields stay zero, exactly as in
  // firmware/full_case/telemetry_test: those modules are scaffolds.  The
  // frame is emitted at FULL SIZE regardless so the schema -- and every
  // log recorded against it -- survives them being filled in.
  frame->flags = cube::kFlagImuValid | cube::kFlagDryRun;

  // Last, after every other field: this stamps magic/length/version and
  // computes the CRC over the populated body.
  cube::FinalizeFrame(frame);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println();
  Serial.println("=== xiao_frame_source ===");
  Serial.printf("frame size     %u bytes\n",
                static_cast<unsigned>(cube::kTelemetryFrameSize));
  Serial.printf("rate           %d Hz\n", static_cast<int>(FRAME_RATE_HZ));
  Serial.printf("batching       %u frames/datagram (%u bytes)\n",
                static_cast<unsigned>(kFramesPerDatagram),
                static_cast<unsigned>(kDatagramSize));
  Serial.printf("payload rate   %lu bytes/s\n",
                static_cast<unsigned long>(cube::kTelemetryFrameSize *
                                           FRAME_RATE_HZ));

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
  WiFi.setSleep(false);        // see xiao_softap: latency, not power

  Serial.print("ap ip          ");
  Serial.println(WiFi.softAPIP());

  udp.begin(kControlPort);

  Serial.println();
  Serial.println("Waiting for the laptop to announce itself.  Frames are sent");
  Serial.println("to whoever last sent a packet to udp/5005, so kick it off:");
  Serial.println("  echo hello | nc -u -w1 192.168.4.1 5005");
  Serial.println("then capture:");
  Serial.println("  capture.py --udp 0.0.0.0:5006 --out logs/wifi_synth.bin");
  Serial.println();
}

void loop() {
  static uint32_t next_deadline_us = micros();
  static uint32_t last_frame_us = micros();

  // Learn the laptop's address from whatever it sends.  Hardcoding it
  // would make a DHCP lease change a firmware edit.
  const int len = udp.parsePacket();
  if (len > 0) {
    uint8_t scratch[256];
    udp.read(scratch, sizeof(scratch));
    if (!g_laptop_known || g_laptop_ip != udp.remoteIP()) {
      g_laptop_ip = udp.remoteIP();
      g_laptop_known = true;
      Serial.print("peer           ");
      Serial.println(g_laptop_ip);
    }
  }

  const uint32_t now_us = micros();
  // Unsigned subtraction stays correct across the micros() rollover, which
  // on this part is every 71.6 minutes -- the same wrap the frame's t_us
  // field carries and the Python decoder unwraps.
  const uint32_t elapsed_us = now_us - last_frame_us;
  last_frame_us = now_us;

  cube::TelemetryFrame frame;
  buildFrame(&frame, now_us, static_cast<float>(elapsed_us) * 1e-6f);

  if (g_laptop_known) {
    memcpy(&g_datagram[g_datagram_used], &frame, sizeof(frame));
    g_datagram_used += sizeof(frame);
    if (g_datagram_used + sizeof(frame) > kDatagramSize) { flushDatagram(); }
  } else {
    // Sequence numbers still advance: the host then sees the stream start
    // partway through rather than at zero, which is honest about how much
    // was generated before anyone was listening.
    g_frames_dropped_no_peer++;
  }

  static uint32_t next_status_ms = 0;
  const uint32_t now_ms = millis();
  if (static_cast<int32_t>(now_ms - next_status_ms) >= 0) {
    next_status_ms = now_ms + 1000;
    Serial.printf(
        "seq %lu  frames %lu  dgrams %lu  fail %lu  nopeer %lu  late %lu  "
        "clients %u  heap %lu\n",
        static_cast<unsigned long>(g_seq),
        static_cast<unsigned long>(g_frames_sent),
        static_cast<unsigned long>(g_datagrams_sent),
        static_cast<unsigned long>(g_send_failures),
        static_cast<unsigned long>(g_frames_dropped_no_peer),
        static_cast<unsigned long>(g_late_cycles),
        WiFi.softAPgetStationNum(),
        static_cast<unsigned long>(ESP.getFreeHeap()));
  }

  // Absolute-deadline scheduling, matching telemetry_test and
  // cube_balancer: accumulating the period onto the previous deadline
  // keeps the average rate exact, whereas delay(period) accumulates every
  // cycle's overhead as permanent drift.
  next_deadline_us += kFramePeriodUs;
  const int32_t slack = static_cast<int32_t>(next_deadline_us - micros());
  if (slack > 0) {
    delayMicroseconds(static_cast<uint32_t>(slack));
  } else {
    g_late_cycles++;
    if (-slack > static_cast<int32_t>(kFramePeriodUs)) {
      next_deadline_us = micros() + kFramePeriodUs;
    }
  }
}
