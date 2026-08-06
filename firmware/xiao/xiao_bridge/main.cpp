// Step X4 -- the real bridge.  Teensy <-> UART <-> Xiao <-> WiFi <-> laptop.
//
//   pio run -e xiao_bridge -t upload
//   pio device monitor -e xiao_bridge          (status only -- NOT the data)
//
//     ┌────────────┐   UART D6/D7    ┌────────────────┐   WiFi / UDP   ┌────────┐
//     │ Teensy 4.1 │ ──telemetry──►  │ XIAO ESP32-C6  │ ──udp/5006──►  │ laptop │
//     │ IMU + CAN  │ ◄──control───   │  (this sketch) │ ◄─udp/5005───  │ python │
//     └────────────┘                 └────────────────┘                └────────┘
//
// THIS IS A DUMB BYTE PIPE, AND IT MUST STAY ONE.
//
// It never parses a frame, never validates a CRC, never reformats
// anything.  Bytes off the UART go out as UDP payload; UDP payload goes
// out of the UART.  That is not laziness -- it is the contract stated in
// include/core/TelemetrySink.hpp, and it is what lets the wire format
// change (new fields, new version) without this firmware being reflashed.
// The moment the bridge understands the payload, it becomes a third place
// that has to agree about the layout, and the Teensy, the host decoder and
// the Xiao start drifting apart in threes.
//
// NO EXTRA FRAMING IS ADDED HERE -- deliberately.
//
// docs/2D_model/WIFI_TELEMETRY_CONTROL_Cubli.md proposes a
// [0xAA][LEN][payload][checksum] envelope at this layer.  Do not add it.
// The payload is already cube::TelemetryFrame, which carries a 0xA5C3
// magic, an explicit length, a version and a CRC-16/CCITT over the whole
// record (include/core/TelemetryFrame.hpp).  Wrapping a framed, checksummed
// record inside a second framed, checksummed envelope buys nothing and
// costs three things: the bridge stops being format-agnostic, the sum-of-
// bytes checksum is strictly weaker than the CRC it duplicates, and
// tools/telemetry/decode.py -- which already resynchronises on the magic --
// would need a second parser in front of the one that works.
//
// BAUD RATE: 921600, NOT 115200.  THIS IS ARITHMETIC, NOT PREFERENCE.
//
//   118 bytes/frame x 200 Hz               = 23 600 bytes/s
//   x 10 bits/byte (8N1: start + 8 + stop) = 236 000 baud MINIMUM
//
// 115200 baud carries 11 520 bytes/s, i.e. 97 frames/s -- less than half
// the balancer's rate.  It does not fail cleanly either: the Teensy's TX
// blocks or this end's RX FIFO overruns, and the result is a stream that
// decodes for a while and then corrupts under load.  At 400 Hz (the IMU's
// ODR) the requirement doubles to 472 kbaud.  921600 gives 92 160 bytes/s,
// ~3.9x headroom at 200 Hz and ~2x at 400 Hz.
//
// The Teensy side MUST be opened at the same rate -- Serial1.begin(921600).
// A mismatch does not produce silence, it produces plausible garbage.
//
// WHAT IS NOT HERE: THE WATCHDOG.
//
// If the WiFi link dies, something must stop the motor.  That decision
// belongs on the TEENSY (docs/2D_model/WIFI_TELEMETRY_CONTROL_Cubli.md
// Step 6), for the obvious reason that a fail-safe implemented on the
// radio module cannot fire when the radio module is what failed.  This
// sketch reports link age on its status line so the behaviour is
// observable, and does nothing about it.
//
// For the same reason this never injects its own bytes into the UART: the
// Teensy's control parser would have to distinguish bridge chatter from
// laptop commands, and a "helpful" keepalive is exactly the kind of thing
// that keeps a watchdog from ever firing.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include <string.h>

namespace {

#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID "CubliNet"
#endif
#ifndef WIFI_AP_PASSWORD
#define WIFI_AP_PASSWORD "cubli1234"
#endif

// See the arithmetic in the header comment before lowering this.
#ifndef UART_BAUD
#define UART_BAUD 921600
#endif

// Upper bound on how long a telemetry frame may sit in the batching
// buffer.  Trades packet rate against latency: at 200 Hz, 10 ms batches
// about 2 frames per datagram (halving the packet rate) while capping the
// added delay at 10 ms.  Raise it for a pure logging run, lower it if you
// are eyeballing --live output and want it to feel immediate.  It does NOT
// affect the control direction, which is forwarded with no batching at all.
#ifndef FLUSH_TIMEOUT_MS
#define FLUSH_TIMEOUT_MS 10
#endif

constexpr int kUartRxPin = D7;      // <- Teensy pin 1 (TX1)
constexpr int kUartTxPin = D6;      // -> Teensy pin 0 (RX1)

constexpr uint16_t kControlPort = 5005;     // laptop -> cube
constexpr uint16_t kTelemetryPort = 5006;   // cube -> laptop

// 944 bytes = 8 x 118, sized to stay under the 1500-byte MTU.  Going over
// does not error, it fragments -- and a lost fragment discards the entire
// datagram, converting a one-frame loss into an eight-frame one.
constexpr size_t kDatagramSize = 944;

// Must be set BEFORE Serial1.begin() or it is silently ignored.  The
// 256-byte default overruns in under 3 ms at 921600 baud, which presents
// as intermittent CRC failures rather than as an obvious error.
constexpr size_t kUartRxBufferSize = 8192;

// Control packets are small; this is deliberately generous so an
// oversized one is truncated at a known boundary rather than smearing.
constexpr size_t kControlBufferSize = 512;

WiFiUDP udp;

uint8_t g_datagram[kDatagramSize];
size_t g_datagram_used = 0;
uint32_t g_datagram_opened_ms = 0;

uint8_t g_control[kControlBufferSize];

IPAddress g_laptop_ip;
bool g_laptop_known = false;
uint32_t g_last_control_ms = 0;

// Counters.  Every one of these answers a specific bring-up question:
// "did the Teensy send anything", "did it reach the air", "did the laptop
// reply", "did anything get dropped, and at which hop".
uint32_t g_uart_rx_bytes = 0;
uint32_t g_uart_tx_bytes = 0;
uint32_t g_udp_tx_bytes = 0;
uint32_t g_udp_tx_packets = 0;
uint32_t g_udp_rx_bytes = 0;
uint32_t g_udp_rx_packets = 0;
uint32_t g_udp_tx_failures = 0;
uint32_t g_uart_tx_dropped = 0;
uint32_t g_telemetry_dropped_no_peer = 0;

void flushDatagram() {
  if (g_datagram_used == 0) { return; }

  if (!g_laptop_known) {
    // Nowhere to send it.  Discarding beats buffering: telemetry from
    // before anyone was listening has no value, and holding it would only
    // deliver a burst of stale frames at connect time.
    g_telemetry_dropped_no_peer += g_datagram_used;
    g_datagram_used = 0;
    return;
  }

  udp.beginPacket(g_laptop_ip, kTelemetryPort);
  const size_t written = udp.write(g_datagram, g_datagram_used);
  if (udp.endPacket() == 1 && written == g_datagram_used) {
    g_udp_tx_packets++;
    g_udp_tx_bytes += g_datagram_used;
  } else {
    // Counted, never retried.  Resending a stale frame delays the next
    // fresh one; the receiver detects the loss from the sequence gap in
    // the frames themselves, which is more informative than a retry.
    g_udp_tx_failures++;
  }
  g_datagram_used = 0;
}

// UART -> WiFi.  Drains the RX buffer completely rather than a fixed 64
// bytes per iteration: a fixed chunk cannot keep up with a burst, and the
// residue accumulates until the buffer overruns.
void pumpTelemetry() {
  int available = Serial1.available();
  while (available > 0) {
    if (g_datagram_used == 0) { g_datagram_opened_ms = millis(); }

    const size_t room = kDatagramSize - g_datagram_used;
    const size_t chunk =
        (static_cast<size_t>(available) < room) ? static_cast<size_t>(available)
                                                : room;

    // readBytes() would apply the 1000 ms stream timeout; the count is
    // already known to be available, so read() straight into the buffer
    // cannot block.
    const int got = Serial1.read(&g_datagram[g_datagram_used], chunk);
    if (got <= 0) { break; }

    g_datagram_used += static_cast<size_t>(got);
    g_uart_rx_bytes += static_cast<uint32_t>(got);

    if (g_datagram_used == kDatagramSize) { flushDatagram(); }
    available = Serial1.available();
  }

  // Bound the time a partial datagram waits, so a stream that pauses does
  // not strand the last few frames indefinitely.
  if (g_datagram_used > 0 &&
      (millis() - g_datagram_opened_ms) >= FLUSH_TIMEOUT_MS) {
    flushDatagram();
  }
}

// WiFi -> UART.  No batching, no delay: this is the control direction and
// every millisecond added here is a millisecond of extra dead time in the
// loop the commands are steering.
void pumpControl() {
  const int len = udp.parsePacket();
  if (len <= 0) { return; }

  const int n = udp.read(g_control, sizeof(g_control));
  if (n <= 0) { return; }

  g_udp_rx_packets++;
  g_udp_rx_bytes += static_cast<uint32_t>(n);
  g_last_control_ms = millis();

  // Learn the peer from whatever last spoke to us, so a laptop that
  // reconnects on a new DHCP lease keeps working without a reflash.
  if (!g_laptop_known || g_laptop_ip != udp.remoteIP()) {
    g_laptop_ip = udp.remoteIP();
    g_laptop_known = true;
    Serial.print("peer           ");
    Serial.println(g_laptop_ip);
  }

  // Refuse rather than block.  A blocking write here would stall the
  // telemetry pump behind a full TX FIFO, and a stalled pump overruns the
  // UART RX buffer -- one slow control packet would corrupt the telemetry
  // stream, which is a spectacularly confusing failure to debug.
  if (Serial1.availableForWrite() >= n) {
    Serial1.write(g_control, static_cast<size_t>(n));
    g_uart_tx_bytes += static_cast<uint32_t>(n);
  } else {
    g_uart_tx_dropped += static_cast<uint32_t>(n);
  }
}

void printStatus() {
  const uint32_t now = millis();
  const uint32_t link_age =
      (g_last_control_ms == 0) ? 0 : (now - g_last_control_ms);

  Serial.printf(
      "up %lus  clients %u  peer %s  link_age %lums | "
      "uart_rx %lu  udp_tx %lu B / %lu pkt  fail %lu | "
      "udp_rx %lu B / %lu pkt  uart_tx %lu  drop %lu/%lu | heap %lu\n",
      static_cast<unsigned long>(now / 1000), WiFi.softAPgetStationNum(),
      g_laptop_known ? g_laptop_ip.toString().c_str() : "none",
      static_cast<unsigned long>(link_age),
      static_cast<unsigned long>(g_uart_rx_bytes),
      static_cast<unsigned long>(g_udp_tx_bytes),
      static_cast<unsigned long>(g_udp_tx_packets),
      static_cast<unsigned long>(g_udp_tx_failures),
      static_cast<unsigned long>(g_udp_rx_bytes),
      static_cast<unsigned long>(g_udp_rx_packets),
      static_cast<unsigned long>(g_uart_tx_bytes),
      static_cast<unsigned long>(g_uart_tx_dropped),
      static_cast<unsigned long>(g_telemetry_dropped_no_peer),
      static_cast<unsigned long>(ESP.getFreeHeap()));
}

}  // namespace

void setup() {
  // The USB port and the bridged UART are two different devices, so this
  // text can never contaminate the binary telemetry stream.  That
  // separation is the only reason a status line is safe here at all.
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial1.setRxBufferSize(kUartRxBufferSize);      // before begin(), always
  Serial1.begin(UART_BAUD, SERIAL_8N1, kUartRxPin, kUartTxPin);

  WiFi.mode(WIFI_AP);
  const bool ap_ok = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
  // Modem sleep parks the radio between beacons and delivers queued
  // packets in bursts at wake-up.  On a logging link that is jitter; on a
  // control link it is tens of milliseconds of unmodelled dead time.
  WiFi.setSleep(false);

  Serial.println();
  Serial.println("=== xiao_bridge ===");
  Serial.printf("uart           %d baud, RX=D7 TX=D6\n",
                static_cast<int>(UART_BAUD));
  Serial.printf("uart capacity  %lu bytes/s (%lu frames/s at 118 B)\n",
                static_cast<unsigned long>(UART_BAUD / 10),
                static_cast<unsigned long>(UART_BAUD / 10 / 118));
  Serial.printf("ssid           %s\n", WIFI_AP_SSID);
  Serial.print("ap ip          ");
  Serial.println(WiFi.softAPIP());
  Serial.printf("control port   %u  (laptop -> cube)\n", kControlPort);
  Serial.printf("telemetry port %u  (cube -> laptop)\n", kTelemetryPort);
  Serial.printf("batching       up to %u bytes / %d ms\n",
                static_cast<unsigned>(kDatagramSize),
                static_cast<int>(FLUSH_TIMEOUT_MS));

  if (!ap_ok) {
    while (true) {
      Serial.println("FAIL softAP() returned false -- check antenna/board");
      delay(1000);
    }
  }

  udp.begin(kControlPort);
  Serial.println("bridge up -- telemetry starts once the laptop announces itself");
  Serial.println();
}

void loop() {
  // Control first: it is latency-critical and the cheaper of the two, so
  // servicing it ahead of a potentially large telemetry drain keeps the
  // command path's worst case short.
  pumpControl();
  pumpTelemetry();

  static uint32_t next_status_ms = 0;
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - next_status_ms) >= 0) {
    next_status_ms = now + 1000;
    printStatus();
  }
}
