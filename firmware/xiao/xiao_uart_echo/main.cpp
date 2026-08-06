// Step X1 -- does the UART link to the Teensy work, in both directions?
//
// Wiring (both boards are 3.3 V logic -- direct wire, no level shifter):
//
//    XIAO ESP32-C6                     Teensy 4.1
//   ┌───────────────┐                 ┌──────────────────┐
//   │  GND ─────────┼─────────────────┤ GND              │
//   │  D6 (TX) ─────┼─────────────────┤ 0   (RX1)        │
//   │  D7 (RX) ─────┼─────────────────┤ 1   (TX1)        │
//   └───────────────┘                 └──────────────────┘
//
// THE GND WIRE IS NOT OPTIONAL.  Two boards on separate USB ports may sit
// at different ground potentials; without the common return the UART reads
// as noise or as a permanently framing-errored stream.
//
//   pio run -e xiao_uart_echo -t upload
//   pio device monitor -e xiao_uart_echo
//
// This sketch does two things at once, so one flash tests both directions:
//
//   XIAO -> Teensy   transmits "XIAO_PING <n>\n" on Serial1 once a second.
//                    Watch for it on the Teensy's own USB monitor.
//   Teensy -> XIAO   dumps everything arriving on Serial1 to the XIAO's USB
//                    monitor, as hex AND as printable ASCII.
//
// WHY HEX AND NOT JUST TEXT.  A baud-rate mismatch does not produce
// silence, it produces plausible-looking wrong characters.  Seeing the raw
// bytes is what distinguishes "wrong baud" (consistent garbage, often
// 0xFF/0x00-heavy) from "wrong pin" (nothing at all) from "floating RX"
// (random bytes only when you touch the wire).
//
// BAUD RATE.  Defaults to the real telemetry rate, not 115200 -- see the
// arithmetic in xiao_bridge/main.cpp.  Testing the link at a rate you will
// not actually use proves very little, because framing errors appear at
// high baud and not at low.  Override at build time if you want to bisect:
//
//   pio run -e xiao_uart_echo -t upload --build-flag "-DUART_BAUD=115200"

#include <Arduino.h>

namespace {

#ifndef UART_BAUD
#define UART_BAUD 921600
#endif

// XIAO ESP32-C6 silkscreen pins for the secondary UART.
constexpr int kUartRxPin = D7;
constexpr int kUartTxPin = D6;

constexpr uint32_t kPingPeriodMs = 1000;

// One line of hex output per 16 bytes, classic hexdump width.
constexpr size_t kDumpWidth = 16;

uint32_t g_rx_bytes = 0;
uint32_t g_tx_pings = 0;

// Buffered so a burst arriving mid-line still prints as one tidy row.
uint8_t g_line[kDumpWidth];
size_t g_line_len = 0;

void flushLine() {
  if (g_line_len == 0) { return; }

  Serial.printf("rx %6lu | ", static_cast<unsigned long>(g_rx_bytes));
  for (size_t i = 0; i < kDumpWidth; i++) {
    if (i < g_line_len) {
      Serial.printf("%02X ", g_line[i]);
    } else {
      Serial.print("   ");
    }
  }
  Serial.print("| ");
  for (size_t i = 0; i < g_line_len; i++) {
    const uint8_t b = g_line[i];
    Serial.write((b >= 0x20 && b < 0x7F) ? static_cast<char>(b) : '.');
  }
  Serial.println();

  g_line_len = 0;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  // setRxBufferSize MUST precede begin() -- afterwards it is silently
  // ignored and you keep the 256-byte default, which overruns in
  // milliseconds at 921600 baud and looks exactly like a bad solder joint.
  Serial1.setRxBufferSize(4096);
  Serial1.begin(UART_BAUD, SERIAL_8N1, kUartRxPin, kUartTxPin);

  Serial.println();
  Serial.println("=== xiao_uart_echo ===");
  Serial.printf("baud           %d\n", static_cast<int>(UART_BAUD));
  Serial.printf("RX pin         D7 (GPIO%d)  <- Teensy pin 1 (TX1)\n",
                kUartRxPin);
  Serial.printf("TX pin         D6 (GPIO%d)  -> Teensy pin 0 (RX1)\n",
                kUartTxPin);
  Serial.println("sending XIAO_PING on Serial1 once a second");
  Serial.println("dumping everything received on Serial1 below");
  Serial.println();
}

void loop() {
  static uint32_t next_ping_ms = 0;
  static uint32_t last_rx_ms = 0;

  const uint32_t now = millis();

  if (static_cast<int32_t>(now - next_ping_ms) >= 0) {
    next_ping_ms = now + kPingPeriodMs;
    Serial1.printf("XIAO_PING %lu\n", static_cast<unsigned long>(++g_tx_pings));
  }

  while (Serial1.available() > 0) {
    const int value = Serial1.read();
    if (value < 0) { break; }
    g_rx_bytes++;
    last_rx_ms = now;
    g_line[g_line_len++] = static_cast<uint8_t>(value);
    if (g_line_len == kDumpWidth) { flushLine(); }
  }

  // Flush a partial line once the stream goes quiet, so a short message
  // like "ping\n" appears immediately instead of waiting for 16 bytes that
  // may never arrive.
  if (g_line_len > 0 && (now - last_rx_ms) > 20) { flushLine(); }

  // Heartbeat when nothing at all is arriving, so a silent link is
  // distinguishable from a hung sketch.
  static uint32_t next_idle_ms = 0;
  if (g_rx_bytes == 0 && static_cast<int32_t>(now - next_idle_ms) >= 0) {
    next_idle_ms = now + 2000;
    Serial.printf("... no bytes on Serial1 yet (%lu pings sent)\n",
                  static_cast<unsigned long>(g_tx_pings));
  }
}
