// Step X2 -- does the radio come up, and do UDP datagrams cross it?
//
// No UART, no Teensy, no telemetry format.  This isolates the wireless
// link so that when the full bridge later drops packets you already know
// whether the radio was ever healthy on its own.
//
//   pio run -e xiao_softap -t upload
//   pio device monitor -e xiao_softap
//
// SOFTAP, NOT STATION MODE.  The cube creates its own network rather than
// joining one.  A venue's WiFi may be absent, congested, or configured to
// block client-to-client traffic (very common on conference and campus
// networks), any of which kills the control link for reasons nothing in
// this repository can fix.  The cost is range and effectively one client,
// which is exactly the intended deployment: one laptop, one cube.
//
// WHAT TO DO WITH IT, from the laptop:
//
//   1. join the WiFi network "CubliNet", password "cubli1234"
//   2. ping 192.168.4.1
//   3. echo hello | nc -u -w1 192.168.4.1 5005
//
// Step 3 echoes the payload straight back to the sender's IP on port 5006,
// so a single command exercises both directions.  Listen with:
//
//   nc -u -l 5006
//
// WIFI POWER SAVE IS DISABLED HERE, AND THAT MATTERS.  The ESP32 defaults
// to a modem sleep that parks the radio between beacon intervals; incoming
// datagrams then queue until the next wake, adding jitter measured in tens
// to hundreds of milliseconds.  On a logging link that is a curiosity; on
// a link carrying control commands to a balancing cube it is the
// difference between a stable loop and an unexplained one.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

namespace {

#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID "CubliNet"
#endif
#ifndef WIFI_AP_PASSWORD
#define WIFI_AP_PASSWORD "cubli1234"
#endif

// Laptop -> cube.  Also the port the ESP32 binds and therefore the source
// port of everything it sends.
constexpr uint16_t kControlPort = 5005;
// Cube -> laptop.
constexpr uint16_t kTelemetryPort = 5006;

// One datagram, generously sized.  Larger than any control frame; the
// telemetry direction never passes through this sketch.
constexpr size_t kBufferSize = 1024;

WiFiUDP udp;
uint8_t g_buffer[kBufferSize];

uint32_t g_rx_packets = 0;
uint32_t g_tx_packets = 0;
uint8_t g_last_clients = 0;

}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println();
  Serial.println("=== xiao_softap ===");

  WiFi.mode(WIFI_AP);
  const bool ok = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
  // See the header note: this is a latency fix, not a power decision.
  WiFi.setSleep(false);

  if (!ok) {
    // Nothing downstream can work, so say so loudly and forever rather
    // than proceeding into a loop that silently receives nothing.
    while (true) {
      Serial.println("FAIL softAP() returned false -- check antenna/board");
      delay(1000);
    }
  }

  Serial.printf("ssid           %s\n", WIFI_AP_SSID);
  Serial.printf("password       %s\n", WIFI_AP_PASSWORD);
  Serial.print("ap ip          ");
  Serial.println(WiFi.softAPIP());
  Serial.printf("control port   %u  (laptop -> cube)\n", kControlPort);
  Serial.printf("telemetry port %u  (cube -> laptop)\n", kTelemetryPort);
  Serial.printf("channel        %d\n", static_cast<int>(WiFi.channel()));

  udp.begin(kControlPort);
  Serial.println("udp echo ready -- payloads bounce back on the telemetry port");
  Serial.println();
}

void loop() {
  const int len = udp.parsePacket();
  if (len > 0) {
    const int n = udp.read(g_buffer, sizeof(g_buffer));
    const IPAddress from = udp.remoteIP();
    g_rx_packets++;

    Serial.printf("rx #%lu  %d bytes from %s:%u  | ",
                  static_cast<unsigned long>(g_rx_packets), n,
                  from.toString().c_str(), udp.remotePort());
    for (int i = 0; i < n && i < 32; i++) { Serial.printf("%02X ", g_buffer[i]); }
    if (n > 32) { Serial.print("..."); }
    Serial.println();

    // Echo to the SENDER's address, never a hardcoded one.  The laptop's
    // DHCP lease changes across reconnects; learning the address from the
    // packet is what keeps that from being a firmware edit.
    if (n > 0) {
      udp.beginPacket(from, kTelemetryPort);
      udp.write(g_buffer, static_cast<size_t>(n));
      udp.endPacket();
      g_tx_packets++;
    }
  }

  // Client join/leave is the single most useful diagnostic here: a laptop
  // that "connected" in the OS but never appears in this count is on a
  // different network, or fell back to a saved one.
  const uint8_t clients = WiFi.softAPgetStationNum();
  if (clients != g_last_clients) {
    Serial.printf("clients        %u -> %u\n", g_last_clients, clients);
    g_last_clients = clients;
  }

  static uint32_t next_status_ms = 0;
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - next_status_ms) >= 0) {
    next_status_ms = now + 5000;
    Serial.printf("status         up %lus  clients %u  rx %lu  tx %lu  heap %lu\n",
                  static_cast<unsigned long>(now / 1000), clients,
                  static_cast<unsigned long>(g_rx_packets),
                  static_cast<unsigned long>(g_tx_packets),
                  static_cast<unsigned long>(ESP.getFreeHeap()));
  }
}
