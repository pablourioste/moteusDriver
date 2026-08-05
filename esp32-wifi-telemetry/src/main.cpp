#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp;
constexpr uint16_t kControlPort   = 5005;
constexpr uint16_t kTelemetryPort = 5006;
IPAddress laptopIP;
bool laptopKnown = false;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial1.begin(115200, SERIAL_8N1, /*RX=*/D7, /*TX=*/D6);

  WiFi.softAP("CubliNet", "cubli1234");
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  udp.begin(kControlPort);
  Serial.println("UDP relay ready");
}

void loop() {
  int len = udp.parsePacket();
  if (len > 0) {
    uint8_t buf[64];
    int n = udp.read(buf, sizeof(buf));
    laptopIP = udp.remoteIP();
    laptopKnown = true;

    Serial.printf("Got %d bytes from %s: ", n, laptopIP.toString().c_str());
    for (int i = 0; i < n; i++) Serial.printf("%02X ", buf[i]);
    Serial.println();

    Serial1.write(buf, n);   // still forwards to Teensy, even with nothing listening yet
  }

  if (Serial1.available()) {
    uint8_t buf[64];
    int n = Serial1.readBytes(buf, min((int)Serial1.available(), 64));
    if (laptopKnown) {
      udp.beginPacket(laptopIP, kTelemetryPort);
      udp.write(buf, n);
      udp.endPacket();
    }
  }
}
