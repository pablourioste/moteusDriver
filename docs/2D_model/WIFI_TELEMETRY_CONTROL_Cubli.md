# WiFi Telemetry & Control — XIAO ESP32-C6 ↔ Teensy 4.1 ↔ Laptop

**Hands-on procedure.** Bring up each link separately, then join them end to end.

**Target architecture:**

```
 ┌────────────┐   UART (D6/D7)   ┌────────────────┐   WiFi / UDP   ┌──────────┐
 │  Teensy 4.1│ ───────────────► │ XIAO ESP32-C6  │ ─────────────► │  Laptop  │
 │ (IMU, CAN, │ ◄─────────────── │ (WiFi bridge)  │ ◄───────────── │ (Python) │
 │  motor ctrl│   telemetry out  │                │  control in    │          │
 └────────────┘   control in     └────────────────┘                └──────────┘
```

The Teensy owns the real-time control loop and never touches WiFi directly. The
ESP32-C6 is a **dumb bridge**: bytes in from UART go out over WiFi, bytes in from WiFi
go out over UART. All the "meaning" (what a telemetry frame contains, what a command
means) lives in the Teensy firmware and the laptop script — the bridge doesn't need to
understand it.

---

## Before you start

| Need | Note |
|---|---|
| XIAO ESP32-C6 with antenna | From your BOM |
| Teensy 4.1, already running from the IMU guide | This builds on that project |
| 3 jumper wires | TX, RX, GND between the two boards — **no level shifter needed**, both are 3.3 V logic |
| Laptop with WiFi | Will connect directly to the cube's own network |

---

# STEP 0 — Environment: ESP32-C6 project in PlatformIO

## 0.1 — New environment, same repo

Add a second environment to your existing `platformio.ini` (or a separate project —
either works, since these are two independent MCUs):

```ini
[env:esp32c6]
platform = espressif32
board = seeed_xiao_esp32c6
framework = arduino
monitor_speed = 115200
upload_speed = 921600
build_flags =
    -DARDUINO_USB_CDC_ON_BOOT=1
```

**If `pio run -e esp32c6` can't find the board** (this board definition has landed in
PlatformIO's official `espressif32` platform only fairly recently — older installs
don't have it), fall back to the community fork that's known to carry it:

```ini
[env:esp32c6]
platform = https://github.com/pioarduino/platform-espressif32.git#develop
board = seeed_xiao_esp32c6
framework = arduino
```

## 0.2 — Flashing: probably simpler than the Teensy was

The Teensy needed a WSL/Windows split because flashing swaps it to a completely
different USB device (HalfKay bootloader). The ESP32-C6 doesn't do that — it flashes
over its **native USB** (no separate USB-serial chip) and generally keeps the same
device identity across a reset. In practice that means:

```bash
pio run -e esp32c6 -t upload      # try this directly from WSL first
```

**If the port drops on every upload the way the Teensy's did**, fall back to the same
pattern as before — build in WSL, flash from Windows. Test it once and you'll know
which camp you're in.

**If it's never detected at all:** hold the **BOOT** button while plugging in USB, then
release once the upload starts — this forces the bootloader manually, same idea as the
Teensy's program button.

## 0.3 — Prove the toolchain first

```cpp
#include <Arduino.h>
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  Serial.println("ESP32-C6 alive");
}
void loop() { Serial.println("tick"); delay(1000); }
```

### ✅ Check 0
- [ ] Builds and flashes
- [ ] `pio device monitor` prints `ESP32-C6 alive` then `tick` once/sec

---

# STEP 1 — Wire the UART bridge

The XIAO ESP32-C6's hardware UART0 is fixed to two specific pins — **D6 is TX, D7 is
RX**. Cross-connect them to the Teensy's `Serial1`:

```
   XIAO ESP32-C6                    Teensy 4.1
  ┌──────────────┐                ┌──────────────────┐
  │  GND ────────┼────────────────┤ GND              │
  │  D6 (TX) ────┼────────────────┤ 0   (RX1)        │
  │  D7 (RX) ────┼────────────────┤ 1   (TX1)        │
  └──────────────┘                └──────────────────┘
```

Both boards run 3.3 V logic, so this is a direct wire — no level shifter, no divider.

### ✅ Check 1 — loopback sanity, before either side does anything smart
On the ESP32-C6:
```cpp
Serial1.begin(115200, SERIAL_8N1, /*RX=*/D7, /*TX=*/D6);
```
On the Teensy:
```cpp
Serial1.begin(115200);
```
Have the Teensy print `"ping"` once a second on `Serial1`, and have the ESP32-C6 echo
anything it receives on `Serial1` out to its own USB `Serial` monitor.
- [ ] `pio device monitor` on the ESP32-C6 shows `ping` once a second

**Do not continue until this passes** — everything below assumes this link works.

---

# STEP 2 — Bring up WiFi (SoftAP)

## Why SoftAP over joining an existing network

Two options exist: **station mode** (ESP32 joins an existing WiFi network) or **SoftAP**
(ESP32 *creates* its own network, laptop joins it). For a competition/field setting,
SoftAP is the more robust default — it doesn't depend on venue WiFi being available,
stable, or letting your devices talk to each other (many venue/conference networks
block device-to-device traffic). The trade-off is shorter range and effectively one
client at a time, which is fine for a single laptop controlling one cube.

```cpp
#include <Arduino.h>
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  WiFi.softAP("CubliNet", "cubli1234");   // pick your own SSID/password
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());        // will be 192.168.4.1 by default
}
void loop() {}
```

### ✅ Check 2
- [ ] `CubliNet` shows up in the laptop's WiFi list
- [ ] Laptop connects to it
- [ ] `ping 192.168.4.1` from the laptop gets replies

---

# STEP 3 — UDP relay on the ESP32-C6

**Design choice: UDP, not TCP**, for both directions. Telemetry can tolerate an
occasional dropped packet (the next one arrives in milliseconds); a TCP handshake and
retransmission stall would actually hurt a real-time control link more than a lost
packet would. Control commands ride UDP too, but see Step 6 for how to make dropped
*commands* safe rather than just ignored.

**Design choice: don't hardcode the laptop's IP.** Have the ESP32-C6 remember whoever
last sent it a control packet, and send telemetry back to that address. This survives
the laptop reconnecting with a different IP without you editing firmware.

```cpp
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp;
constexpr uint16_t kControlPort   = 5005;   // laptop -> cube
constexpr uint16_t kTelemetryPort = 5006;   // cube -> laptop
IPAddress laptopIP;
bool laptopKnown = false;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, /*RX=*/D7, /*TX=*/D6);
  WiFi.softAP("CubliNet", "cubli1234");
  udp.begin(kControlPort);
}

void loop() {
  // WiFi -> UART (control commands in)
  int len = udp.parsePacket();
  if (len > 0) {
    uint8_t buf[64];
    int n = udp.read(buf, sizeof(buf));
    laptopIP = udp.remoteIP();
    laptopKnown = true;
    Serial1.write(buf, n);
  }

  // UART -> WiFi (telemetry out)
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
```

> **Why this alone isn't quite enough:** UART is a byte stream with no concept of
> "packet boundaries." This loop forwards whatever bytes happened to arrive since the
> last iteration — which might be half a telemetry frame, or two and a half frames.
> Fixing that is Step 4: a framing format that both the Teensy and the laptop parse
> the same way, so fragments and re-joins never lose sync.

### ✅ Check 3
- [ ] Sending any UDP packet to `192.168.4.1:5005` from the laptop shows up on the
      Teensy's `Serial1` (use `nc -u 192.168.4.1 5005` from WSL, or Python's `socket`)
- [ ] Bytes the Teensy prints on `Serial1` arrive at the laptop on UDP port 5006

---

# STEP 4 — A framing format that survives arbitrary chunking

Simple and enough for this project:

```
[0xAA] [LEN] [PAYLOAD...LEN bytes...] [CHECKSUM]
```

`0xAA` is a sync byte, `LEN` is the payload length, `CHECKSUM` is a sum-of-bytes check.
Both ends buffer incoming bytes and only act once a complete, checksum-valid frame is
assembled — never assume "one read = one frame."

**Teensy side** — send telemetry, e.g. 6 floats (accel XYZ, gyro XYZ):
```cpp
void sendTelemetry(const float vals[6]) {
  uint8_t payload[24];
  memcpy(payload, vals, sizeof(payload));
  uint8_t checksum = 0;
  for (uint8_t b : payload) checksum += b;

  Serial1.write(0xAA);
  Serial1.write(sizeof(payload));
  Serial1.write(payload, sizeof(payload));
  Serial1.write(checksum);
}
```

Parse incoming control frames with a small state machine (sync → length → payload →
checksum), rather than trusting `Serial1.available()` counts to line up with frame
boundaries.

**Laptop side (Python)** — same framing, both directions:
```python
import socket, struct, threading, time

CUBE_IP = "192.168.4.1"
CONTROL_PORT = 5005
TELEMETRY_PORT = 5006

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", TELEMETRY_PORT))
sock.settimeout(1.0)

def send_command(target_angle_deg: float):
    payload = struct.pack("<f", target_angle_deg)
    checksum = sum(payload) & 0xFF
    frame = bytes([0xAA, len(payload)]) + payload + bytes([checksum])
    sock.sendto(frame, (CUBE_IP, CONTROL_PORT))

def telemetry_loop():
    buf = bytearray()
    while True:
        try:
            data, _ = sock.recvfrom(512)
        except socket.timeout:
            continue
        buf += data
        while len(buf) >= 2 and buf[0] == 0xAA:
            length = buf[1]
            if len(buf) < 2 + length + 1:
                break                          # wait for more bytes
            payload = buf[2:2 + length]
            checksum = buf[2 + length]
            if (sum(payload) & 0xFF) == checksum:
                ax, ay, az, gx, gy, gz = struct.unpack("<6f", payload)
                print(f"a=({ax:.2f},{ay:.2f},{az:.2f}) g=({gx:.2f},{gy:.2f},{gz:.2f})")
            buf = buf[2 + length + 1:]
        if buf and buf[0] != 0xAA:
            buf.pop(0)                         # resync on garbage

threading.Thread(target=telemetry_loop, daemon=True).start()

while True:
    send_command(0.0)          # replace with real control input
    time.sleep(0.1)
```

### ✅ Check 4
- [ ] Laptop console prints clean, correctly-parsed telemetry values continuously
- [ ] Deliberately send a truncated/garbage UDP packet — the parser resyncs instead of
      getting stuck (this is what the checksum + resync loop is for)

---

# STEP 5 — Control commands: make dropped packets safe, not just ignored

UDP does not guarantee delivery or order. For **telemetry** that's fine — a missed
sample is invisible a moment later. For **control commands**, design so that a lost
packet degrades gracefully rather than corrupting state:

- **Send absolute targets, not deltas.** `"set target angle = 3.2°"` survives a dropped
  packet — the next one just repeats the same instruction. `"increase angle by 0.1°"`
  does not — losses accumulate into silent drift.
- **Repeat the current command continuously** (e.g. every 100 ms) rather than only on
  change. A single lost "stop" command is much less dangerous if the next repeat
  arrives 100 ms later than if it was the only one ever sent.

---

# STEP 6 — The failure case that matters most: WiFi drops mid-control

If the laptop's link drops while the cube is actively balancing, the Teensy must not
keep coasting on the last command it heard forever. This is a **watchdog**, and it goes
on the Teensy side — it must not depend on the WiFi link being alive to trigger.

```cpp
uint32_t lastCommandMs = 0;
constexpr uint32_t kCommandTimeoutMs = 500;

void loop() {
  // ... parse incoming frames, update lastCommandMs on each valid one ...

  if (millis() - lastCommandMs > kCommandTimeoutMs) {
    // Fail-safe: zero the control output / cut motor power.
    applySafeState();
  }
}
```

### ✅ Check 6
- [ ] With the cube actively receiving commands, unplug the laptop's WiFi (or walk out
      of range) — motors go to a safe state within ~500 ms, not indefinitely
- [ ] Reconnecting resumes normal control without a manual reset

---

# STEP 7 — Soak test

Run the full pipeline for 10+ minutes with the laptop script logging:
- round-trip latency (timestamp a command, measure when its effect shows up in
  telemetry)
- packet loss rate on each direction
- any SoftAP client disconnects, and whether telemetry + control both resume cleanly
  afterward without a firmware reset

### ✅ Check 7
- [ ] No unexplained latency spikes
- [ ] Loss rate low enough that the watchdog in Step 6 doesn't trip during normal
      operation, but reliably does trip when the link is actually cut

---

# Final checklist

| Step | Check | ✅ |
|---|---|---|
| 0 | ESP32-C6 toolchain builds, flashes, prints serial | ☐ |
| 1 | UART loopback: Teensy's `ping` visible on ESP32-C6 monitor | ☐ |
| 2 | `CubliNet` visible, laptop connects, `192.168.4.1` pingable | ☐ |
| 3 | Raw UDP bytes cross in both directions | ☐ |
| 4 | Framed telemetry parses cleanly; parser resyncs after garbage | ☐ |
| 5 | Control commands are absolute + repeated, not deltas | ☐ |
| 6 | Watchdog cuts motors within ~500 ms of link loss | ☐ |
| 7 | 10-minute soak: latency stable, loss rate acceptable | ☐ |

With this in place, your side of the team split — telemetry out, control in, over WiFi
— is ready to plug into whatever the balancing control loop and 3D-simulation
counterpart need from it.

---

## Quick reference

| Item | Value |
|---|---|
| ESP32-C6 UART0 pins | TX = D6 (GPIO16), RX = D7 (GPIO17) |
| PlatformIO board id | `seeed_xiao_esp32c6` (platform `espressif32`; use the `pioarduino` fork if not found) |
| Default SoftAP IP | `192.168.4.1` |
| Control port | 5005 (laptop → cube) |
| Telemetry port | 5006 (cube → laptop) |
| Frame format | `0xAA, LEN, payload[LEN], checksum` |
| Command timeout / watchdog | 500 ms (tune to your control loop's tolerance) |
