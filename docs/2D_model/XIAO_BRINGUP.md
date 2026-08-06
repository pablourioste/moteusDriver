# XIAO ESP32-C6 bring-up — the runnable procedure

`WIFI_TELEMETRY_CONTROL_Cubli.md` is the teaching guide: it explains *why*
SoftAP over station mode, *why* UDP over TCP, *why* absolute setpoints. This
file is the executable companion — the firmware that exists in this repository,
the exact commands, and what each check proves.

Read the other one first if you have not. Read this one with the boards in front
of you.

---

## What lives where

```
firmware/xiao/
  xiao_board_check/     X0  is the Xiao alive?            no wiring
  xiao_uart_echo/       X1  does the UART link work?      3 wires to the Teensy
  xiao_softap/          X2  does the radio work?          no wiring
  xiao_frame_source/    X3  does WiFi carry real frames?  no wiring
  xiao_bridge/          X4  the bridge itself             3 wires + laptop
tools/telemetry/
  wifi_link_test.py         loss / jitter / verdict for the WiFi hop
  capture.py --udp          the same capture tool, over UDP instead of COM
```

Environment names are flat, matching every other sketch in this repo:
`pio run -e xiao_bridge`, not `-e xiao/xiao_bridge`.

---

## Two decisions that differ from the teaching guide

Both are corrections, not preferences. They are repeated in the source headers
so nobody has to remember them.

### 1. The UART runs at 921600, not 115200

```
118 bytes/frame × 200 Hz               = 23 600 bytes/s
× 10 bits/byte (8N1: start + 8 + stop) = 236 000 baud MINIMUM
```

115200 baud carries 11 520 bytes/s — **97 frames/s, under half the balancer's
rate**, and a quarter of what `telemetry_test` produces at 400 Hz. It does not
fail cleanly: the Teensy's TX blocks, or the Xiao's RX FIFO overruns, and you
get a stream that decodes for a few seconds and then corrupts under load.

921600 gives 92 160 bytes/s — ~3.9× headroom at 200 Hz, ~2× at 400 Hz.

**Both ends must agree.** A baud mismatch produces plausible garbage, not
silence, which is why X1 hexdumps the bytes instead of printing them as text.

### 2. No `[0xAA][LEN][payload][checksum]` envelope

The teaching guide's Step 4 adds a framing layer at the bridge. **Do not.** The
payload is already `cube::TelemetryFrame` (`include/core/TelemetryFrame.hpp`):
0xA5C3 magic, explicit length, version byte, and a CRC-16/CCITT over the whole
record. `tools/telemetry/decode.py` already resynchronises on that magic one
byte at a time, and `capture.py` already discards datagram boundaries on
purpose.

Wrapping it costs three things and buys none: the bridge stops being
format-agnostic (so a new frame version means reflashing the radio), a
sum-of-bytes checksum is strictly weaker than the CRC it duplicates, and the
host would need a second parser in front of the one that already works.

The bridge is a **dumb byte pipe**. That is the contract in
`include/core/TelemetrySink.hpp`, and keeping it is what lets the wire format
evolve without touching this firmware.

---

## ⚠ WSL2 cannot receive the telemetry — run the receivers from Windows

This rig's WSL2 is in the default **NAT** networking mode (`eth0` on
172.27.x.x, no `.wslconfig`). That breaks the inbound half of the link, and it
breaks it silently:

- Outbound works. A packet from WSL to `192.168.4.1` is SNAT'd through Windows
  and arrives fine.
- **The bridge therefore learns the WINDOWS host's address**, not WSL's — the
  source IP it sees is `192.168.4.x`, Windows' address on `CubliNet`.
- Telemetry is sent to `192.168.4.x:5006`, which arrives at **Windows**. WSL
  never sees it. `capture.py --udp` sits there reporting nothing, with no error.

`netsh portproxy` does **not** fix this — it forwards TCP only, and this link is
UDP by design.

So the split is the same one the COM path already uses:

| Job | Where | Why |
|---|---|---|
| `wifi_link_test.py`, `capture.py --udp` | **Windows** (`python.exe`) | must receive inbound UDP |
| `decode.py`, `analyze.ipynb` | **WSL** (`moteus-venv`) | needs pandas + pyarrow |
| `pio run`, `pio device monitor` | **WSL** | unchanged |

`wifi_link_test.py` is deliberately **stdlib-only** so it runs under a bare
Windows Python with nothing installed. `capture.py --udp` is too — its pyserial
dependency is only on the `--port` path.

The `.bin` lands on the shared filesystem either way, so capture on Windows and
decode in WSL needs no copying.

> The docstring in `capture.py` says the UDP path "runs in WSL like everything
> else". That is true only with WSL2 **mirrored** networking
> (`networkingMode=mirrored` in `%USERPROFILE%\.wslconfig`, Windows 11 22H2+),
> which is not enabled here. Either enable it or use `python.exe`.

## The USB/COM path is untouched

Everything here is additive. `[env:telemetry_test]` — the sketch behind the
working `capture.py --port COM9` capture — compiles to a **bit-for-bit
identical binary** before and after this work; the alternate UART sink is
`#ifdef`'d out of it entirely and lives in a separate env,
`[env:telemetry_test_uart]`.

To go back to USB at any point, flash `-e telemetry_test`. Nothing to undo.

---

# X0 — is the Xiao alive?

No wiring. This separates "my build/flash pipeline is broken" from "my circuit
is broken" before anything is attached.

```bash
pio run -e xiao_board_check -t upload
pio device monitor -e xiao_board_check
```

**Do not pass `--upload-port`, and do not set `upload_port` in the ini.**
Autodetect is already identity-based: `seeed_xiao_esp32c6.json` declares
`build.hwids` of `0x2886:0x0046` and `0x303A:0x1001`, and PlatformIO matches
serial devices against those. The Xiao is selected by VID:PID, so it is immune
to `/dev/ttyACM*` renumbering and cannot pick the Teensy (`16C0:0483`) even with
both boards attached.

Overriding it makes things worse, which is worth knowing because the failure is
misleading. `/dev/ttyACM*` numbers are assignment order, not identity, so a
literal number is wrong about half the time — and a `/dev/serial/by-id/...*`
glob fails too: PlatformIO does not expand wildcards against the filesystem. It
fnmatches the pattern against `/dev/ttyACM0`-style names only
(`platformio/device/finder.py`), matches nothing, and hands the unexpanded
string to esptool, which reports `the port is busy or doesn't exist`.

If you genuinely must force it, use the exact by-id path with **no** wildcard —
without a `*` it is not treated as a pattern and pyserial opens the symlink
directly.

Unlike the Teensy, this normally flashes **directly from WSL**. The Teensy
reboots into its HalfKay bootloader on upload and enumerates as a different USB
device, dropping the usbipd attachment every time; the ESP32-C6 flashes over
native USB and keeps its identity.

If the port is not visible in WSL at all, attach it from an Administrator
PowerShell first:

```powershell
usbipd list
usbipd attach --wsl --busid <BUSID>
```

**Check X0**
- [ ] `chip ESP32-C6 rev N`, core count, flash size and free heap all print
- [ ] `tick N  uptime …  heap …` appears once a second
- [ ] `reset reason  POWERON` (`BROWNOUT` means the USB supply is sagging —
      try a different cable or port before believing anything else)

If the port never appears: hold **BOOT** while plugging in USB, release once the
upload starts. If uploads keep dropping the port, fall back to the Teensy
pattern — build in WSL, flash from Windows.

---

# X1 — does the UART link to the Teensy work?

> **Testing the radio only?** X2 and X3 need no wiring at all and can be run
> straight after X0. Do that first if the Xiao is still on the bench — it
> proves the module end to end before a single wire exists to be blamed. Come
> back here when you are ready to join the two boards.

Three wires. Both boards are 3.3 V logic, so this is direct — no level shifter,
no divider.

```
   XIAO ESP32-C6                    Teensy 4.1
  ┌──────────────┐                ┌──────────────────┐
  │  GND ────────┼────────────────┤ GND              │
  │  D6 (TX) ────┼────────────────┤ 0   (RX1)        │
  │  D7 (RX) ────┼────────────────┤ 1   (TX1)        │
  └──────────────┘                └──────────────────┘
```

**The GND wire is not optional.** Two boards on separate USB ports can sit at
different ground potentials; without the common return the UART reads as noise
or as a permanently framing-errored stream.

```bash
pio run -e xiao_uart_echo -t upload
pio device monitor -e xiao_uart_echo
```

One flash tests both directions: the Xiao transmits `XIAO_PING <n>` on `Serial1`
once a second, and hexdumps everything it receives.

For the Teensy half, flash any sketch that writes to `Serial1` at 921600.
`-e telemetry_test_uart` works and is what you will use anyway — it emits binary,
so expect the hexdump to show `C3 A5 76 01 …` (the frame magic, little-endian)
every 118 bytes rather than readable text.

**Check X1**
- [ ] Xiao's monitor shows bytes arriving, at the right rate
- [ ] The Teensy's own monitor shows `XIAO_PING` once a second
- [ ] The hex is *structured*, not random — for telemetry frames you should see
      `C3 A5` recurring on a 118-byte pitch

Diagnosing the hexdump:

| What you see | What it means |
|---|---|
| Nothing at all | Wrong pin, or the Teensy is not transmitting |
| Consistent garbage, no `C3 A5` | Baud mismatch — check both `begin()` calls |
| Random bytes only when you touch the wire | RX is floating; check GND and the RX wire |
| `C3 A5` present but at an irregular pitch | Bytes are being lost — RX buffer or baud |

To bisect a bad link, drop both ends to a slow rate:

```bash
pio run -e xiao_uart_echo -t upload --build-flag "-DUART_BAUD=115200"
```

**Do not continue until this passes.** Everything below assumes the link works.

---

# X2 — does the radio work on its own?

No UART, no Teensy. This proves the wireless link healthy *before* it can be
blamed for anything.

```bash
pio run -e xiao_softap -t upload
pio device monitor -e xiao_softap
```

Then, on the laptop: join the WiFi network **`CubliNet`**, password
**`cubli1234`**.

```bash
ping 192.168.4.1                        # basic reachability
nc -u -l 5006 &                         # listen for the echo
echo hello | nc -u -w1 192.168.4.1 5005 # send
```

The sketch echoes any payload straight back to the sender's IP on port 5006, so
that one command exercises both directions.

**Check X2**
- [ ] `CubliNet` appears in the laptop's WiFi list
- [ ] `ping 192.168.4.1` replies
- [ ] The monitor's `clients` count goes `0 -> 1` when the laptop joins
- [ ] `hello` comes back on port 5006

The `clients` count is the diagnostic that matters. A laptop that shows
"connected" in the OS but never appears here has silently fallen back to a saved
network — a very common half-hour of wasted debugging.

---

# X3 — does WiFi carry *real* telemetry frames?

Still no Teensy, no UART, no IMU. The Xiao generates valid 118-byte
`cube::TelemetryFrame` records itself and sends them over UDP.

This is the highest-value test here. The full chain is IMU → Teensy → UART →
Xiao → WiFi → laptop; when the first end-to-end run shows corrupt frames there
are five candidates. This removes four of them. **Anything the host fails to
decode here was mangled by the radio and by nothing else.**

It is simultaneously the acceptance test for the host tooling — `capture.py`'s
`--udp` path, `decode.py`'s CRC and sequence accounting, and the raw-vs-SI scale
cross-check — with not a single wire attached.

```bash
pio run -e xiao_frame_source -t upload
```

Join `CubliNet`, then, in three terminals:

```bash
# 1. tell the Xiao where you are (it has no hardcoded laptop IP)
python.exe tools/telemetry/wifi_link_test.py --announce

# 2. measure the link and get a verdict
python.exe tools/telemetry/wifi_link_test.py -s 60

# 3. or capture to disk with the normal tool, over UDP instead of COM
python.exe tools/telemetry/capture.py \
    --udp 0.0.0.0:5006 --out logs/wifi_synth.bin -s 60 --live
./moteus-venv/bin/python tools/telemetry/decode.py logs/wifi_synth.bin
```

`wifi_link_test.py` sends probe packets at 10 Hz while it listens, so the
control direction is under load the whole time it measures the telemetry
direction.

**Check X3**
- [ ] `PASS` from `wifi_link_test.py`
- [ ] Frame rate ≈ 200 Hz, throughput ≈ 23.6 kB/s
- [ ] `CRC failures 0` — at this stage there is no UART, so a CRC failure means
      real radio corruption and deserves investigation before you go further
- [ ] `frames lost` under 1 %
- [ ] `frames/datagram` shows `8x…` — batching is working
- [ ] `datagram gap max` well under 250 ms — a 100 ms+ stall means WiFi power
      save is still on somewhere

Note the asymmetry, because it is how you will diagnose the full chain later:

> **A dropped datagram costs whole frames and never fails a CRC.**
> **A CRC failure is corruption, not loss.**

So once the UART is in the picture, CRC failures point at the UART (baud
mismatch, RX overrun) and sequence gaps point at the radio.

Bandwidth headroom check at the IMU's full ODR:

```bash
pio run -e xiao_frame_source -t upload --build-flag "-DFRAME_RATE_HZ=400"
```

---

# X4 — the bridge

Now the real thing. Flash the Xiao, then the Teensy.

```bash
# Xiao: the bridge
pio run -e xiao_bridge -t upload

# Teensy: same telemetry sketch, sink swapped from USB CDC to Serial1
pio run -e telemetry_test_uart          # then flash from Windows
```

`telemetry_test_uart` is the *same source file* as `telemetry_test` plus
`-D TELEMETRY_SINK_UART1`. Only the `ITelemetrySink` implementation differs —
same frames, same ring, same drain policy. That one-line swap is the entire
reason `TelemetrySink.hpp` exists.

Keep the Xiao's USB monitor open. It prints a status line every second, and it
is safe to do so because the USB port and the bridged UART are different
devices — the text can never contaminate the binary stream:

```
up 42s  clients 1  peer 192.168.4.2  link_age 98ms | uart_rx 991200  udp_tx 991200 B / 8400 pkt  fail 0 | udp_rx 4200 B / 420 pkt  uart_tx 4200  drop 0/0 | heap 210344
```

Then from the laptop:

```bash
python.exe tools/telemetry/wifi_link_test.py -s 60
python.exe tools/telemetry/capture.py \
    --udp 0.0.0.0:5006 --out logs/wifi_full.bin -s 120
./moteus-venv/bin/python tools/telemetry/decode.py logs/wifi_full.bin --parquet
```

**Check X4**
- [ ] `uart_rx` climbs at ~47 kB/s (400 Hz × 118 B)
- [ ] `peer` is the laptop's address, not `none`
- [ ] `udp_tx` bytes track `uart_rx` bytes closely — a growing gap is loss at
      the radio, not at the UART
- [ ] `fail 0` and `drop 0/0`
- [ ] `wifi_link_test.py` reports `PASS`
- [ ] `decode.py` reports 0 CRC failures and no version skew
- [ ] `heap` is flat over minutes — a falling heap is a leak, and it will crash
      the bridge eventually rather than immediately

Reading the counters when something is wrong:

| Symptom | Where to look |
|---|---|
| `uart_rx` stays 0 | The Teensy is not transmitting. Did you flash `telemetry_test_uart`, not `telemetry_test`? |
| `uart_rx` climbs, `peer none` | The laptop never announced itself — run `wifi_link_test.py --announce` |
| CRC failures in `decode.py` | UART layer: baud mismatch, or the Xiao's RX buffer is overrunning |
| Sequence gaps, no CRC failures | Radio: distance, interference, or power save |
| `fail` counter rising | `udp.endPacket()` is failing — the client dropped off the AP |
| `drop` second number rising | Telemetry generated before any peer was known. Harmless at startup |

---

# X5 — the watchdog belongs on the Teensy

**Not implemented here, and deliberately not in the bridge firmware.**

If the laptop's link dies while the cube is balancing, something must stop the
motor. That decision cannot live on the radio module, for the obvious reason
that a fail-safe implemented on the radio cannot fire when the radio is what
failed.

For the same reason `xiao_bridge` never injects its own bytes into the UART: a
"helpful" keepalive from the bridge is exactly the kind of thing that keeps a
watchdog from ever firing.

The Teensy side is `WIFI_TELEMETRY_CONTROL_Cubli.md` Step 6, and it composes
with two mechanisms that already exist in this project:

- `BalancingController`'s safety envelope (tilt e-stop, torque clamp,
  wheel-speed limit) — currently a scaffold
- moteus's own `watchdog_timeout` on every position command, which stops the
  board if the *Teensy* dies. See CLAUDE.md, safety rule 3.

Three layers, each covering the failure of the one above it.

**Check X5** (once written)
- [ ] Walk the laptop out of range mid-run — the motor reaches a safe state
      within ~500 ms, not indefinitely
- [ ] Reconnecting resumes control without a manual reset

---

# X6 — soak

Ten minutes minimum, with the cube on its own power rather than a bench USB
supply.

```bash
python.exe tools/telemetry/wifi_link_test.py -s 600 \
    --max-loss-pct 1.0 --max-gap-ms 250
```

**Check X6**
- [ ] `PASS` over the full duration
- [ ] Loss rate low enough that the Teensy watchdog does not trip in normal
      operation, but it reliably does trip when the link is actually cut
- [ ] No unexplained latency spikes in `datagram gap max`
- [ ] The Xiao's `heap` is as flat at minute 10 as at minute 1
- [ ] Any SoftAP client disconnect recovers both directions without a reflash

---

## Quick reference

| Item | Value |
|---|---|
| Board id | `seeed_xiao_esp32c6` (pioarduino fork — see `[xiao_base]`) |
| UART pins | Xiao TX = D6 → Teensy pin 0 (RX1); Xiao RX = D7 ← Teensy pin 1 (TX1) |
| UART baud | **921600** both ends |
| SoftAP | SSID `CubliNet`, password `cubli1234`, IP `192.168.4.1` |
| Control port | 5005, laptop → cube |
| Telemetry port | 5006, cube → laptop |
| Frame format | `cube::TelemetryFrame`, 118 bytes, magic `0xA5C3`, CRC-16/CCITT |
| Datagram batching | up to 944 bytes (8 frames) or 10 ms, whichever first |
| Bandwidth | 23.6 kB/s at 200 Hz, 47.2 kB/s at 400 Hz |
| Watchdog | Teensy side, 500 ms — **not yet written** |

Overridable at build time without editing source:

```bash
pio run -e xiao_bridge -t upload \
  --build-flag "-DUART_BAUD=460800" \
  --build-flag "-DFLUSH_TIMEOUT_MS=20" \
  --build-flag '-DWIFI_AP_SSID=\"CubliNet2\"'
```
