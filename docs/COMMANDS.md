# moteusDriver — module-by-module command card

Run everything from the project root: `~/projects/moteusDriver`.

Each section starts from a cold board and ends in a check you can read off the
screen. Every command is copied from a source of truth in the repo
(`CMakeLists.txt`, `platformio.ini`, `docs/2D_model/*.md`,
`tools/telemetry/`) — if one stops working, that file is what to fix first.

---

## Contents

- [1. Where each command runs](#1-where-each-command-runs)
- [2. moteus controller — direct connection](#2-moteus-controller--direct-connection)
- [3. moteus Python tooling](#3-moteus-python-tooling-bring-up-not-runtime)
- [4. Teensy 4.1 — the MCU on its own](#4-teensy-41--the-mcu-on-its-own)
- [5. IMU — BMI270 over SPI](#5-imu--bmi270-over-spi-on-the-teensy)
- [6. CAN link from the Teensy](#6-can-link-from-the-teensy)
- [7. Telemetry over USB](#7-telemetry-over-usb-the-baseline-path)
- [8. WiFi hop — XIAO ESP32-C6](#8-wifi-hop--xiao-esp32-c6)
- [9. Balancer](#9-balancer)
- [10. Quick smoke sequence](#10-quick-smoke-sequence--is-everything-still-working)
- [11. Safety rules](#11-safety-rules--any-command-that-moves-the-motor)

---

## 1. Where each command runs

Three environments, and mixing them up is the most common wasted hour.

| Job | Where | Why |
|---|---|---|
| `cmake`, `ctest`, host binaries | WSL | the Linux driver |
| `pio run` (compile) | WSL | toolchain lives here |
| `pio run -t upload` (Teensy) | **Windows** | HalfKay re-enumerates, drops usbipd |
| `pio run -t upload` (XIAO) | WSL | native USB, keeps its identity |
| `tview`, `moteus_tool` | WSL | WSLg, fdcanusb over usbipd |
| `capture.py --udp`, `wifi_link_test.py` | **Windows** (`python.exe`) | WSL2 NAT eats inbound UDP |
| `decode.py`, `analyze.ipynb` | WSL | needs pandas + pyarrow |

### Attach the USB device to WSL (once per plug-in)

From an **Administrator PowerShell** on Windows:

```powershell
usbipd list
usbipd attach --wsl --busid <BUSID>
```

The fdcanusb then appears as `/dev/ttyACM0`. Permission denied:

```bash
sudo usermod -aG dialout $USER    # then log out and back in
```

### Build everything host-side

```bash
cmake -S . -B build         # from the project root, NOT from inside build/
cmake --build build
ctest --test-dir build --output-on-failure
```

*Proves:* the toolchain, the layering rule (`cube_core` links with no driver
sources), and the telemetry wire format — all with no hardware attached.

---

## 2. moteus controller — direct connection

The reference module: host → fdcanusb → CAN-FD → moteus. Nothing else in the
rig is involved.

### Spin the motor at constant velocity

```bash
cmake --build build && ./build/moteus_driver --velocity 2.0
```

Velocity mode on moteus is `position = NaN` with `velocity` set — there is no
separate velocity mode. The driver pushes `build/current_config.cfg` at
startup, runs a 100 Hz loop with a watchdog, and calls `SetStop()` on Ctrl-C.

```bash
./build/moteus_driver --velocity 0            # config push only, no motion
./build/moteus_driver --velocity 2.0 --config build/current_config.cfg
```

### Persist the configuration to flash

Without this, **every setting lives in volatile memory and is lost on power
cycle**. The driver re-pushing on each run hides that until `tview` or
`moteus_tool` talks to the board first and sees the stale flashed values —
which is what produces fault 39, `kStartOutsideLimit`.

```bash
cmake -S . -B build -DPERSIST_CONFIG_TO_FLASH=ON && cmake --build build
./build/moteus_driver --velocity 0
cmake -S . -B build -DPERSIST_CONFIG_TO_FLASH=OFF && cmake --build build
```

Run it once, then turn the flag back off — flash has a finite write count and
there is no reason to rewrite it every session.

### Change a controller setting

Settings are **CMake cache variables**, not a file you edit.
`build/current_config.cfg` is generated; never edit it.

```bash
cmake -S . -B build -DSERVO_MAX_CURRENT_A=8 -DAUX2_SPI_RATE_HZ=4000000
cmake --build build
```

> ⚠ `servo.max_current_A` can destroy hardware. Use the *lower* of the motor's
> rated phase current and the controller's rating.

Read-only run (skip the config push entirely):

```bash
cmake -S . -B build -DAPPLY_CONFIG_ON_STARTUP=OFF && cmake --build build
```

### Motor health checks

```bash
./build/direct_motor_test                 # interactive menu
./build/direct_motor_test --run-all       # exit code is the verdict
./build/direct_motor_test --run-all --yes # no confirmation prompts
./build/direct_motor_test --apply-config build/current_config.cfg
```

*Proves:* encoder counts, fault state, torque response and the CAN round-trip.
Writes a CSV into `logs/`.

---

## 3. moteus Python tooling (bring-up, not runtime)

```bash
TOOL=./moteus-venv/bin/moteus_tool

$TOOL -t 1 --info                        # is the board there at all?
$TOOL -t 1 --dump-config | grep aux2     # what does it actually believe?
$TOOL -t 1 --console                     # raw `d pos` / `conf set` REPL
$TOOL -t 1 --write-config build/current_config.cfg
```

### Live plotting

```bash
./moteus-venv/bin/tview --devices=1
```

Channels that matter during bring-up:

| Channel | What it tells you |
|---|---|
| `servo_stats.position`, `.velocity` | does the encoder move with the shaft |
| `servo_stats.filt_bus_V` | supply sag under load |
| `aux2.spi.active` | must read `True` |

> ⚠ `aux2.spi.active = False` is a **pin conflict**, not a calibration problem.
> A stale `aux2.uart.mode` claiming an SPI pin produces `error.uart_pin_error`
> on this board.

### Calibration

```bash
$TOOL -t 1 --calibrate --cal-motor-speed 1.0
```

> ⚠ Calibration needs `servopos.position_min/max = nan`. The sweep is issued as
> a *position mode* command and travel limits block it — the symptom is a
> `ZeroDivisionError` on `poles == 0`, which looks nothing like a limits
> problem. `nan` is already the `CMakeLists.txt` default. `--cal-speed` is
> deprecated; `--cal-motor-speed` is mechanical Hz.

### This rig's hardware facts

| Item | Value | Set by |
|---|---|---|
| Controller CAN ID | 1 | `MOTEUS_CONTROLLER_ID` |
| Aux port | **aux2 only — no aux1 on this board** | — |
| Encoder | MA600, off-axis, SPI, 65536 CPR | `AUX2_SPI_MODE=5` |
| SPI rate | 6 MHz (not the 12 MHz default) | `AUX2_SPI_RATE_HZ` |
| Motor poles | 24 | from calibration |
| Gearing | 1.0, direct drive | `ROTOR_TO_OUTPUT_RATIO` |

Any `conf set aux1...` from a generic moteus tutorial **will be rejected on
this board**.

### SocketCAN alternative to fdcanusb

```bash
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set up can0
```

---

## 4. Teensy 4.1 — the MCU on its own

Build in WSL, flash from Windows. Environment names are flat: `-e imu_read`,
never `-e imu/imu_read`.

```bash
pio run -e board_check                 # WSL:     compile
pio run -e board_check -t upload       # Windows: flash
pio device monitor -e board_check      # 115200
```

*Proves:* the board is alive and the flash pipeline works. Blinks
asymmetrically, prints `F_CPU` and `sizeof(double)`. No external hardware.

```bash
pio run -e spi_loopback -t upload      # needs ONE jumper: pin 11 -> pin 12
pio run -e core_check   -t upload      # does core/ compile on BOTH toolchains?
```

Bit-rot check — build every environment without flashing anything:

```bash
pio run -e board_check -e spi_loopback -e core_check -e imu_spi_test \
        -e imu_config_upload -e imu_read -e imu_calibrate -e imu_driver_test \
        -e can_listen -e telemetry_test -e telemetry_test_uart \
        -e moteus_driver_test -e cube_balancer \
        -e xiao_board_check -e xiao_uart_echo -e xiao_softap \
        -e xiao_frame_source -e xiao_bridge
```

> ⚠ A bare `pio run` builds only `default_envs` (`board_check`), **not** every
> environment.

---

## 5. IMU — BMI270 over SPI on the Teensy

Wiring is `docs/2D_model/IMU_SETUP.md` step 1. Run these in order — each one
assumes the previous passed.

### Step 2 — does the chip answer?

```bash
pio run -e imu_spi_test -t upload
pio device monitor -e imu_spi_test
```

*Proves:* `CHIP_ID` reads `0x24`. Anything else is wiring or SPI mode, not
software.

### Step 3 — upload the 8 kB config blob

```bash
pio run -e imu_config_upload -t upload
```

*Proves:* the accel and gyro turn on. They ship **disabled**, and the blob does
not survive a reset — every later sketch re-uploads it itself.

### Step 4 — read real data

```bash
pio run -e imu_read -t upload
pio device monitor -e imu_read
```

*Proves:* converted accel/gyro at 10 Hz. The physical sanity test: one axis
reads ≈ +9.81 when pointed down, and rotating the board moves the matching gyro
axis.

### Steps 4b/5/6 — calibrate

```bash
pio run -e imu_calibrate -t upload
pio device monitor -e imu_calibrate
```

Interactive menu: `[m]` axis mapping, `[g]` gyro bias, `[1]`–`[6]` then `[f]`
the six-position accel fit, `[s]` prints paste-ready build flags.

> ⚠ Paste the `[s]` output into the shared `build_flags` in `platformio.ini`
> **and** the `IMU_*` cache variables in `CMakeLists.txt`. Each value is defined
> twice under two macro names (`GYRO_BIAS_X` for the Teensy driver,
> `DEFAULT_IMU_GYRO_BIAS_X` for the host driver). Both fall back to the identity
> transform, so a missing name does not fail the build — it silently produces an
> *uncalibrated* binary that looks exactly like a calibrated one.

Re-measure the gyro bias if the rig runs much warmer than the 23.5 °C bench
conditions in `data/imu_calibration/`.

### Cross-validate the gathered driver

```bash
pio run -e imu_driver_test -t upload
pio device monitor -e imu_driver_test
```

*Proves:* `Bmi270SpiDriver` — the `IImuSensor` the control loop actually uses —
still agrees with the raw sketches: same bench, same pose, same numbers.

### Host-side IMU (Linux I²C path)

Separate driver, separate bus. Only relevant if the IMU is wired to the host
rather than the Teensy.

```bash
./build/imu_test                          # validation + gyro bias calibration
./build/imu_test --stream                 # live raw-sensor stream
./build/imu_test --device /dev/i2c-1 --address 0x68 --bias-seconds 10
```

---

## 6. CAN link from the Teensy

### Listen-only

```bash
pio run -e can_listen -t upload
pio device monitor -e can_listen
# in another terminal, make the HOST talk to the board:
./build/moteus_driver --velocity 0
```

*Proves:* the Teensy sees the host's CAN-FD traffic. It cannot transmit, by
construction.

> ⚠ Read the meter checks in the sketch header **before** flashing. A 5 V
> transceiver destroys CAN3 RX permanently. Pins 30 (TX) / 31 (RX) — CAN3 is the
> only FD-capable module on this silicon, and moteus requires FD.

### Teensy transmits — query path only

```bash
pio run -e moteus_driver_test -t upload
pio device monitor -e moteus_driver_test
```

*Proves:* `TeensyMoteusDriver` queries the board and validates its config
against `EXPECT_SERVO_MAX_CURRENT_A` etc. `sendTorque()` is not compiled in, so
the motor cannot be commanded no matter what the menu does.

> ⚠ **Wheel off the shaft**, moteus on logic power only.

### Teensy commands torque — the only firmware that moves the motor

```bash
pio run -e moteus_driver_test_torque -t upload
```

> ⚠ **REMOVE THE REACTION WHEEL FROM THE SHAFT BEFORE FLASHING.** Menu
> `[+]`/`[-]` check both torque signs; `[w]` checks the watchdog — unplug the
> Teensy mid-command and the shaft must stop in ≈150 ms. If it keeps turning,
> `watchdog_timeout` was left at `kIgnore` and nothing after this step is safe.

---

## 7. Telemetry over USB (the baseline path)

```bash
# 1. build + flash                                        WSL / Windows
pio run -e telemetry_test -t upload

# 2. capture                                 Windows python, from WSL terminal
python.exe tools/telemetry/capture.py --port COM9 -s 60 --live

# 3. decode                                                          WSL
./moteus-venv/bin/python tools/telemetry/decode.py logs/telemetry_<stamp>.bin --parquet

# 4. replay a capture with no hardware attached                       WSL
./moteus-venv/bin/python tools/telemetry/live.py logs/telemetry_<stamp>.bin --speed 1.0
```

> ⚠ `telemetry_test` **emits binary, not text**. Opening it in a serial monitor
> shows garbage, and a monitor holding the port blocks the capture. Do not run
> `pio device monitor` on this env.

Reading the decode summary — the distinction that matters:

| Counter | What it means |
|---|---|
| `ring overflows` | producer-side: the drain fell behind. A *firmware* problem. |
| `frames lost` | frames left the ring and never arrived. A *link* problem. |
| `CRC failures` | corruption, not loss. Non-zero over USB means a real bug. |

---

## 8. WiFi hop — XIAO ESP32-C6

Full procedure with per-step checks: `docs/2D_model/XIAO_BRINGUP.md`. The XIAO
flashes **directly from WSL** — it keeps its USB identity across a flash, unlike
the Teensy.

> ⚠ **Do not pass `--upload-port`, and do not set `upload_port` in the ini.**
> Autodetect matches the board's VID:PID, so it is immune to `/dev/ttyACM*`
> renumbering and cannot pick the Teensy. A `by-id` glob does *not* work —
> PlatformIO fnmatches the pattern against `/dev/ttyACM0`-style names only,
> matches nothing, and esptool reports the misleading
> `port is busy or doesn't exist`.

> ⚠ **WSL2 here is in NAT mode and cannot receive the telemetry.** The bridge
> learns the *Windows* host's address, sends to it, and `capture.py --udp` in
> WSL sits there reporting nothing with no error. `netsh portproxy` does not
> help — it forwards TCP, and this link is UDP. Run the receivers with
> `python.exe`. Both receiver tools are stdlib-only so a bare Windows Python
> runs them.

Network: SSID **`CubliNet`**, password **`cubli1234`**, bridge at
`192.168.4.1`. Ports: `5005` laptop → cube, `5006` cube → laptop.

### X0 — is the XIAO alive? (no wiring)

```bash
pio run -e xiao_board_check -t upload
pio device monitor -e xiao_board_check
```

*Proves:* chip, heap, flash size and reset reason print, then `tick N` once a
second. `reset reason POWERON` is what you want — `BROWNOUT` means the USB
supply is sagging, so change the cable before believing anything else.

### X1 — does the UART to the Teensy work? (3 wires)

```
   XIAO ESP32-C6                    Teensy 4.1
  ┌──────────────┐                ┌──────────────────┐
  │  GND ────────┼────────────────┤ GND              │
  │  D6 (TX) ────┼────────────────┤ 0   (RX1)        │
  │  D7 (RX) ────┼────────────────┤ 1   (TX1)        │
  └──────────────┘                └──────────────────┘
```

> ⚠ The GND wire is **not optional**. Two boards on separate USB ports sit at
> different ground potentials; without the common return the UART reads as
> noise.

```bash
pio run -e xiao_uart_echo -t upload
pio device monitor -e xiao_uart_echo
# bisect a bad link by dropping both ends to a slow rate:
pio run -e xiao_uart_echo -t upload --build-flag "-DUART_BAUD=115200"
```

*Proves:* one flash tests both directions — the XIAO sends `XIAO_PING` once a
second and hexdumps everything it receives. With `telemetry_test_uart` on the
Teensy you should see `C3 A5` recurring on a 118-byte pitch. Consistent garbage
with no `C3 A5` is a baud mismatch.

> ⚠ **The UART runs at 921600, not 115200.** 118 B × 200 Hz × 10 bits =
> 236 kbaud minimum; at the IMU's 400 Hz ODR it is 472 kbaud. 115200 carries
> 97 frames/s and does not fail cleanly — it corrupts under load. Both ends must
> match: `UART_BAUD` in `xiao_bridge`, `kUartBaud` in `telemetry_test`.

### X2 — does the radio work on its own? (no wiring)

```bash
pio run -e xiao_softap -t upload
pio device monitor -e xiao_softap
```

Join `CubliNet` on the laptop, then:

```bash
ping 192.168.4.1
nc -u -l 5006 &
echo hello | nc -u -w1 192.168.4.1 5005
```

*Proves:* `hello` comes back on port 5006. The monitor's `clients` count going
`0 -> 1` is the diagnostic that matters — a laptop showing "connected" in the OS
but never appearing here has silently fallen back to a saved network.

### X3 — does WiFi carry real telemetry frames? (no wiring)

The highest-value test here: the XIAO generates valid 118-byte
`cube::TelemetryFrame` records itself. No Teensy, no UART, no IMU — so anything
the host fails to decode was mangled by the radio and nothing else.

```bash
pio run -e xiao_frame_source -t upload
```

Join `CubliNet`, then from Windows:

```bash
# tell the XIAO where you are -- it has no hardcoded laptop IP
python.exe tools/telemetry/wifi_link_test.py --announce

# measure the link and get a verdict
python.exe tools/telemetry/wifi_link_test.py -s 60

# or capture to disk with the normal tool, over UDP instead of COM
python.exe tools/telemetry/capture.py --udp 0.0.0.0:5006 --out logs/wifi_synth.bin -s 60 --live
./moteus-venv/bin/python tools/telemetry/decode.py logs/wifi_synth.bin
```

*Proves:* `PASS`; ≈200 Hz and ≈23.6 kB/s; `CRC failures 0`; frames lost under
1 %; `frames/datagram` showing `8x…` (batching works); datagram gap max well
under 250 ms — a 100 ms+ stall means WiFi power save is still on somewhere.

Headroom check at the IMU's full ODR:

```bash
pio run -e xiao_frame_source -t upload --build-flag "-DFRAME_RATE_HZ=400"
```

### X4 — the bridge, end to end

```bash
pio run -e xiao_bridge -t upload            # XIAO: the bridge
pio run -e telemetry_test_uart              # Teensy: sink swapped to Serial1,
                                            #         then flash from Windows
pio device monitor -e xiao_bridge           # safe: USB and the bridged UART
                                            #       are different devices
```

From Windows:

```bash
python.exe tools/telemetry/wifi_link_test.py -s 60
python.exe tools/telemetry/capture.py --udp 0.0.0.0:5006 --out logs/wifi_full.bin -s 120
./moteus-venv/bin/python tools/telemetry/decode.py logs/wifi_full.bin --parquet
```

*Proves:* `uart_rx` climbing at ≈47 kB/s (400 Hz × 118 B); `peer` showing the
laptop's address, not `none`; `udp_tx` tracking `uart_rx` closely; `fail 0` and
`drop 0/0`; a *flat* heap over minutes.

The asymmetry that localises a fault once the whole chain is running:

| Symptom | Where to look |
|---|---|
| CRC failures, no sequence gaps | **UART** — baud mismatch or RX overrun |
| Sequence gaps, no CRC failures | **radio** — distance, interference, power save |
| `uart_rx` stays 0 | flashed `telemetry_test` instead of `telemetry_test_uart` |
| `peer none` | the laptop never announced — run `--announce` |

> ⚠ **Two load-bearing design decisions.** (1) The bridge adds *no* framing —
> `TelemetryFrame` already carries magic, length, version and CRC-16, and
> `decode.py` already resyncs on it; the teaching guide's
> `[0xAA][LEN][payload][checksum]` envelope is superseded. (2) `Serial1`'s
> default TX buffer on Teensy 4.x is 64 bytes and a frame is 118 —
> `DrainTelemetry()` refuses to write unless `space() >= sizeof(frame)`, so
> without `Serial1.addMemoryForWrite()` *not one frame is ever sent*, with no
> error anywhere.

To go back to the USB capture path at any point, flash `-e telemetry_test`. It
compiles to a bit-for-bit identical binary before and after the WiFi work —
nothing to undo.

---

## 9. Balancer

### Host

```bash
./build/cube_balancer --dry-run
./build/cube_balancer --k-theta 3.2 --k-theta-dot 0.31 --k-omega 0.0021
./build/cube_balancer --rate 200 --max-torque 0.2 --theta-offset 0.0
```

> ⚠ It **refuses to start** while the gains are unset, naming the first unset
> field. That refusal is the expected result today, and it is a safety property,
> not an inconvenience: tunables are `NaN`/`-1` sentinels rather than defaults,
> because NaN comparisons are all false — an unset `max_tilt_rad` would silently
> *disable* the tilt e-stop rather than erroring.

### Firmware, torque disarmed

```bash
pio run -e cube_balancer -t upload
python.exe tools/telemetry/capture.py --port COM9 -s 60 --live
```

*Proves:* BMI270 → StateEstimator → BalancingController → moteus at 200 Hz with
binary telemetry, commanding *nothing* — `sendTorque()` is not compiled in. Fill
the gains in from `docs/3d_scaling/README.md` §3–§4 before the refusal goes
away.

### Firmware, torque armed

```bash
pio run -e cube_balancer_torque -t upload
```

> ⚠ **Wheel off** for the first run. Tilt by hand and check the shaft pushes the
> *right* way. If the cube drives its own fall, **do not flip the sign in the
> control law** — the bug is upstream, in `EST_INVERT_THETA` or `SOURCE0_SIGN`,
> and negating the law to compensate hides it somewhere worse. Then: wheel on,
> start at ≈30 % of the computed gains with `CTL_MAX_TORQUE_NM` clamped low,
> walk both up together across runs, and tune `k_omega` last.

---

## 10. Quick smoke sequence — is everything still working?

Twenty minutes, cold rig to balancing-ready. Each line is attributable to one
part; stop at the first failure rather than continuing.

```bash
# --- host, no hardware ------------------------------------------------
cmake -S . -B build && cmake --build build
ctest --test-dir build --output-on-failure

# --- moteus over CAN --------------------------------------------------
usbipd attach --wsl --busid <BUSID>          # Administrator PowerShell
./moteus-venv/bin/moteus_tool -t 1 --info
./moteus-venv/bin/moteus_tool -t 1 --dump-config | grep aux2
./build/direct_motor_test --run-all          # exit code is the verdict
./build/moteus_driver --velocity 2.0         # Ctrl-C stops it

# --- Teensy + IMU -----------------------------------------------------
pio run -e board_check      -t upload
pio run -e imu_driver_test  -t upload        # calibrated driver, real numbers

# --- telemetry over USB -----------------------------------------------
pio run -e telemetry_test -t upload
python.exe tools/telemetry/capture.py --port COM9 -s 30 --live
./moteus-venv/bin/python tools/telemetry/decode.py logs/telemetry_<stamp>.bin

# --- WiFi hop ---------------------------------------------------------
pio run -e xiao_board_check  -t upload
pio run -e xiao_frame_source -t upload       # radio alone, no wires
python.exe tools/telemetry/wifi_link_test.py --announce
python.exe tools/telemetry/wifi_link_test.py -s 60

pio run -e xiao_bridge         -t upload     # then the whole chain
pio run -e telemetry_test_uart               # flash from Windows
python.exe tools/telemetry/wifi_link_test.py -s 60

# --- balancer ---------------------------------------------------------
./build/cube_balancer --dry-run              # names the first unset gain
```

---

## 11. Safety rules — any command that moves the motor

1. `servo.max_current_A` can destroy hardware. Use the *lower* of the motor's
   and the controller's rating — a motor rated higher than the board does not
   raise the board's ceiling.
2. **Always `SetStop()` before exiting**, including on SIGINT. Without it the
   board holds position and keeps drawing current indefinitely.
3. **Set `watchdog_timeout`** on commands in a real control loop, so the board
   stops itself if the host dies.
4. With `servo.default_velocity_limit` / `default_accel_limit` unset, velocity
   changes instantaneously and the first position command slams to target. They
   are set here (2.0 / 10.0) for that reason.
5. `servo.max_velocity` is a power-derating backstop, **not** a trajectory
   limit.
6. Set `rotor_to_output_ratio` *before* tuning `servo.pid_position` — gains are
   in physical units at the output, so changing the ratio later silently changes
   what every gain means.
7. A reaction wheel must spin freely, so `servopos.position_min/max` are `nan`.
   The former ±1.0 defaults produced fault 103 (`kLimitPositionBounds`, brakes
   hard mid-run) and fault 39 (`kStartOutsideLimit`, refuses to move at all,
   needs `d stop` to clear) — different-looking faults, same root cause.

---

Sources of truth: `CMakeLists.txt`, `platformio.ini`,
`docs/2D_model/XIAO_BRINGUP.md`, `docs/2D_model/IMU_SETUP.md`,
`docs/2D_model/CAN_BRINGUP.md`, `tools/telemetry/README.md`.
