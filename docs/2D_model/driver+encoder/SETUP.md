# moteus + MA600 off-board encoder — setup guide

Written for **this** board and this encoder, not the general case:

| Item | Value | Where it's set |
|---|---|---|
| Controller CAN ID | 1 | `MOTEUS_CONTROLLER_ID` |
| Aux port | **aux2 only** — this board has no aux1 | — |
| Encoder | MA600, off-axis, SPI | `AUX2_SPI_MODE=5` |
| SPI rate | 6 MHz (not the 12 MHz default) | `AUX2_SPI_RATE_HZ` |
| Encoder CPR | 65536 | `SOURCE0_CPR` |
| Commutation + output source | source 0 (the MA600) | `COMMUTATION_SOURCE`, `OUTPUT_SOURCE` |

> **This board exposes aux2 only.** `moteus_tool --dump-config` reports no
> `aux1.*` keys whatsoever. Every `conf set aux1...` command you find in
> generic moteus tutorials will be rejected here. Use `aux2.*` throughout.

Everything below assumes the venv tools:

```bash
cd ~/projects/moteusDriver
TOOL=./moteus-venv/bin/moteus_tool
```

---

## Phase 1 — Wiring and encoder configuration

### 1.1 Physical wiring

The MA600 is a 4-wire SPI device. Map it to aux2 as the config expects:

| MA600 pin | aux2 pin | `aux2.pins.X.mode` | Meaning |
|---|---|---|---|
| SCLK | pin 0 | `1` (SPI) | clock |
| MISO | pin 1 | `1` (SPI) | data from encoder |
| MOSI | pin 2 | `1` (SPI) | data to encoder |
| CS   | pin 3 | `2` (SPI_CS) | chip select |
| —    | pin 4 | `0` (NC) | unused |

Plus 3.3 V and ground from the aux2 connector. Consult
`moteus/docs/reference/pinouts.md` and your board's pinout SVG for the
physical connector orientation before powering on.

Cable notes that matter for this build:

- **Keep the SPI cable short.** The 6 MHz rate in this config exists
  because the MA600 would not hold the 12 MHz default over the breakout
  cable. A longer or unshielded cable may need 4 MHz
  (`-DAUX2_SPI_RATE_HZ=4000000`).
- The MA600 is an **off-axis** capable sensor, but still wants the magnet
  positioned per its datasheet. Poor magnet placement shows up as a
  noisy or non-monotonic position reading in Phase 3.

### 1.2 Register the encoder as the position source

Two independent steps, and skipping the second is the most common
mistake: **pin modes only wire up the bus; they do not tell the firmware
to read position from it.**

Everything here is already in `current_config.cfg.in` and gets pushed
automatically by the driver. To apply it by hand:

```bash
# 1. Pin modes: SPI bus on pins 0-2, chip select on pin 3.
$TOOL -t 1 --console
conf set aux2.pins.0.mode 1
conf set aux2.pins.1.mode 1
conf set aux2.pins.2.mode 1
conf set aux2.pins.3.mode 2
conf set aux2.pins.4.mode 0

# 2. SPI peripheral: mode 5 = MA600, at 6MHz.
conf set aux2.spi.mode 5
conf set aux2.spi.rate_hz 6000000

# 3. CRITICAL: UART must be off or it holds the same pins.
conf set aux2.uart.mode 0

# 4. Now make the firmware actually *read* from that bus.
conf set motor_position.sources.0.aux_number 2
conf set motor_position.sources.0.type 1        # 1 = SPI
conf set motor_position.sources.0.cpr 65536
conf set motor_position.sources.0.reference 0   # 0 = rotor
conf set motor_position.sources.0.pll_filter_hz 400

# 5. Point commutation and output at source 0 instead of the onboard sensor.
conf set motor_position.commutation_source 0
conf set motor_position.output.source 0

conf write     # persist to flash
```

Two values worth understanding rather than copying:

- **`pll_filter_hz 400`, not 0.** The MA600 does not measure velocity
  natively. With the filter at `0` the source produces *no velocity
  reading at all*, and position mode has nothing to control against.
- **`reference 0` (rotor).** Correct because this encoder is on the
  rotor and drives commutation. An output-side encoder after a gearbox
  would use `1`, and would also need
  `motor_position.rotor_to_output_ratio`.

### 1.3 The pin-conflict failure this board already hit

`aux2.uart.mode 5` was left set while SPI wanted the same pins. Result:

```
error.uart_pin_error
spi.active  False
```

Two peripherals cannot hold one pin. This is why
`current_config.cfg.in` sets **every** peripheral explicitly — quadrature,
hall, index, sine/cosine, PWM, BiSS-C, all three I2C slots — including the
disabled ones. A stale value on any of them silently claims a pin that SPI
needs. `CMakeLists.txt` also has a configure-time `FATAL_ERROR` guard that
catches the UART/SPI combination before it reaches the board.

---

## Phase 1.5 — Gearbox and safety limits

> ⚠️ **Apply this phase AFTER calibration (Phase 2), not before.**
>
> Calibration must spin the rotor through many continuous revolutions.
> Current-mode calibration issues `d pos nan 0 nan c<current> b1` — a
> *position mode* command — so it is subject to `servopos.position_min/max`.
> With those set to ±1 rotation the controller produces holding torque but
> the phase sweep never advances: the shaft becomes hard to turn by hand,
> phase current stays near zero, and the raw stream shows `phase 0` with the
> encoder jittering a few counts. Calibration then fails with a
> `ZeroDivisionError` on `poles == 0`.
>
> During calibration `servopos.position_min/max` must be `nan`, and
> `servo.max_current_A` must be high enough to actually turn the motor.
> Restore these limits once calibration succeeds.

These are the general first-time-setup parameters, independent of the
encoder. A bare board ships without them, and **with the trajectory limits
unset the firmware applies commands immediately — velocity changes
instantaneously and can "grow to arbitrary magnitude."** So the first
position command you send in normal operation slams to target at whatever
speed the hardware allows. Set these before running the driver — but after
calibration.

All of them are in `current_config.cfg.in` and pushed by the driver. The
defaults are deliberately conservative — meant to be raised toward the real
machine, not lowered:

| Parameter | Default here | Notes |
|---|---|---|
| `motor_position.rotor_to_output_ratio` | `1.0` | direct drive; 4x reducer = `0.25` |
| `servopos.position_min` / `_max` | `-1.0` / `1.0` | output rotations; `nan` disables |
| `servo.max_current_A` | `5.0` | torque ceiling — see warning below |
| `servo.max_velocity` | `20.0` | backstop, power derates past this |
| `servo.default_velocity_limit` | `2.0` | output rev/s |
| `servo.default_accel_limit` | `10.0` | output rev/s² |

Override at configure time:

```bash
cmake -S . -B build \
  -DROTOR_TO_OUTPUT_RATIO=0.25 \
  -DSERVOPOS_POSITION_MIN=-10 -DSERVOPOS_POSITION_MAX=10 \
  -DSERVO_MAX_CURRENT_A=8
```

Three things the checklist page understates:

- **`servo.max_current_A` can damage hardware if raised too far.** The
  upstream reference is explicit: "Increasing beyond the factory configured
  value can result in hardware damage." Use the *lower* of your motor's
  rated phase current and the controller's own rating — a motor rated
  higher than the board does not raise the board's ceiling.
- **Set `rotor_to_output_ratio` before tuning PID.** `servo.pid_position`
  gains are in physical units *at the output*, i.e. after this scaling.
  Change the ratio afterwards and every gain silently changes meaning.
  A ratio above `1.0` faults unless `rotor_to_output_override` is set;
  CMake now rejects that at configure time, since entering a 4x reduction
  as `4` instead of `0.25` is the easy mistake.
- **`servo.max_velocity` is not the trajectory limit.** It only derates
  output power as a backstop. What governs how fast the motor travels to
  a target is `default_velocity_limit` / `default_accel_limit`.

**PID tuning (`servo.pid_position`) comes after calibration**, so it is not
in this file — the gains depend on the calibrated motor. Tune it once
Phase 2 succeeds, following the upstream PID Tuning guide.

`id.id` is also deliberately excluded: the driver addresses the board by
its CAN ID, so pushing a new one mid-session would disconnect it. Change it
by hand with `moteus_tool`, one controller on the bus at a time. Note that
with tview or firmware older than late 2025 you must relaunch as
`python -m moteus_gui.tview -t N` with the new ID before `conf write` will
save.

---

## Phase 2 — Calibration with the external encoder

**Warning: the motor spins freely in both directions at high speed during
calibration. Unbolt or clear anything attached to the shaft.**

Note that `./build/moteus_driver` re-pushes the whole config — including the
Phase 1.5 limits — on every run, because `APPLY_CONFIG_ON_STARTUP` defaults
to `ON`. Running the driver between `conf set` and calibrating will silently
put the limits back. While bringing up calibration, build with:

```bash
cmake -S . -B build -DAPPLY_CONFIG_ON_STARTUP=OFF && cmake --build build
```

Do Phase 1 *first and verify it*, because `--calibrate` uses whatever
`commutation_source` currently points at. Calibrating before repointing it
silently characterises the onboard sensor instead.

**Do NOT apply Phase 1.5 before this.** Its travel limits block the
multi-revolution sweep calibration needs. If Phase 1.5 has already been
applied, clear the limits first:

```
conf set servopos.position_min nan
conf set servopos.position_max nan
conf set servo.max_current_A <high enough to turn the motor>
```

Restore them after calibration succeeds.

```bash
$TOOL -t 1 --calibrate
```

For an off-board encoder specifically:

```bash
# If the rotor barely turns or the cable is long, cap the calibration speed.
# Note: --cal-speed is DEPRECATED in the installed version; --cal-motor-speed
# is the current flag and is mechanical Hz, not electrical rps.
$TOOL -t 1 --calibrate --cal-motor-speed 1.0

# There is also a maximum-error knob if calibration is borderline:
#   --cal-max-remainder F
```

### Failure modes

**`CAL timeout` / calibration hangs**

Almost always the encoder isn't actually being read. Check before
re-running:

```bash
$TOOL -t 1 --dump-config | grep -E "aux2.spi.mode|commutation_source"
# expect:  aux2.spi.mode 5
#          motor_position.commutation_source 0
```

Then confirm `spi.active` is `True` in Phase 3. If it's `False`, you have
a pin conflict (§1.3), not a calibration problem.

**Directional mismatch — motor runs away, oscillates, or faults instantly**

The encoder counts the opposite way from the motor's electrical phase
order. Do **not** fix this by swapping motor phase wires; flip the source
sign:

```bash
conf set motor_position.sources.0.sign -1
conf write
```
Then recalibrate. (Build-side equivalent: `-DSOURCE0_SIGN=-1`.)

**Erratic position / calibration succeeds but commutation is rough**

Signal integrity on the SPI cable. Drop the rate and retry:

```bash
conf set aux2.spi.rate_hz 4000000
conf write
```

**Wrong CPR** — if position wraps at the wrong point, `SOURCE0_CPR` does
not match the MA600's configured resolution. It must equal the encoder's
actual counts per revolution (65536 here).

After calibrating a new motor type you may need to retune PID gains.

---

## Phase 3 — Verify telemetry before running C++

Confirm the encoder is alive and sane *before* introducing your own code
as a variable.

```bash
$TOOL -t 1 --info
```

Then confirm the whole aux2 block came back as intended:

```bash
$TOOL -t 1 --dump-config | grep aux2
```

The runtime status field that matters most is `aux2.spi.active` — it must
read `True`. If it is `False`, Phase 1 is not actually done and you have a
pin conflict (§1.3). Read it in tview's telemetry tree (below), which shows
live state rather than stored configuration.

Watch live position, velocity, and voltage:

```bash
$TOOL -t 1 --console
d stop
d pos nan 0 nan
```

Or use the GUI, which plots channels in real time:

```bash
./moteus-venv/bin/tview
```

In tview, tick `servo_stats.position`, `servo_stats.velocity`, and
`servo_stats.filt_bus_V` in the left tree. **Turn the shaft by hand** and
confirm:

- position changes smoothly and monotonically in one direction
- it wraps cleanly rather than jumping mid-rotation
- velocity is non-zero while turning — if it's pinned at zero,
  `pll_filter_hz` is `0` (see §1.2)
- `fault` stays `0`

Only proceed once all four hold.

---

## Phase 4 — Build and run the C++ driver

```bash
cd ~/projects/moteusDriver
cmake -S . -B build
cmake --build build
```

Run from the **project root**, not from inside `build/` — `cmake -S .`
resolves relative to the current directory and creates a broken nested
configuration otherwise.

```bash
./build/moteus_driver
```

Transport selection. **The C++ binary and the Python tools spell these
differently:**

```bash
# C++ driver:
./build/moteus_driver --fdcanusb /dev/ttyACM0
./build/moteus_driver --socketcan-iface can0

# Python moteus_tool / tview:
$TOOL --fdcanusb /dev/ttyACM0
$TOOL --can-iface socketcan --can-chan can0
```

Bringing up a SocketCAN interface for moteus (CAN-FD, 1 Mbps arbitration /
5 Mbps data):

```bash
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set up can0
```

Build options:

```bash
cmake -S . -B build -DAPPLY_CONFIG_ON_STARTUP=OFF   # read-only, don't push config
cmake -S . -B build -DPERSIST_CONFIG_TO_FLASH=ON    # follow the push with `conf write`
```

Without `PERSIST_CONFIG_TO_FLASH` the settings are re-applied each run and
lost on power cycle — fine while iterating, but do a `conf write` once the
pin map is settled.

This is worth being explicit about, because **every parameter — the encoder
setup, the Phase 1.5 limits, and `id.id` — lives in volatile memory until
`conf write`.** A power cycle silently reverts a board you thought was
configured. The driver re-pushes on each run, which masks the problem right
up until something else talks to the controller first. Once the values are
settled:

```bash
cmake -S . -B build -DPERSIST_CONFIG_TO_FLASH=ON && cmake --build build
```

or in the tview console / `moteus_tool --console`:

```
conf write
```

Tests:

```bash
sudo apt install libboost-test-dev
cmake -S . -B build && cmake --build build
ctest --test-dir build --output-on-failure
```

### What the driver does today

`apps/legacy_moteus_driver.cpp` runs a 100 Hz velocity-mode loop:
`position = NaN` with `command.velocity` set, which is how moteus does
constant-speed running. Override the speed per run:

```bash
./build/moteus_driver --velocity 2.0     # output rev/s
```

For the balancing rig the relevant binaries are `direct_motor_test`,
`imu_test` and `cube_balancer` -- see the [root README](../README.md).
`MoteusDriverWrapper::sendTorque()` uses a different command shape from the
legacy driver: `position = NaN` with `kp_scale = kd_scale = 0` and
`feedforward_torque` set, which turns off the board's own regulation so the
balancing law is the only regulator.

Stop the motor with:

```cpp
c.SetStop();
```

Always `SetStop()` before exiting, or the controller holds its last
commanded position and keeps drawing current.
