# Bench run procedure — the 1D/2D single-panel rig

Everything you need to go from "code is written" to "the rig balances," in
order, with what stays plugged in at each step. This is the *how*; the *why*
behind each stage is [`INTEGRATION.md`](INTEGRATION.md) (S1–S6, N1–N2) and
[`../3d_scaling/README.md`](../3d_scaling/README.md) §4 (the LQR gains and the
sign question).

This targets `firmware/full_case/cube_balancer` — **the rig itself**, the
Teensy 4.1 firmware. `apps/cube_balancer.cpp` (the host binary) is a desk
reference only; it is not what runs the panel. See `CLAUDE.md` "Which
balancer is the rig."

---

## 0. Where things stand right now

Masses, lengths, inertias and the LQR gains (`CTL_K_THETA` / `CTL_K_THETA_DOT`
/ `CTL_K_OMEGA`) are already measured and pasted into
`[env:cube_balancer]` in `platformio.ini` — see
[`../3d_scaling/measurements.md`](../3d_scaling/measurements.md).

**Still unset, and the firmware will refuse to boot until they aren't:**

| Build flag | What it is | How to get it |
|---|---|---|
| `EST_PIVOT_AXIS_X` / `_Y` / `_Z` | the physical balance edge, as a unit vector in the IMU's own frame | §1 below |
| `EST_INVERT_THETA` | sign convention | §1 below |
| `EST_THETA_OFFSET` | accelerometer angle at true balance | §1 below |
| `CTL_MAX_TILT_RAD` | the tilt e-stop threshold | your call — see §1.4 |

`firstUnsetField()` checks these unconditionally at boot, in every env
including the no-torque S6 build, and halts naming the first one it finds.
That halt is a working safety gate, not a bug — do §1 before flashing
anything else.

---

## 1. Find the pivot axis and sign, on the mounted rig

Gyro bias and accelerometer offset/scale are **already calibrated** (they're
the `GYRO_BIAS_*` / `ACCEL_OFFSET_*` / `ACCEL_SCALE_*` flags already sitting
in the shared `[env]` block of `platformio.ini`, from
`data/imu_calibration/imu-cal-20260805T125013.log`). What's left is
orientation: where the physical balance edge sits relative to the IMU's own
axes, and which sign convention matches "a positive wheel torque corrects
the tilt." Both depend on how the IMU sits on the panel today, so both have
to be measured on the assembled rig, not computed.

**The IMU is not mounted flush with the panel — no raw sensor axis is
vertical at the balance point.** That rules out the old scheme of "pick one
raw axis for the gyro rate, two raw axes for the accelerometer angle": with
a tilted mount, no such pick exists. `StateEstimator::Config` now takes the
pivot direction as a **unit vector** instead (`pivot_axis_x/y/z`), which
handles any mounting angle. It can be found from three ordinary static
poses, because of one fact: the balance edge is horizontal and gravity is
vertical, so gravity's component *along* the pivot axis is exactly zero at
every tilt angle, for any mounting. Two accelerometer readings taken at two
different tilts therefore both lie in the plane perpendicular to the pivot
axis — so their cross product gives that axis directly:

```
pivot_axis = normalize(a_left × a_right)
```

`firmware/imu/imu_axis_verify` automates exactly this, using the
**equilibrium + left-limit + right-limit** poses — the same three poses you'd
naturally want to check anyway — and calls the real
`StateEstimator::accelAngle()` (the exact function `cube_balancer` runs), so
what it reports is exactly what the rig will compute.

### 1.1 Build, flash, monitor

```bash
# WSL2:
pio run -e imu_axis_verify
```
```powershell
# Windows PowerShell, same repo path via \\wsl$\...:
pio run -e imu_axis_verify -t upload
pio device monitor
```

(Single USB serial port here — no `USB_DUAL_SERIAL` on this env — so it's
just the one COM port PlatformIO opens.)

### 1.2 Menu

```
l  raw live stream                    f  fit pivot_axis + theta_offset
e  capture EQUILIBRIUM pose           d  dashboard: live theta (fitted)
2  capture LEFT limit pose            i  toggle invert_theta
3  capture RIGHT limit pose           s  summary + paste-ready flags
```

### 1.3 Procedure

1. Mount the IMU on the rig exactly as it will run for real.
2. `l` — raw live stream, just to confirm the sensor looks sane (`|a|` near
   9.81 m/s²) before spending time on the fit.
3. Hold the panel **at the balance point**, hold still, press `e`. It
   averages ~1.5 s of readings and rejects the capture if it detects motion.
4. Tilt slowly to the **left** limit you care about (e.g. where
   `CTL_MAX_TILT_RAD` will trip), hold still, press `2`.
5. Tilt to the **right** limit, hold still, press `3`.
6. Press `f`. It fits `pivot_axis` from the three poses and derives
   `theta_offset` from the equilibrium one (so theta reads ~0 there by
   construction), and flags two failure modes: left/right too close to
   parallel (tilt further apart and re-capture), or the equilibrium pose not
   actually perpendicular to the fitted axis (a sign a pose wasn't held
   still, or the rig moved on something other than the intended tilt).
7. Press `s`. It prints `theta` at all three poses and flags a problem if
   left/right come out the same sign (poses too similar — re-capture) or are
   wildly asymmetric (rig not re-centred, or equilibrium mis-captured).
8. **The sign — this tool cannot decide it for you.** Tilt the panel toward
   whichever side a *positive* wheel torque is supposed to correct. `theta`
   must read **positive** there. If it reads negative, press `i`
   (`invert_theta`) and press `s` again — it recomputes from the stored
   poses, no re-tilting or re-capturing needed.

   Getting this backwards and only catching it later is the single most
   dangerous mistake in the project: with the wheel on and torque live, a
   flipped sign drives the panel *into* its fall at full authority instead
   of catching it. This is exactly what N1 (§4) exists to catch as well —
   treat this step as the first pass, N1 as confirmation with the real
   motor.

9. `d` — live dashboard using the fitted mapping. Sweep slowly through the
   full range one more time and confirm theta moves smoothly, with no jump
   near either limit.
10. `s` prints the paste-ready block:
    ```
    -D EST_PIVOT_AXIS_X=<value>
    -D EST_PIVOT_AXIS_Y=<value>
    -D EST_PIVOT_AXIS_Z=<value>
    -D EST_INVERT_THETA=<true|false>
    -D EST_THETA_OFFSET=<value>
    ```
    Paste these into `[env:cube_balancer]` in `platformio.ini`, replacing the
    commented-out `;-D EST_...` lines already there.

### 1.4 CTL_MAX_TILT_RAD — your decision, not a tool's

This is the tilt e-stop threshold: past it, the wheel has no authority left
to recover and driving further only adds energy to the fall. The Build Guide
gives a conservative-actuator envelope of ~11° but says to expect 7–9° once
real friction and gain error are in play. `platformio.ini` already has the
line ready, just commented out:

```
;-D CTL_MAX_TILT_RAD=0.1222
```

Start conservative — 0.1222–0.1396 rad (7–8°) — and widen only once N1/N2
show real headroom. Uncomment it with your chosen value.

---

## 2. Power and connection topology

There are **two separate CAN links** in this project — don't conflate them:

| Link | Used for | Needed while the rig balances? |
|---|---|---|
| fdcanusb dongle ↔ your laptop ↔ moteus | `moteus_tool`, `tview`, calibration, config push | **No** |
| Teensy CAN3 (pins 30/31) ↔ moteus, wired directly | the running control loop | **Yes** — this is the only one the firmware uses |

Once `cube_balancer` is flashed, the **Teensy is its own CAN master** — it
talks to the moteus controller directly over its own wiring.
`TeensyMoteusDriver::initialize()` does **not** push controller
configuration; it expects the moteus board's config (encoder setup, current
limits, `servopos` bounds) to already live in the board's own flash.

**One-time prerequisite, before you rely on that:** confirm the moteus
board's config has actually been persisted with `conf write` (or a host
`moteus_driver` run built with `-DPERSIST_CONFIG_TO_FLASH=ON`). If that
hasn't been done, the moteus board reverts to unconfigured defaults on its
next power cycle, independent of anything the Teensy does — check this once
via the fdcanusb before disconnecting it for good.

So: **yes**, you can flash once and then only need the Teensy's own USB
connection — the fdcanusb / host CAN link plays no role in a running balance
session. What the Teensy's USB *does* still need to provide:

| Need | Why |
|---|---|
| Power (5 V via USB) | Teensy 4.1 draws its logic power from USB unless you wire an external 5 V to `VIN` instead |
| Operator console | arm (`o`), gain-scale ramp (`+`), fault reset (`r`), e-stop (`SPACE`) — all typed on the second CDC serial port |
| Telemetry | binary `TelemetryFrame` records on the first CDC serial port, read with `tools/telemetry/capture.py` |

There is no wireless control path yet — the WiFi telemetry bridge
(`docs/2D_model/XIAO_BRINGUP.md`) only forwards telemetry one-way and has no
command parser or link-loss watchdog. Keep the Teensy tethered to a computer
for every run described below.

**Separately, the moteus controller needs its own motor bus power** (bench
supply or battery) — this is *not* provided by the Teensy's USB at all, and
is its own connection with its own physical e-stop, kept within reach.

---

## 3. Physical setup

- Wheel **off** the motor shaft. Stays off through S6 and N1.
- moteus board connected to its own bench/battery power — separate circuit
  from the Teensy.
- Physical e-stop wired and within reach (software `SetStop()` is not
  enough on its own).
- Teensy CAN3 (pins 30/31) wired to the moteus transceiver.
- Teensy USB plugged into the laptop.
- Run on, or over, a soft surface.

---

## 4. Build in WSL2, flash and monitor from Windows

This repo lives in WSL2, but flashing re-enumerates the Teensy's USB device
(it reboots into the HalfKay bootloader, a different USB device ID than the
running sketch), which drops any `usbipd` attachment. So:

| Task | Where | Command |
|---|---|---|
| Build | WSL2 | `pio run -e <env>` |
| Flash | **Windows** | `pio run -e <env> -t upload` |
| Serial | **Windows** | `pio device monitor` |

Windows reads the repo in place at `\\wsl$\<distro>\<path>` — no second
clone needed.

---

## 5. S6 — prove the loop, no torque, no motor power

```bash
# WSL2:
pio run -e cube_balancer
```
```powershell
# Windows:
pio run -e cube_balancer -t upload
pio device monitor
```

`USB_DUAL_SERIAL` is on for this env, so Windows enumerates **two**
consecutive COM ports: the first carries binary telemetry (open only with
`tools/telemetry/capture.py`, never a plain terminal — it'll look like
garbage and can desync the stream), the second is the text console —
`pio device monitor` should attach to that one.

Confirm, with the wheel and motor power both off:

- The banner reads `=== cube_balancer -- S6, torque path not compiled in ===`
  and boot does **not** halt with "REFUSING TO RUN" (if it does, go back to
  §1/§1.4 — it's naming the field you still need).
- The state machine reaches `IDLE`.
- The loop holds 400 Hz with no missed cycles (visible in telemetry via
  `capture.py` / `tools/telemetry/decode.py`).

This is the "is the code ready to run" checkpoint — it proves the build, the
NaN gate, and the state machine all work before any motor power is applied.

---

## 6. N1 — first torque, wheel still off

```bash
# WSL2:
pio run -e cube_balancer_torque
```
```powershell
# Windows:
pio run -e cube_balancer_torque -t upload
pio device monitor
```

Now apply moteus board power for the first time. It **boots in OBSERVE
MODE regardless** — press `o` at the console to arm real torque output.

With the wheel still off, tilt the panel by hand and confirm the shaft
pushes the **correct** way (same sign check as §1.3 step 7, now with the
real motor as the judge). If it pushes the wrong way:

> Fix `EST_INVERT_THETA` (or `SOURCE0_SIGN`) and reflash. **Never** flip the
> sign of the LQR gains to compensate — that leaves the damping term
> pointing the wrong way and hides the real bug somewhere worse.

Record the result in `../3d_scaling/measurements.md` under "The sign —
record what N1 showed."

---

## 7. N2 — wheel on, tethered, gain ramp

Mount the wheel. Stay tethered, over a soft surface. `gain_scale` boots at
0.1; step it up with `+` through `{0.1, 0.3, 0.6, 1.0}`, one run per step.
Oscillation means too much `k_theta` or too little `k_theta_dot`;
sluggishness is the reverse. Tune `k_omega` **last**, once it balances at
all — raise it only until the wheel drifts back toward zero over a few
seconds without the panel visibly leaning.

---

## Quick reference

```
§1  imu_axis_verify  -> EST_PIVOT_AXIS_X / _Y / _Z / EST_INVERT_THETA / EST_THETA_OFFSET
§1.4                 -> CTL_MAX_TILT_RAD (your safety call)
§5  S6   pio -e cube_balancer          wheel off, motor power off
§6  N1   pio -e cube_balancer_torque   wheel off, motor power ON, 'o' to arm
§7  N2   wheel on, gain_scale ramp 0.1 -> 0.3 -> 0.6 -> 1.0
```

Throughout §5–§7: fdcanusb stays unplugged; Teensy USB stays plugged into a
computer the whole time (power + console + telemetry).
