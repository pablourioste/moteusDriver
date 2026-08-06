# Work that needs no Teensy

Tasks you can complete while hardware is being wired, or while waiting on parts.
Ordered by how much they unblock.

None of these need the Teensy powered, the CAN bus connected, or the IMU wired.

---

## ⚠ First: the usbipd question

If someone hands you this list:

```bash
winget install --interactive --exact dorssel.usbipd-win
sudo usermod -a -G dialout $USER
wsl --shutdown          # needs restart
```

**You do not need any of it for the Teensy.** Teensy flashing happens on the *Windows*
side (see [INSTALL.md](../INSTALL.md) Part B4), so the board never crosses into WSL —
no usbipd, no dialout, no restart.

Those commands are for the **fdcanusb**, which does need USB passthrough into WSL for
`moteus_tool` and `tview`. Even then, check before running:

```bash
groups | tr ' ' '\n' | grep -x dialout          # already a member? skip the usermod
"/mnt/c/Program Files/usbipd-win/usbipd.exe" --version   # already installed? skip winget
```

`wsl --shutdown` is only needed to make a *new* group membership take effect. If you are
already in `dialout`, skip it — and it is a WSL restart, not a Windows reboot.

---

## 1. Measure the physical parameters ← highest value

**This blocks everything.** No LQR gains without these numbers, no
`cube_balancer`, no N1, no balancing. It is also the one task that needs **zero
electronics** — a scale, a ruler, a knife edge and a stopwatch.

Full procedure: [`../3d_scaling/README.md`](../3d_scaling/README.md) §3.

| Symbol | Meaning | How |
|---|---|---|
| `m_b` | body mass, excluding wheel | scale |
| `m_w` | wheel mass | scale |
| `l` | pivot → body centre of mass | balance the body on a knife edge |
| `l_w` | pivot → wheel axis | ruler |
| `I_b` | body inertia about the pivot edge | swing test, below |
| `I_w` | wheel inertia about its own axis | `½·m_w·r²` for a solid disc |

**Swing test.** Hang the cube from its pivot edge, swing at small amplitude, time 20
oscillations, divide:

```
I_b = (m_b · g · l · T²) / (4π²)
```

Do it with the wheel mounted but **locked**, then subtract `m_w · l_w²`. Time it twice
and average — 5% accuracy is plenty.

**Record the numbers in a file as you go.** They are needed again in §4, and remeasuring
because nobody wrote them down is a bad afternoon.

### Then compute the gains

With the parameters in hand, §4 has a runnable `scipy` Riccati solve producing
`k_theta`, `k_theta_dot`, `k_omega` — plus sanity checks and a staged tuning procedure.
Pure desktop Python. No hardware.

That turns three of the `NaN` sentinels into real numbers.

---

## 2. Determine the IMU pivot axis — from geometry, if the mount allows it

`StateEstimator::Config` takes the pivot direction as a unit vector, all unset (`NAN`):

```cpp
double pivot_axis_x = NAN;  // physical balance edge, in the IMU's own frame
double pivot_axis_y = NAN;
double pivot_axis_z = NAN;
bool invert_theta = false;  // sign convention
```

The rule: the cube tips about one edge. That edge defines the rotation axis, and
`pivot_axis` is that direction expressed in the sensor's own coordinates.

**If the IMU is bolted flush to a cube face**, you can work this out from the drawing
before it ever powers on: `pivot_axis` is just a raw sensor axis. Worked example from
the header — if the cube tips about the sensor's X axis, gravity swings in the Y-Z
plane, so `pivot_axis = (1, 0, 0)`.

**If the IMU sits at any angle on the panel — the normal case, not the exception —**
no amount of reading the drawing will find it: no raw sensor axis is parallel to the
edge, so there is nothing to read off. That has to be *measured* on the assembled rig
with `firmware/imu/imu_axis_verify` (`pio run -e imu_axis_verify`), which fits
`pivot_axis` from three static poses — equilibrium, left limit, right limit — rather
than guessed from geometry. See its header comment, or
[BENCH_RUN_PROCEDURE.md §1](BENCH_RUN_PROCEDURE.md#1-find-the-pivot-axis-and-sign-on-the-mounted-rig).

**Write down which way the IMU is glued** either way — which chip axis points roughly
where relative to the cube. It's a useful sanity check against whatever
`imu_axis_verify` fits, even when the precise vector has to come from measurement.

> `invert_theta` still needs the real sensor — it depends on sign conventions you cannot
> resolve on paper. Leave it for Step 4.

---

## 3. Pre-stage the IMU test procedure

Read [IMU_SETUP.md](IMU_SETUP.md) end to end once before touching hardware. Specifically:

- **Print the final checklist** and have it on the bench
- **Identify your breakout's pin labels.** Naming varies: some boards label SPI pins for
  I2C use (`SDA`→SDI, `SCL`→SCK, `SDO`→MISO, `CS`→CSB). Work this out now, not with a
  multimeter in one hand
- **Confirm the breakout is 3.3 V** and find its VDD pin
- **Prepare a flat, level surface** for calibration — a spirit level helps

---

## 4. Review the CAN wiring before it is powered

Andrea is wiring the CAN connector. Two checks from
[INTEGRATION.md](INTEGRATION.md) H1/H3 are worth doing *before* anything is energised:

- [ ] **Transceiver is 3.3 V logic.** Check the datasheet's *logic supply* rating, not
      just its bus rating. A 5 V MCP2551 destroys Teensy CAN3 — unrecoverably, the pin
      is inside the SoC
- [ ] **Pins 30 (TX) / 31 (RX).** CAN3 is the only FD-capable module on this silicon;
      CAN1/CAN2 cannot talk to moteus at all
- [ ] **Termination: ~60 Ω** measured CANH↔CANL, everything unpowered (two 120 Ω in
      parallel). 120 Ω means a terminator is missing; 40 Ω means there are three
- [ ] **Common ground** Teensy ↔ transceiver ↔ moteus

Catching a wrong transceiver now costs a part swap. Catching it after power costs a
Teensy.

---

## 5. Host-side work that still builds today

The CMake build is untouched by the port and still runs:

```bash
cmake --build build
ctest --test-dir build --output-on-failure
```

- **Implement `StateEstimator::update()` and `accelAngle()`.** The 8-step derivation is
  in the `.cpp` as numbered TODOs, each annotated with the specific failure it prevents.
  Pure math — testable on the desktop with synthetic input, no hardware
- **Implement `BalancingController::update()`** — safety envelope ordering, then `u = -Kx`
- **Write a simulated `IImuSensor`.** [ARCHITECTURE.md](ARCHITECTURE.md) §7 names this as
  an extension point: synthetic pendulum dynamics behind the same interface lets you
  exercise the whole control path with no rig at all. This is the cheapest way to test a
  control law without risking hardware

---

## Suggested order

```
   1. Physical parameters + LQR gains     ← unblocks the most, needs no electronics
        │
        ├── 2. IMU axis mapping from geometry
        │
        ├── 5. StateEstimator / BalancingController (desktop, testable)
        │
        └── 3. Pre-read IMU_SETUP, print the checklist
                │
                └── 4. Review Andrea's CAN wiring BEFORE power
```

When the Teensy is ready, you resume at [IMU_SETUP.md](IMU_SETUP.md) **Step 0.4**
(blink) — Step 0.3 already passed.
