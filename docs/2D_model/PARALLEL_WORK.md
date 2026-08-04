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

## 2. Determine the IMU axis mapping — from geometry

`StateEstimator::Config` has three axis fields, all unset (`-1`):

```cpp
int gyro_axis    = -1;      // which gyro axis measures the tilt RATE
int accel_axis_a = -1;      // the two accel axes gravity swings between
int accel_axis_b = -1;
bool invert_theta = false;  // sign convention
```

You can work these out **from how the IMU is mounted**, before it ever powers on.

The rule: the cube tips about one edge. That edge defines the rotation axis.

- **`gyro_axis`** = the sensor axis parallel to the pivot edge
- **`accel_axis_a` / `_b`** = the other two — gravity swings in their plane

Worked example from the header: if the cube tips about the sensor's X axis, gravity
swings in the Y-Z plane, so `gyro_axis = 0`, `accel_axis_a = 1`, `accel_axis_b = 2`.

**Write down which way the IMU is glued** — which chip axis points where relative to the
cube. Step 4 of [IMU_SETUP.md](IMU_SETUP.md) then *confirms* it rather than discovering
it, which is much faster.

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
