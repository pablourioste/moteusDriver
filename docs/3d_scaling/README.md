# From 1D edge balancing to 3D corner balancing

How this codebase gets from one reaction wheel on one edge to three wheels
balancing on a corner, in the manner of ETH Zurich's Cubli.

This is also the reference for the parts of the 1D rig that are deliberately
left unfinished: the LQR gains, the BMI270 config blob, and the physical
parameters neither can be computed without.

---

## Contents

1. [Prerequisites before any of this matters](#1-prerequisites)
2. [The BMI270 configuration blob](#2-the-bmi270-configuration-blob)
3. [Measuring the physical parameters](#3-measuring-the-physical-parameters)
4. [Computing real LQR gains for 1D](#4-computing-real-lqr-gains-for-1d)
5. [What changes in 3D](#5-what-changes-in-3d)
6. [Hardware scaling](#6-hardware-scaling)
7. [Software changes, file by file](#7-software-changes-file-by-file)
8. [Bring-up order for the 3D rig](#8-bring-up-order)
9. [References](#9-references)

---

## 1. Prerequisites

Do not start on 3D until the 1D rig balances reliably for minutes at a time.
Every problem in 3D is a 1D problem times three plus coupling, and an
unresolved 1D issue becomes three unresolved issues that interact.

Specifically, all of these must hold first:

- `direct_motor_test --run-all` passes every check, including the watchdog.
- `imu_test` reports a stable gyro bias and a tilt angle that reads ~0° at
  the balance point and has the correct sign.
- `cube_balancer` holds the edge balance through a hand disturbance without
  the wheel saturating.
- You have measured, not guessed, the inertias in §3.

---

## 2. The BMI270 configuration blob

**The IMU will not produce data until this is done.** The BMI270 holds its
accelerometer and gyroscope disabled after power-on until an ~8 KB
configuration blob has been uploaded into its internal core. Until then the
chip ID reads back correctly and every data register reads zero — which looks
exactly like a wiring fault and is not one.

The blob is Bosch's binary microcode, distributed under Apache 2.0 with their
sensor API.

**This has already been done** — `third_party/bmi270_config.h` is present
(8192 bytes, extracted from upstream) and the build was verified with
`-DBMI270_CONFIG_HEADER=ON`. Build with:

```bash
cmake -S . -B build -DBMI270_CONFIG_HEADER=ON && cmake --build build
```

The rest of this section records how the file was produced, so it can be
regenerated or updated against a newer upstream:

```bash
mkdir -p third_party
curl -sL https://raw.githubusercontent.com/boschsensortec/BMI270-Sensor-API/master/bmi270.c \
  -o /tmp/bmi270.c

# Extract the bmi270_config_file[] array into a header.
python3 - <<'PY'
import re
src = open('/tmp/bmi270.c').read()
m = re.search(r'const uint8_t bmi270_config_file\[\]\s*=\s*\{(.*?)\};', src, re.S)
if not m:
    raise SystemExit('array not found - upstream layout changed')
body = m.group(1)
n = len(re.findall(r'0x[0-9a-fA-F]{2}', body))
with open('third_party/bmi270_config.h', 'w') as f:
    f.write('// Extracted from BMI270-Sensor-API (Apache-2.0), Bosch Sensortec.\n')
    f.write('#pragma once\n#include <cstdint>\n\n')
    f.write('static const uint8_t bmi270_config_file[] = {')
    f.write(body)
    f.write('};\n')
print(f'wrote third_party/bmi270_config.h with {n} bytes')
PY

cmake -S . -B build -DBMI270_CONFIG_HEADER=ON
cmake --build build
```

Expect roughly 8192 bytes. If the count is far off, the upstream file layout
changed and the regex needs revisiting.

`ImuDriver::uploadConfigBlob()` implements the datasheet sequence: disable
advanced power save, `INIT_CTRL=0`, burst-write the blob through
`INIT_ADDR_0/1` + `INIT_DATA`, `INIT_CTRL=1`, then poll `INTERNAL_STATUS`
until its low nibble reads `0b0001`.

---

## 3. Measuring the physical parameters

The LQR gains cannot be computed without these, and they cannot be guessed to
within a factor that matters. All are for the 1D case; §5 extends them.

| Symbol | Meaning | Unit | How to get it |
|---|---|---|---|
| `m_b` | body mass, excluding wheel | kg | scale |
| `m_w` | wheel mass | kg | scale |
| `l` | pivot to body centre of mass | m | balance the body on a knife edge |
| `l_w` | pivot to wheel axis | m | measure |
| `I_b` | body inertia about the pivot edge | kg·m² | pendulum test, below |
| `I_w` | wheel inertia about its own axis | kg·m² | see below |
| `K_t` | motor torque constant | Nm/A | `60 / (2π · kv)` |

### Body inertia by swing test

Let the cube hang from its pivot edge and swing at a small amplitude. Time
20 oscillations and divide. For a physical pendulum:

```
I_b = (m_b · g · l · T²) / (4π²)
```

Do this with the wheel mounted but locked, then subtract `m_w · l_w²`. Ten
swings timed twice, averaged, is plenty — this only needs ~5% accuracy.

### Wheel inertia

For a solid disc, `I_w = ½ · m_w · r²`. For a spoked or rimmed wheel, either
compute it from CAD or measure it: spin it up with a known torque and time
the deceleration against a known friction estimate.

From the calibration log, this motor has `kv = 373.6`, so:

```
K_t = 60 / (2π · 373.6) ≈ 0.0256 Nm/A
```

With `servo.max_current_A = 8`, the torque ceiling is about **0.20 Nm** —
which is *below* the 0.5 Nm currently set as `MAX_TORQUE_NM`. That mismatch
matters: the control law believes it has 0.5 Nm of authority and does not.
Either raise the current limit (within the board's and motor's rating) or
lower `--max-torque` to the real ceiling.

---

## 4. Computing real LQR gains for 1D

### Linearised model

State `x = [θ, θ̇, ω_w]`, input `u = τ` (motor torque, Nm), linearised about
the upright equilibrium:

```
θ̈ = (m_total · g · l / I_total) · θ  −  (1 / I_total) · τ
ω̇_w = (1/I_w + 1/I_total) · τ
```

where `I_total = I_b + m_w · l_w²`. In state-space form:

```
      ⎡ 0                        1    0 ⎤        ⎡      0        ⎤
  A = ⎢ m_total·g·l / I_total    0    0 ⎥    B = ⎢ −1 / I_total  ⎥
      ⎣ 0                        0    0 ⎦        ⎣ 1/I_w + 1/I_total ⎦
```

The positive `(2,1)` entry is the open-loop instability — the cube falls.

### Solving for K

```python
import numpy as np
from scipy.linalg import solve_continuous_are

# --- measured, per section 3 ---
m_b, m_w = 0.9, 0.2       # kg
l, l_w   = 0.06, 0.08     # m
I_b      = 3.5e-3         # kg m^2, about the pivot edge
I_w      = 1.2e-4         # kg m^2
g        = 9.80665

m_total = m_b + m_w
I_total = I_b + m_w * l_w**2

A = np.array([[0.0,                          1.0, 0.0],
              [m_total*g*l / I_total,        0.0, 0.0],
              [0.0,                          0.0, 0.0]])
B = np.array([[0.0], [-1.0/I_total], [1.0/I_w + 1.0/I_total]])

# Q penalises state error, R penalises control effort.
# Start here, then tune: raising Q[0,0] stiffens the angle response;
# raising Q[2,2] makes the cube work harder to keep the wheel near zero;
# raising R makes the whole thing gentler.
Q = np.diag([100.0, 1.0, 0.01])
R = np.array([[1.0]])

P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P
print("k_theta     =", K[0,0])
print("k_theta_dot =", K[0,1])
print("k_omega     =", K[0,2])
```

Feed the result in without rebuilding:

```bash
./build/cube_balancer --k-theta 3.2 --k-theta-dot 0.31 --k-omega 0.0021
```

Once they are settled, promote them to the `kDefault*Gain` constants in
`include/core/BalancingController.hpp`.

### Sanity checks on the result

- `k_theta` should be **positive** and comfortably larger than `k_theta_dot`.
- `k_omega` should be **small** — a thousandth or so of `k_theta`. Too large
  and the cube visibly leans away from vertical to bleed wheel speed.
- Verify the closed loop is stable before running: `np.linalg.eigvals(A - B@K)`
  must all have negative real parts.

### Tuning procedure on the rig

1. Start with `--dry-run` and confirm θ and θ̇ look right as you tilt by hand.
2. Run live with gains at ~25% of computed, holding the cube. Feel whether the
   wheel pushes the correct way.
3. Scale up toward 100%. Oscillation means too much `k_theta` or not enough
   `k_theta_dot`; sluggishness the reverse.
4. Only once it balances, raise `k_omega` until the wheel drifts back toward
   zero over several seconds without the cube visibly leaning.

---

## 5. What changes in 3D

### The geometry

Corner balancing means the cube pivots on a single vertex, with three
reaction wheels on mutually orthogonal axes. Two things change fundamentally:

**Tilt stops being a scalar.** In 1D, θ is one number. On a corner, the body's
attitude is a full 3-DOF rotation, and there is no singularity-free
three-parameter representation of it — Euler angles gimbal-lock. The estimator
must move to a **quaternion**, and the state becomes:

```
x = [q_vec (3), ω_body (3), ω_wheels (3)]     — 9 states
u = [τ_1, τ_2, τ_3]                            —  3 inputs
```

where `q_vec` is the vector part of the error quaternion between the current
and upright attitudes. Near upright, `q_vec ≈ ½ · (axis · angle)`, which
linearises cleanly and is why the vector part is used rather than Euler angles.

**The axes couple.** The three wheels are not three independent 1D problems.
Two effects link them:

- *Gyroscopic coupling.* A spinning wheel resists reorientation. With three
  wheels at speed, rotating the body about one axis produces torque about the
  others: `τ_gyro = ω_body × (I_w · ω_wheel)`.
- *Inertia cross-terms.* Unless the cube is perfectly symmetric and the wheels
  perfectly aligned, the inertia tensor has non-zero off-diagonal terms.

A diagonal (three-independent-SISO-loops) controller ignores both and works
adequately near upright at low wheel speed. It degrades exactly when it
matters — during a large correction with fast wheels. Use a full 9-state LQR.

### The control law

```
τ = −K · x        K is 3×9
```

Same Riccati solve, larger matrices. `A` gains the gyroscopic terms, which are
functions of wheel speed, so strictly the system is only linear-time-invariant
at a fixed wheel speed. Two options:

1. **Linearise about zero wheel speed.** Simple, and valid as long as the
   controller keeps the wheels near zero — which `k_omega` exists to do.
2. **Gain scheduling.** Compute `K` at several wheel speeds and interpolate.
   Necessary if the rig does momentum-jump manoeuvres, where wheels spin up
   deliberately.

Start with option 1.

### The jump-up manoeuvre

The Cubli's party trick — spinning all three wheels to high speed, then braking
them simultaneously to fling the cube from flat onto its corner — is a separate
open-loop manoeuvre, not part of the balancing controller. Sequence:

1. Spin wheels to a computed speed (velocity mode, not torque mode).
2. Brake hard and simultaneously; the transferred angular momentum tips the cube.
3. Hand over to the balancing controller once attitude is within its region of
   attraction.

Step 3 is the hard part. The balancing LQR is only valid near upright, so the
handover must happen inside its basin, and the braking must be accurate enough
to land there. Get continuous balancing solid first.

---

## 6. Hardware scaling

| Item | 1D | 3D |
|---|---|---|
| Motors | 1 | 3 |
| CAN IDs | 1 | 1, 2, 3 |
| Encoders | 1 × MA600 on aux2 | 3, one per controller |
| IMU | 1 × BMI270 | 1, but mounted at the geometric centre |
| Power | one bus | three motors' peak draw simultaneously |

Three things bite in practice:

**CAN bandwidth and latency.** Three controllers queried sequentially at 200 Hz
is 600 transactions/s, and `SetPosition` per controller in a loop means three
serial round-trips per cycle. Use `Transport::Cycle()` to issue all three in
one bus transaction — see `moteus/lib/cpp/examples/multiple_cycle.cc`. This is
not an optimisation; at 200 Hz over fdcanusb it is the difference between
meeting and missing the deadline.

**Setting the CAN IDs.** Do it one controller at a time, with only that one on
the bus, using `moteus_tool`. Note that `id.id` is deliberately excluded from
`current_config.cfg.in` — pushing a new ID mid-session disconnects the board
you are talking to.

**IMU placement.** At the geometric centre, so no wheel's reaction shows up as
linear acceleration at the sensor. Off-centre mounting injects wheel torque
directly into the accelerometer, and the estimator reads it as tilt.

---

## 7. Software changes, file by file

The interface split exists precisely so this is contained.

| File | Change |
|---|---|
| `core/Types.hpp` | `BodyState` → quaternion + `ω_body[3]` + `ω_wheels[3]` |
| `core/StateEstimator.*` | complementary filter → quaternion Madgwick/Mahony or an EKF |
| `core/BalancingController.*` | scalar gains → 3×9 `K`; `ControlOutput.torque_nm` → `torque_nm[3]` |
| `core/interfaces/IMotorDriver.hpp` | unchanged — one instance per motor |
| `drivers/MoteusDriverWrapper.*` | add a multi-controller path over `Transport::Cycle()` |
| `drivers/ImuDriver.*` | unchanged |
| `app_cube_balancer.cpp` | three drivers, vector torque, per-axis E-stop |
| `CMakeLists.txt` | `MOTEUS_CONTROLLER_ID` → a list of three IDs |

`IMotorDriver` and `IImuSensor` do not change shape. That is the payoff of the
current structure: the 3D work is concentrated in the estimator and the
controller, and the driver layer is reused as-is.

### E-stop in 3D

The 1D `|θ| > 15°` becomes a limit on the total angle of the error quaternion:

```
tilt_angle = 2 · acos(clamp(q_err.w, −1, 1))
```

A per-axis check is wrong: three axes each at 14° is a 24° total tilt that
would pass. Trip on the total.

---

## 8. Bring-up order

1. **One axis at a time.** Mount all three wheels, but balance on an edge with
   only one active. Confirm each of the three works alone, in the 1D
   configuration. This catches wiring, sign, and encoder errors individually.
2. **Two axes.** Balance on a corner with one wheel disabled and the cube
   constrained to a plane. First point at which coupling appears.
3. **Three axes, held.** Full controller, cube held by hand near the corner
   balance point. Confirm all three wheels respond correctly to disturbances.
4. **Three axes, free.** Release. Expect several falls; a soft landing surface
   and a tether are worth the setup time.
5. **Jump-up.** Only after step 4 is reliable.

Between every step, re-run `imu_test` — an IMU that shifted in its mount
invalidates every gain above it.

---

## 9. References

- **Cubli** — Gajamohan, Merz, Thommen, D'Andrea, *The Cubli: A Cube that can
  Jump Up and Balance*, IROS 2012. The original; §3 covers the momentum-jump
  manoeuvre.
- **Cubli control** — Muehlebach & D'Andrea, *Nonlinear Analysis and Control of
  a Reaction Wheel-based 3D Inverted Pendulum*, CDC 2013. The 9-state model
  and the region of attraction analysis.
- **BMI270** — [Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf)
  ·
  [Sensor API](https://github.com/boschsensortec/BMI270-Sensor-API) (source of
  the config blob)
- **moteus** — [Reference docs](https://mjbots.github.io/moteus/) ·
  `moteus/docs/reference/` in this tree ·
  `moteus/lib/cpp/examples/multiple_cycle.cc` for the multi-controller pattern
- **Attitude estimation** — Madgwick, *An efficient orientation filter for
  inertial and inertial/magnetic sensor arrays*, 2010.
