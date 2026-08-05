# Control Integration Specification

How the control code being written right now — `StateEstimator::update()`,
`StateEstimator::accelAngle()`, `BalancingController::update()` — gets inserted into this
repository and consumed by the loop.

**This document does not implement those three functions.** It specifies the contract they
must satisfy, where the code goes, what calls it and in what order, what each caller
assumes, and how you prove the insertion worked before any torque is commanVded.

Related:
[`ARCHITECTURE.md`](ARCHITECTURE.md) — what the system looks like and why ·
[`INTEGRATION.md`](INTEGRATION.md) — the Teensy port, step by step ·
[`../3d_scaling/README.md`](../3d_scaling/README.md) — measuring the rig, computing the gains

---

# ★ QUICK GUIDE

Everything below this section is detail. This part is the map.

## What you are inserting, in one picture

```
   ┌──────────────────────────────────────────────────────────────────┐
   │  ALREADY WRITTEN — do not modify while inserting                 │
   │                                                                  │
   │   ImuDriver ──ImuData──┐                                         │
   │                        │                                         │
   │   MoteusDriver ──MotorState──┐                                   │
   └────────────────────────┼─────┼──────────────────────────────────-┘
                            │     │
              ┌─────────────▼─────▼──────────────┐
              │  ►► YOUR CODE GOES HERE ◄◄       │
              │                                  │
              │  StateEstimator::update()        │  ← 3 functions
              │  StateEstimator::accelAngle()    │    2 files
              │  BalancingController::update()   │    0 new files
              └─────────────┬────────────────────┘
                            │ BodyState + ControlOutput
   ┌────────────────────────▼─────────────────────────────────────────┐
   │  ALREADY WRITTEN                                                 │
   │   cube_balancer loop ── sendTorque() ── moteus                   │
   └──────────────────────────────────────────────────────────────────┘
```

**You add no files, no classes, no headers, and no build targets.** Three function bodies
replace three `TODO(you)` blocks in two existing `.cpp` files. Everything around them —
the loop, the safety latch, the config gate, the CMake targets — is already written and
already compiles.

## The three insertion points

| # | Function | File | Currently returns |
|---|---|---|---|
| 1 | `accelAngle()` | [`src/core/StateEstimator.cpp:67`](../../src/core/StateEstimator.cpp#L67) | `NAN` |
| 2 | `StateEstimator::update()` | [`src/core/StateEstimator.cpp:92`](../../src/core/StateEstimator.cpp#L92) | `state_` with `valid=false` |
| 3 | `BalancingController::update()` | [`src/core/BalancingController.cpp:59`](../../src/core/BalancingController.cpp#L59) | disarmed, `kNotReady`, 0 Nm |

Write them **in that order**. Each is testable before the next exists.

## The five rules

Ordered by how much damage breaking them does.

1. **Never `#include` anything from `drivers/` in `core/`.** Enforced by the build —
   `cube_core` links no driver sources, so a violation is a *link error*, not a review
   comment. Also: no `<iostream>`, no `printf`, no `new`/`malloc`, no exceptions.
2. **Never widen `Config`, `BodyState` or `ControlOutput` to make your code fit.** Every
   field you need already exists. If one seems missing, see §7 before adding it — the
   sizes are asserted against the Teensy build (`72` / `56`).
3. **Never replace a NaN sentinel with a plausible default.** The refuse-to-run gate is a
   safety property, not a nuisance. §3.
4. **Never flip the negative sign in the control law** to fix a cube that drives its own
   fall. The bug is upstream in `invert_theta` or `SOURCE0_SIGN`. §5.3.
5. **Never return non-finite values.** A NaN reaching `feedforward_torque` is not a no-op;
   it is an invalid CAN command. §5.4.

## Insertion sequence

```
 ┌─ I1  Write accelAngle()          → verify with imu_test, no motor  ─┐
 │  I2  Write StateEstimator        → verify with imu_test, no motor   │  no torque
 │  I3  Measure the rig             → §3 of 3d_scaling                 │  possible at
 │  I4  Compute the LQR gains       → §4 of 3d_scaling                 │  any point
 │  I5  Populate the Config structs → the NaN gate goes quiet          │
 │  I6  Write BalancingController   → verify with --dry-run           ─┘
 │  I7  Host-side regression test   → desktop, no hardware
 └─ I8  Hand off to N1              → INTEGRATION.md, wheel OFF, first torque
```

I1–I7 command **zero torque**. The first torque a human is responsible for is N1 in
[`INTEGRATION.md`](INTEGRATION.md), with the wheel physically removed.

---
---

# 1. Where the code goes

## 1.1 The two files you edit

Both already exist, already build, already link into `cube_core`.

```
src/core/StateEstimator.cpp        ← 2 TODO blocks
src/core/BalancingController.cpp   ← 1 TODO block
```

Each `TODO(you)` block is a numbered derivation ending in `(void)` casts of the unused
parameters and a deliberately inert return. **Replace the block; keep the comments that
explain why.** Delete the `(void)` casts as the parameters become used — they exist only
to silence `-Wunused-parameter` while the body is empty.

## 1.2 What you do not touch

| File | Why it stays as-is |
|---|---|
| `include/core/StateEstimator.hpp` | class shape, `Config`, doc comments — all final |
| `include/core/BalancingController.hpp` | ditto, plus `SafetyState` and `ControlOutput` |
| `include/core/Types.hpp` | `ImuData` / `MotorState` / `BodyState` are ABI-fixed. §7 |
| `apps/cube_balancer.cpp` | the loop already calls you correctly |
| `CMakeLists.txt` | no new target, no new source, no new flag |
| everything in `drivers/`, `testing/` | not in scope for control work |

Editing a header is the signal that something has gone wrong with the plan. Stop and read
§7 before you do it.

## 1.3 The layering rule, mechanically

`cube_core` is a static library built **only** from `src/core/`. It links against nothing.
So:

```cpp
#include "core/Types.hpp"        // ✅ fine
#include <cmath>                 // ✅ fine — already included
#include "drivers/ImuDriver.hpp" // ❌ link error, by design
#include <iostream>              // ❌ compiles, but see below
```

`<iostream>` will *compile* — it is the Teensy build and the real-time budget that reject
it. §6.2.

---

# 2. The data contract

What arrives, what leaves, and what each field is guaranteed to mean. Types are in
[`include/core/Types.hpp`](../../include/core/Types.hpp).

## 2.1 Inputs

**`ImuData`** — produced by `ImuDriver` (Linux/I2C today, `Bmi270SpiDriver` on Teensy).

| Field | Unit | Guarantee |
|---|---|---|
| `accel_x/y/z` | m/s² | SI, sensor frame, gravity **included**. Never raw LSB |
| `gyro_x/y/z` | rad/s | SI, sensor frame, **bias already subtracted** by `calibrateGyroBias()` |
| `temperature_c` | °C | die temp; useful for spotting bias drift, not used by the filter |
| `timestamp` | s | monotonic. **Do not use it to derive `dt`** — see §2.3 |
| `valid` | — | `false` on bus error. **Treat as a hard stop for integration**, not as zero motion |

**`MotorState`** — produced by the moteus driver.

| Field | Unit | Guarantee |
|---|---|---|
| `position_rev`, `velocity_rev_s` | rev, rev/s | at the **output**. Direct-drive (ratio 1.0), so also the rotor |
| `velocity_rad_s()` | rad/s | inline convenience — **this is what the control law wants** |
| `torque_nm` | Nm | the board's estimate of delivered torque, not your command |
| `fault` | — | `0` = healthy. Any other value has latched on the board until a stop |
| `valid` | — | `false` if the controller did not reply to the last command |

The board already PLL-filters encoder velocity at 400 Hz
(`motor_position.sources.0.pll_filter_hz`). **Do not filter `wheel_omega` again** — a
second filter only adds phase lag to the term whose whole job is to react before the wheel
saturates.

## 2.2 Outputs

**`BodyState`** — what your estimator returns.

| Field | Unit | Contract |
|---|---|---|
| `theta` | rad | tilt from the **balance point** (offset already removed), sign per §5.3 |
| `theta_dot` | rad/s | tilt rate, low-passed per `rate_cutoff_hz` |
| `wheel_omega` | rad/s | copied from `motor.velocity_rad_s()`, unfiltered |
| `valid` | — | `false` until `sample_count_ >= warmup_samples`. Gates the controller |

**`ControlOutput`** — what your controller returns.

| Field | Contract |
|---|---|
| `torque_nm` | **must be `0.0` whenever `armed == false`**. The loop trusts this |
| `armed` | `false` = latched trip. Only `reset()` clears it |
| `safety` | the `SafetyState` that caused it. First trip wins |
| `term_theta`, `term_theta_dot`, `term_omega` | **pre-clamp**, for tuning. Populate even when clamped |
| `torque_clamped` | `true` if the clamp actually bit this cycle |

The three `term_*` fields are not decoration. During N2 tuning they are how you see which
term dominates and which one saturated; leaving them zero makes gain tuning blind.

## 2.3 `dt` comes from the loop, not from the timestamps

`update()` receives `dt` as a parameter. `cube_balancer` measures it between iterations of
its own clock ([`cube_balancer.cpp:254-256`](../../apps/cube_balancer.cpp#L254-L256)) and
passes the **measured** value, never the nominal period.

Use that parameter. Do not compute `dt` from `imu.timestamp` deltas:

- the two clocks are not the same on Teensy (`micros()` vs. the loop's counter),
- a repeated sample would yield `dt == 0` and a division by zero in the filter,
- and the rollover hazard (`INTEGRATION.md` risk #4) is already handled loop-side.

**Validate it anyway.** `dt <= 0 || dt > 0.1` means a missed or duplicated cycle;
integrating a 5-second `dt` injects a step into `theta` that the filter spends seconds
unwinding. Reject and hold, per step 2 of the derivation.

---

# 3. Configuration: the NaN gate

## 3.1 Why the sentinels exist

Every tunable in both `Config` structs ships as `NAN` (or `-1` for the integers). This is
deliberate and is a **safety property, not a placeholder convention**:

> NaN comparisons are all false. An unset `max_tilt_rad` would make
> `fabs(theta) > max_tilt_rad` silently evaluate false forever — the tilt e-stop would not
> fail loudly, it would **cease to exist**.

So the sentinels are caught up-front instead:

- `firstUnsetField()` returns the name of the first sentinel still in place — already
  implemented in both classes, and **not** part of what you write.
- `cube_balancer` calls both before constructing anything that can move the motor
  ([`cube_balancer.cpp:141-173`](../../apps/cube_balancer.cpp#L141-L173)) and refuses to
  start, naming the field.
- `BalancingController::update()` re-checks `isConfigured()` as its very first action and
  returns `kUnconfigured`, disarmed — so even a caller that skipped the gate cannot command
  torque.

## 3.2 What you must fill in before I6

**`StateEstimator::Config`** — 8 fields, all from the rig, none from the LQR solve:

| Field | Source | Note |
|---|---|---|
| `tau` | start ~0.5 s | crossover between gyro and accel |
| `gyro_axis`, `accel_axis_a`, `accel_axis_b` | how the IMU is bolted on | `0`=x, `1`=y, `2`=z |
| `invert_theta` | **verify on the rig** | §5.3 — the most dangerous value in the project |
| `theta_offset` | measured with `imu_test` | balance by hand, read the angle |
| `rate_cutoff_hz` | 10–20 Hz | a *frequency*, not a coefficient — §4.4 |
| `warmup_samples` | ~20 at 200 Hz | 100 ms settling |
| `accel_gate` | measured | fraction of g; ~0.25 to start |

**`BalancingController::Config`** — 7 fields; the first three come from the LQR solve:

| Field | Source |
|---|---|
| `k_theta`, `k_theta_dot`, `k_omega` | [`../3d_scaling/README.md`](../3d_scaling/README.md) §4 |
| `max_tilt_rad` | recoverable envelope; 0.2618 (15°) conservative start |
| `max_torque_nm` | `K_t × servo.max_current_A` — **≈0.38 Nm at 15 A**, see §3.3 |
| `max_wheel_omega` | well under no-load; ~300 rad/s |
| `theta_deadband` | just above measured `theta` noise floor; 0.002 rad ≈ 0.11° |

## 3.3 The torque ceiling mismatch — reconcile before I6

`kv = 373.6` ⟹ `K_t = 60/(2π·kv) ≈ 0.0256 Nm/A`. So:

| `servo.max_current_A` | Real ceiling |
|---|---|
| 15 A | ≈ **0.38 Nm** |
| 8 A | ≈ **0.20 Nm** |

The build default `MAX_TORQUE_NM = 0.5` **exceeds both**. Setting
`Config::max_torque_nm` above the real ceiling means the control law believes it has
authority it does not have: it will saturate silently, `torque_clamped` will read `false`
because *your* clamp never bit, and the missing authority will look like a gain problem
during tuning.

Set `max_torque_nm` at or below the real ceiling. Raise the current limit only within **the
lower of** the motor's and the board's rating.

## 3.4 The two clamps, and which wins

There are three torque limits in series. They are not redundant:

```
   your Config::max_torque_nm  ──►  MoteusConfig::max_torque_nm  ──►  servo.max_current_A
   (the control law's belief)      (the driver's pass-through)      (the board, in hardware)
```

`cube_balancer` raises the driver's ceiling to match yours if yours is higher
([`cube_balancer.cpp:177-179`](../../apps/cube_balancer.cpp#L177-L179)), specifically so the
driver cannot silently clip below the law's own limit and make the clamp invisible. The
board's current limit is the one that cannot be argued with, and it is the one that
protects hardware.

## 3.5 Overriding at runtime

Five of the values are settable from the command line without a rebuild, which is how you
sweep gains during N2:

```bash
./build/cube_balancer --dry-run \
  --k-theta 0.9 --k-theta-dot 0.08 --k-omega 0.0009 \
  --max-torque 0.20 --theta-offset 0.013
```

The rest must be set in the `Config` structs. Anything you edit into a struct as a literal
should carry a comment saying **where the number came from** — a measurement, a solve, or a
tuning run. That is the house style throughout this repo, and for these fields it is the
difference between a reproducible rig and a magic constant.

---

# 4. The estimator contract

Full derivation is in the `TODO(you)` block at
[`src/core/StateEstimator.cpp:92`](../../src/core/StateEstimator.cpp#L92). This section
specifies only what the *callers* rely on.

## 4.1 Call sites

| Caller | Uses | Cadence |
|---|---|---|
| `cube_balancer` | `update()` | every cycle, 200 Hz |
| `imu_test` | `accelAngle()` | interactive — this is how `theta_offset` is measured |
| host regression test (I7) | both | desktop, synthetic input |

`accelAngle()` is `const` and must stay so: `imu_test` calls it on a live stream without
disturbing filter state. **Write it first** — it is two lines of real work, it has no
history, and it is the only part of the estimator you can validate with nothing but a hand
and a level surface.

## 4.2 State machine

```
  reset() ──► unseeded ──first valid sample──► seeded, warming ──N samples──► valid
                 │                                   │                          │
                 │  theta = 0, valid=false           │  theta tracking,         │  full
                 │                                   │  valid=false             │  output
                 └──────────── invalid sample: HOLD, valid=false ───────────────┘
```

Three properties the loop depends on:

- **Seed on the first valid sample** (`state_.theta = theta_accel`), do not start at zero.
  Starting at zero makes the filter walk to the true angle over several `tau` *while the
  controller is already acting on the wrong estimate*.
- **Hold, don't integrate, on `!imu.valid`.** A dropped read is not evidence the cube
  stopped moving.
- **`valid` gates the controller, and is not a fault.** `BalancingController` maps
  `!state.valid` to `kNotReady` — armed, zero torque, no latch. Latching on it would mean
  the rig could never start.

## 4.3 Where the accelerometer is not trusted

During a hard wheel correction the measured vector is mostly reaction, not gravity. The
gate is a magnitude test against 1 g:

```
trust = fabs(|a| - kGravity) < accel_gate * kGravity
```

Untrusted ⟹ gyro-only prediction that cycle. Blending an untrusted sample drags `theta`
toward whatever direction the cube happened to be accelerating — which, during a
correction, is precisely the wrong way.

`kGravity` is already defined at
[`StateEstimator.cpp:9`](../../src/core/StateEstimator.cpp#L9) as `9.80665` — a defined SI
constant, not a tunable.

## 4.4 Both filter coefficients must be derived from the measured `dt`

```
alpha = tau / (tau + dt)                  // complementary blend
rc    = 1 / (2π · rate_cutoff_hz)
beta  = dt / (dt + rc)                    // theta_dot low-pass
```

Never hardcode `alpha` or `beta`. This is what makes the filter behave identically at
200 Hz and 50 Hz, and what stops a single late cycle from silently changing the crossover
frequency. It is also why `rate_cutoff_hz` is stored as a *frequency* rather than a
coefficient.

**Apply `invert_theta` to the gyro rate as well as to the accelerometer angle.** Inverting
only one half makes the two halves of the filter disagree about which way is positive, and
they will fight each other — a failure that looks like a badly tuned `tau`.

---

# 5. The controller contract

Full derivation at
[`src/core/BalancingController.cpp:83`](../../src/core/BalancingController.cpp#L83).

## 5.1 Ordering is part of the specification

Two guards are **already implemented** and run before your code: `isConfigured()`, then the
`!armed_` latch. Your block continues:

```
   ① motor.valid && motor.fault != 0   ──► trip(kMotorFault)      unrecoverable
   ② !state.valid                      ──► kNotReady, ARMED       normal startup
   ③ |theta| > max_tilt_rad            ──► trip(kTiltExceeded)    the e-stop
   ④ |wheel_omega| > max_wheel_omega   ──► trip(kWheelSaturated)  no authority left
   ⑤ !motor.valid                      ──► trip(kSensorFault)     symptom, not cause
   ─────────────────── only now, the control law ───────────────────
   ⑥ deadband on theta only
   ⑦ three terms, stored in `out`
   ⑧ torque = -(sum)
   ⑨ clamp, record torque_clamped
   ⑩ !isfinite ──► trip(kSensorFault), zero
```

The order is not stylistic. **Whichever check trips first is the diagnosis printed on the
terminal after a crash** — `trip()` implements first-trip-wins
([`BalancingController.cpp:48-55`](../../src/core/BalancingController.cpp#L48-L55)), so a
tilt e-stop followed by the motor faulting on the way down still correctly reports the
tilt. Checking `!motor.valid` early would report a dropped reply as the cause of every
fall.

## 5.2 The deadband applies to `theta` only

```cpp
theta = (fabs(state.theta) < theta_deadband) ? 0.0 : state.theta;
```

Not to `theta_dot`, not to `wheel_omega`. Deadbanding the rate term leaves the cube
**undamped exactly where damping matters most** — near vertical, where it spends
essentially all of its time.

## 5.3 The sign convention, and the one thing not to do

`theta` is positive when the cube tilts toward the direction a **positive wheel torque
corrects**. The law is:

```cpp
torque = -(k_theta*theta + k_theta_dot*theta_dot + k_omega*wheel_omega);
```

The negative sign is what makes this stabilising — torque opposes tilt.

> **If the rig accelerates its own fall, do not flip this sign.** The bug is in
> `Config::invert_theta` or in the encoder sign (`SOURCE0_SIGN`). Flipping the law hides
> the real fault and leaves the **damping term pointing the wrong way** — you get a cube
> that appears to catch itself and then oscillates divergently.

This is verified at N1 with the wheel **physically removed**
([`INTEGRATION.md`](INTEGRATION.md) H4). It is risk #2 in that document's register, and the
reason the wheel comes off at all.

## 5.4 Non-finite output is a trip, not a pass-through

```cpp
if (!std::isfinite(torque)) { trip(SafetyState::kSensorFault); /* zero */ }
```

A NaN reaching moteus's `feedforward_torque` is **not** a no-op — it is an invalid command,
and the board's response to it is not something to characterise on a live rig. NaN gets in
via `0 * inf`, an unseeded filter, or a `dt` that slipped the §2.3 validation.

## 5.5 The `k_omega` term is why this is three states

A cart can push against the ground indefinitely; a reaction wheel trades momentum with the
body and has a hard speed ceiling. Once the wheel saturates there is **no control authority
left** regardless of how good the angle feedback is.

So `k_omega` deliberately leans the cube a fraction of a degree off vertical, letting
gravity accelerate the wheel back toward zero — spending a little tilt to buy back momentum
headroom. Expect it to come out roughly a **thousandth** of `k_theta`, and **tune it last**,
after the rig balances at all.

---

# 6. How the loop consumes your code

## 6.1 The call sequence, per cycle

From [`cube_balancer.cpp:253-313`](../../apps/cube_balancer.cpp#L253-L313), already written:

```cpp
const double dt = now - last;                              // measured, not nominal
const cube::ImuData    sample      = imu.read();
const cube::MotorState motor_state = motor.query();        // query BEFORE commanding
const cube::BodyState  body        = estimator.update(sample, motor_state, dt);
const cube::ControlOutput out      = controller.update(body, motor_state);

if (!dry_run && out.armed)  motor.sendTorque(out.torque_nm);
else if (!out.armed)        motor.stop();                  // latched trip
```

Two ordering facts your code inherits:

- **Query precedes command.** The `wheel_omega` feeding `k_omega` is measured *this* cycle,
  so the law acts on current state rather than the previous cycle's.
- **A latched trip is terminal.** The loop prints the `SafetyState` and **breaks**, rather
  than spinning at 200 Hz doing nothing. `reset()` is a deliberate human act; the balancer
  never calls it automatically.

## 6.2 The real-time budget

5 ms per cycle at 200 Hz. Both your functions run inside it, every cycle, and the loop
measures its own lateness and reports it at exit.

| Not allowed in `core/` | Why |
|---|---|
| `printf` / `<iostream>` | formatting doubles at 200 Hz destroys the budget. `INTEGRATION.md` N3 |
| `new` / `malloc` / `std::vector` | non-deterministic; a heap in a control loop on a 1 MB MCU |
| exceptions | no unwinder budget on bare metal |
| unbounded loops | everything here is O(1); keep it that way |

`std::string` appears in the interfaces **only** in `initialize()`, `calibrateGyroBias()`
and `name()` — all startup-only. The hot path is POD-by-value throughout. Keep it that way.

Trig is fine: the Teensy 4.1 has a hardware double-precision FPU (VFPv5), which is why
`core/` is `double` throughout and needs no `float` rewrite. `atan2` and `sqrt` at 200 Hz
are not the problem.

## 6.3 Both builds compile the same source

```bash
cmake --build build            # host: cube_core, linked into cube_balancer
pio run -e board_check         # Teensy: same src/core/*.cpp, referenced in place
```

**`core/` sources are referenced, never copied.** A copy means the host and Teensy control
laws drift and you end up debugging the difference between your simulation and your
hardware.

> **If you find yourself writing `#ifdef ARDUINO` under `include/core/` or `src/core/`,
> stop.** That is precisely the failure the architecture exists to prevent, and it is
> check S1 in [`INTEGRATION.md`](INTEGRATION.md).

---

# 7. Extending the types (read before you do)

The types are ABI-fixed: S1 asserts `sizeof(ImuData) == 72` and `sizeof(MotorState) == 56`
on both host and Teensy, which is how the two builds prove they agree about `double`.

Before adding a field, check the alternatives:

| You want | Do this instead |
|---|---|
| more filter state (integrator, history) | private member of `StateEstimator` — invisible to the ABI |
| an extra tuning knob | a `Config` field, and extend `firstUnsetField()` in the same commit |
| more telemetry | `ControlOutput` — it is an output struct, not a wire format |
| a second IMU axis / 3D | that is the 3D extension, [`../3d_scaling/README.md`](../3d_scaling/README.md) |

If a `Config` field genuinely must be added: give it a **sentinel**, add it to
`firstUnsetField()` in the **same commit**, and write the three-part comment the existing
fields carry — *what it does* / *how to choose it* / `TODO(you)`. A `Config` field without a
sentinel is a silent hole in the refuse-to-run gate.

Adding to `ImuData` or `MotorState` means updating the S1 size assertion and both drivers.
Rarely the right answer.

---

# 8. Verification — how you know the insertion worked

Each stage is checkable **before** the next exists. No stage below commands torque.

## I1 — `accelAngle()`

*No motor. No filter state. This is the only thing being tested.*

```bash
./build/imu_test
```

| Check | Pass criterion | If it fails |
|---|---|---|
| Returns a number | not `NAN` | the sentinel path is still live |
| Level rig | reads ≈ `-theta_offset`, stable | axis pair `a`/`b` is wrong |
| Tilt one way | moves **one consistent direction** | swap `accel_axis_a` / `accel_axis_b` |
| Tilt ±45° | magnitude tracks roughly linearly | using `asin` instead of `atan2` |
| Balance by hand | **record this number → `theta_offset`** | — |

Set `theta_offset` from this step. A nonzero true offset left at `0.0` makes the cube
converge to a permanent lean instead of upright.

## I2 — `StateEstimator::update()`

*Still no motor.*

| Check | Pass criterion |
|---|---|
| Warmup | `valid` is `false` for `warmup_samples`, then `true` and stays true |
| Seeding | first valid `theta` ≈ `accelAngle()`, **no ramp from zero** |
| Static drift | `theta` stable within noise over 5 min held still — catches gyro-bias-only integration |
| Dynamic tracking | fast hand tilt tracks with **no visible lag**, settles without overshoot |
| `dt` robustness | injected `dt = 0`, `dt = -1`, `dt = 5.0` → held estimate, no NaN, no step |
| `!imu.valid` | estimate **held**, `valid=false`, nothing integrated |
| Accel gate | shake it hard: `theta` does not lurch toward the shake direction |
| **Sign** | tilt toward where a **positive** torque corrects → `theta` **positive**. Otherwise set `invert_theta` |

That last row is the single most dangerous unset value in the project. Verify it before a
motor is ever enabled.

## I3–I5 — measure, solve, populate

Physical parameters ([`../3d_scaling/README.md`](../3d_scaling/README.md) §3), then the LQR
solve (§4), then fill both `Config` structs.

**Check:** `./build/cube_balancer --dry-run` no longer prints a `firstUnsetField()` name.
While it still does, that name *is* your to-do list.

Sanity-check the solve before trusting it: `k_theta` should be the largest of the three by a
wide margin and positive; `k_theta_dot` roughly an order of magnitude below it; `k_omega`
roughly a thousandth of it. A solve that violates this pattern is usually a units error in
the measured inertia.

## I6 — `BalancingController::update()`

```bash
./build/cube_balancer --dry-run
```

Dry-run executes the **entire** loop — estimator, controller, safety, timing — and simply
never calls `sendTorque()`. It is the last stage before torque.

| Check | Pass criterion |
|---|---|
| Nominal | `theta ≈ 0` upright → `torque ≈ 0`, `safety = OK`, armed |
| Deadband | small tilt inside `theta_deadband` → exactly `0.0` |
| Proportionality | larger tilt → larger opposing torque, **sign opposes tilt** |
| Terms populated | all three `term_*` nonzero when the state is nonzero |
| Clamp | force a large tilt → `torque` at `±max_torque_nm`, `torque_clamped = true` |
| Tilt e-stop | past `max_tilt_rad` → latches, `kTiltExceeded`, **0 Nm**, loop breaks |
| Latch holds | after a trip, returning upright does **not** re-arm |
| Fault priority | inject `fault != 0` **and** excess tilt → reports `kMotorFault` (checked first) |
| `kNotReady` | during warmup → armed, 0 Nm, **no latch** |
| Non-finite | inject NaN `theta` → `kSensorFault`, 0 Nm, no NaN reaches the driver |
| Timing | `>99%` of cycles meet the 200 Hz deadline |

## I7 — Host-side regression

The reason the dual build exists: `core/` compiles on the desktop with no hardware, so the
control law can be regression-tested after every change. Feed synthetic `ImuData` /
`MotorState` sequences and assert on `BodyState` / `ControlOutput`.

Worth covering permanently, because these are the cases hardware testing reproduces badly:
`dt` pathologies, the `!imu.valid` hold, gate rejection, every latch path, first-trip-wins
ordering, and clamp behaviour at the boundary.

## I8 — Handoff

At this point the software is done and the remaining risk is physical. Go to
[`INTEGRATION.md`](INTEGRATION.md) **N1**: wheel off, motor power on for the first time,
watchdog disconnect test, then N2 at ~30% gains, tethered, on a soft surface, `k_omega`
tuned last.

---

# 9. Definition of done

The control code is correctly inserted when **all** of these hold:

- [ ] Three `TODO(you)` blocks replaced; the explanatory comments retained
- [ ] Zero files added; zero headers modified; zero `CMakeLists.txt` diff
- [ ] `cmake --build build` clean under `-Wall -Wextra`, all four host binaries
- [ ] `pio run -e board_check` links the **same** `src/core/*.cpp`, unmodified
- [ ] No `#ifdef ARDUINO` anywhere under `core/`
- [ ] No `drivers/` include, no `iostream`, no heap, no exception in `core/`
- [ ] Every `Config` sentinel replaced by a value whose **origin is documented in a comment**
- [ ] `max_torque_nm` ≤ `K_t × servo.max_current_A` (§3.3)
- [ ] `--dry-run` names no unset field and passes every I6 check
- [ ] Sign convention verified on the rig (I2 last row) — **before** any motor power
- [ ] `torque_nm == 0.0` on every disarmed path, verified
- [ ] Host regression tests cover the latch paths and `dt` pathologies
- [ ] README.md / ARCHITECTURE.md updated if any documented behaviour changed
