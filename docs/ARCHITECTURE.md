# Architecture

Design rationale and the contracts between layers. For a first orientation
read the [root README](../README.md); this document is the deeper reference.

---

## 1. The layering rule

```
apps/  ──►  drivers/  ──►  core/
                            ▲
                            └── drivers/ implements core's interfaces
```

Exact include graph, as it stands:

```
core/Types.hpp                    → <cstdint>                        LEAF
core/interfaces/IImuSensor.hpp    → core/Types.hpp
core/interfaces/IMotorDriver.hpp  → core/Types.hpp
core/StateEstimator.hpp           → core/Types.hpp
core/BalancingController.hpp      → core/Types.hpp

drivers/MoteusConfig.hpp          → <string>                         LEAF
drivers/ImuDriver.hpp             → core/Types.hpp, core/interfaces/IImuSensor.hpp
drivers/MoteusDriverWrapper.hpp   → core/Types.hpp,
                                    core/interfaces/IMotorDriver.hpp,
                                    drivers/MoteusConfig.hpp
                                    [forward-declares mjbots::moteus::Controller]

src/drivers/MoteusDriverWrapper.cpp → moteus.h        ← the ONLY TU that sees it
src/drivers/ImuDriver.cpp           → <linux/i2c-dev.h>, bmi270_config.h

testing/CsvLogger.hpp             → moteus.h
testing/MotorValidator.hpp        → moteus.h, testing/CsvLogger.hpp
```

Three invariants, and what enforces each:

**`core/` includes nothing outside `core/`.** Enforced by the build:
`cube_core` is a static library compiled from `src/core/*.cpp` only, with no
driver sources and no `moteus_lib` link. A violation is a link error.

**`moteus.h` appears in exactly one translation unit.**
`MoteusDriverWrapper.hpp` forward-declares `mjbots::moteus::Controller` and
holds it behind a `unique_ptr`, so the vendor header — which drags in the
protocol and transport headers, ~140 KB — never reaches a consumer. Changing
a driver internal does not recompile the apps.

**`drivers/` depends on `core/`, never the reverse.** Dependency inversion:
the driver implements an interface the core defines. That is what allows a
simulated IMU to substitute for the real one without the control code
noticing.

### The one asymmetry

`testing/` bypasses `IMotorDriver` and talks to `mjbots::moteus::Controller`
directly. This is deliberate — `MotorValidator` drives velocity-mode commands
and deliberately provokes a watchdog timeout, neither of which the interface
exposes and neither of which belongs in a balancing abstraction.

The cost: `testing/` cannot run against a simulated motor. Accepted, because
its entire purpose is validating *real* hardware.

---

## 2. Interface contracts

### `IMotorDriver`

```cpp
bool      initialize(std::string* error);   // false + message on failure
MotorState sendTorque(double torque_nm);    // clamps; arms a watchdog
MotorState query();                         // read without commanding
void      stop();                           // idempotent, never throws
double    maxTorqueNm() const;
```

Obligations on any implementation:

- `initialize()` **returns** false rather than throwing. A missing controller
  is a normal condition the caller decides how to treat.
- `sendTorque()` **must** clamp to `maxTorqueNm()` and **must** arm a
  watchdog, so a host crash stops the motor.
- `stop()` **must** be safe to call repeatedly, before `initialize()`, from a
  destructor, and from a signal-handler path. It must never throw — there is
  nowhere for an exception to go on those paths.
- A returned `MotorState` with `valid == false` means the controller did not
  answer. It is never a zero-filled "everything is fine".

### `IImuSensor`

```cpp
bool     initialize(std::string* error);
ImuData  read();                                    // never blocks > 1 cycle
bool     calibrateGyroBias(Seconds, std::string*);  // blocks; rejects motion
```

- `read()` must not block longer than a control period. At 200 Hz that is 5 ms.
- `valid == false` on a bus error. The estimator treats it as a safety event
  and holds its previous estimate rather than integrating a bad sample.
- `calibrateGyroBias()` returns false if the rig moved during the
  measurement — the mean would then be bias *plus* real rotation.

### Data types

`ImuData`, `MotorState` and `BodyState` are plain aggregates in SI units.
Conversion from raw counts happens in the driver; nothing above the driver
layer ever sees an LSB. `MotorState::velocity_rad_s()` is inline in the
header so `core/` can use it without linking against `cube_drivers`.

---

## 3. Why a complementary filter

For a single tilt axis, a complementary filter and a Kalman filter perform
nearly identically. The complementary filter has one tuning parameter with a
physical meaning (`tau`, a crossover frequency), where a Kalman filter has a
process and measurement covariance that are themselves guesses on an
uncharacterised rig.

It follows directly from what each sensor measures:

| | Accelerometer | Gyroscope |
|---|---|---|
| Measures | gravity + cube's own acceleration | angular rate |
| Absolute? | yes — gravity is a fixed reference | no — must be integrated |
| Drifts? | no | yes, bias accumulates without bound |
| Fails when | the cube accelerates hard | never, but the integral is unbounded |

They fail in complementary ways, hence the name: high-pass the gyro,
low-pass the accelerometer, cross over at `tau`.

Two details that are easy to get wrong and are called out in the scaffold:

- **Derive the blend coefficient from the measured `dt`**, not a constant.
  `alpha = tau/(tau+dt)` makes the filter behave identically at 200 Hz and
  50 Hz. A hardcoded coefficient silently changes the crossover frequency
  whenever the loop rate changes or a cycle runs late.
- **Gate the accelerometer.** During a hard correction the measured vector is
  mostly wheel reaction, not gravity. Blending it in drags `theta` toward
  whatever direction the cube happened to be accelerating.

---

## 4. Why three states

A cart-pole can push against the ground indefinitely. A reaction wheel
cannot: it works by *trading angular momentum* with the body, and it has a
hard speed ceiling. Once the wheel saturates the cube has no control
authority left and falls, regardless of how good the angle feedback is.

So wheel speed is not a nuisance state — it is a resource. With
`x = [theta, theta_dot, wheel_omega]`:

```
tau = -(k_theta*theta + k_theta_dot*theta_dot + k_omega*wheel_omega)
```

The third term deliberately leans the cube a fraction of a degree off
vertical so gravity does the work of accelerating the wheel back toward zero.
The cube spends a little tilt to buy back momentum headroom. It is small —
roughly a thousandth of `k_theta` — and tuned last.

Linearised about upright, with `I_total = I_b + m_w·l_w²`:

```
      ⎡ 0                      1   0 ⎤        ⎡        0          ⎤
  A = ⎢ m·g·l / I_total        0   0 ⎥    B = ⎢  −1 / I_total     ⎥
      ⎣ 0                      0   0 ⎦        ⎣ 1/I_w + 1/I_total ⎦
```

The positive `A(2,1)` entry is the open-loop instability — the cube falls.
Full solve in [`3d_scaling/README.md`](3d_scaling/README.md) §4.

---

## 5. The safety envelope

Checks run **before** any control math, in this order:

| Order | Condition | Result | Why here |
|---|---|---|---|
| 0 | config unset | `kUnconfigured`, disarmed | before anything can mask it |
| 1 | `motor.fault != 0` | `kMotorFault`, latched | the board already stopped; it is telling you why |
| 2 | `!state.valid` | `kNotReady`, **not** latched | normal during warmup; latching would prevent startup |
| 3 | `|theta| > max_tilt` | `kTiltExceeded`, latched | past recovery — driving on adds energy to the crash |
| 4 | `|omega| > max_wheel` | `kWheelSaturated`, latched | no authority left |
| 5 | `!motor.valid` | `kSensorFault`, latched | last: a dropped reply mid-fall is a symptom, not the cause |

**Trips latch.** First trip wins, so a tilt e-stop followed by the motor
faulting on the way down still reports the tilt. Recovery requires an
explicit `reset()` — a deliberate human act.

**`kNotReady` is not a fault.** It is the startup state. Distinguishing it
from a real trip is why `SafetyState` is an enum rather than a bool.

Three guards inside the law itself:

- **Deadband on `theta` only.** Applying it to the rate terms would leave the
  cube undamped exactly where damping matters most — near vertical, where it
  spends all its time.
- **Clamp**, with `torque_clamped` reported so persistent saturation is
  visible during tuning.
- **`isfinite` guard.** A NaN in moteus's `feedforward_torque` is not a no-op;
  it is an invalid command, and the board's response is not something to
  discover on a live rig.

### Unset configuration is a safety feature

`NaN` propagates silently: every comparison against it is false, so
`fabs(theta) > max_tilt_rad` would never trip and the e-stop would simply not
exist. That is why `isConfigured()` runs first, and why `cube_balancer`
refuses to start rather than warning.

---

## 6. Real-time behaviour

The loop sleeps to an **absolute deadline**, not for a fixed interval:

```cpp
next_deadline += period;
double slack = next_deadline - now();
if (slack > 0) usleep(slack * 1e6);
else { late_cycles++; if (-slack > period) next_deadline = now() + period; }
```

A fixed `usleep(5000)` would add the loop's own execution time to every
cycle, so the real rate drifts below nominal and the effective control
bandwidth changes. Absolute deadlines keep the *average* rate correct.

The resync on gross overrun matters: without it, a single long stall would be
followed by a burst of back-to-back cycles trying to catch up — each with a
tiny `dt` that the estimator would have to integrate.

`cube_balancer` reports late-cycle count and worst overrun on exit, and
suggests `chrt -f 50` if more than 5% of cycles miss. Linux `usleep` on a
non-realtime kernel typically holds a few hundred microseconds of jitter,
which is fine against a 5 ms budget.

---

## 7. Extension points

**A simulated IMU.** Implement `IImuSensor` with synthetic pendulum dynamics.
`cube_balancer` needs no change beyond choosing which to construct — this is
the cheapest way to test a control law without risking the rig.

**A second motor.** `IMotorDriver` is per-controller; construct one per axis.
But **do not** call `sendTorque()` in a loop over three drivers: that is
three serial CAN round-trips per cycle. Build frames with `MakePosition()`
and issue them together through `Transport::Cycle()`. See
`moteus/lib/cpp/examples/multiple_cycle.cc`.

**Telemetry logging.** `CsvLogger` currently takes a
`mjbots::moteus::Query::Result`, so it is tied to the moteus type. To log
from the balancer, either add an overload taking `MotorState` + `BodyState`,
or write a parallel logger in `core/` — the latter keeps it hardware-free and
usable from a simulation.

**3D corner balancing.** `theta` becomes a quaternion (Euler angles
gimbal-lock on a corner), the state grows to 9, and the axes couple
gyroscopically. The interfaces do not change shape — that is the payoff of
this structure. Full treatment in
[`3d_scaling/README.md`](3d_scaling/README.md) §5.
