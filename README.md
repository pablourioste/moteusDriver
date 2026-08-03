# moteusDriver — 1D reaction wheel balancing cube

Host-side C++ control software for a reaction-wheel inverted pendulum: a cube
that balances on one edge by accelerating a flywheel. Built on an
[mjbots moteus](https://github.com/mjbots/moteus) brushless controller over
CAN-FD and a Bosch BMI270 IMU over I2C.

**The moteus board does the real motor control.** Its firmware runs FOC
commutation and a position/velocity/torque cascade at ~30 kHz *on the
controller itself*. Everything in this repository is the **outer loop**: read
tilt, decide a torque, send it. Do not try to reimplement the inner loops
here — you cannot beat 30 kHz over a 200 Hz CAN link.

---

## Status

The hardware layer is written and verified against the real rig. The control
layer is a **documented scaffold** — the structure and the derivations are
there, the bodies are yours to write.

| Module | State |
|---|---|
| `drivers/MoteusDriverWrapper` | ✅ Implemented, verified on hardware |
| `drivers/ImuDriver` (BMI270) | ✅ Implemented, register map from datasheet |
| `testing/MotorValidator` | ✅ Implemented, six health checks |
| `testing/CsvLogger` | ✅ Implemented |
| `apps/legacy_moteus_driver` | ✅ Implemented, constant-velocity driver |
| `core/StateEstimator` | 🔨 **Scaffold** — `update()` is yours |
| `core/BalancingController` | 🔨 **Scaffold** — `update()` is yours |

Why scaffolds and not defaults: the filter time constant, the three LQR
gains, the axis mapping and the safety thresholds **cannot be guessed**. They
depend on your cube's inertia, mass and centre of mass. Shipping plausible
numbers would make invented values indistinguishable from measured ones — so
they ship as `NaN` sentinels and the code refuses to run until you replace
them.

```
$ ./build/cube_balancer --dry-run
Refusing to run: the control configuration is incomplete.

  StateEstimator::Config::tau is unset.
  BalancingController::Config::k_theta is unset.
```

This matters more than it looks. **NaN does not fail loudly on its own** —
every comparison against NaN is false, so `fabs(theta) > max_tilt_rad` would
silently never trip and the e-stop would simply be gone. Stopping at startup
with a field name is far better than running with a disabled safety envelope.

---

## Layout

```
apps/          entry points — one file per executable
include/core/  control logic, NO hardware dependencies
include/drivers/ hardware implementations of the core's interfaces
include/testing/ hardware validation harness
src/           implementations, mirroring include/
config/        board configuration template
data/          versioned measurements (calibration logs)
docs/          setup guide, architecture, 3D scaling blueprint
third_party/   Bosch BMI270 config blob (Apache-2.0)
logs/          CSV test output (gitignored)
```

### What each file does

**Applications** — `apps/`

| File | Purpose |
|---|---|
| `cube_balancer.cpp` | The 200 Hz balancing loop. Combines estimator + controller + both drivers. |
| `direct_motor_test.cpp` | Motor health checks: voltage, direction, torque ramp, thermal, watchdog, velocity hold. Interactive menu or `--run-all`. |
| `imu_test.cpp` | IMU validation and gyro-bias calibration. No motor involved. |
| `legacy_moteus_driver.cpp` | Pre-HAL single-file driver. Spins the motor at constant speed; the HAL was extracted from it. |

**Core** — `include/core/`, `src/core/` — compiles with no hardware present

| File | Purpose |
|---|---|
| `Types.hpp` | `ImuData`, `MotorState`, `BodyState`. Plain data, SI units. |
| `interfaces/IImuSensor.hpp` | Abstract IMU. Lets a simulated sensor replace the real one. |
| `interfaces/IMotorDriver.hpp` | Abstract motor. Same idea. |
| `StateEstimator.{hpp,cpp}` | 🔨 Complementary filter: accel + gyro → `theta`, `theta_dot`. |
| `BalancingController.{hpp,cpp}` | 🔨 LQR state feedback + the safety envelope (e-stop, clamp, latching). |

**Drivers** — `include/drivers/`, `src/drivers/`

| File | Purpose |
|---|---|
| `MoteusConfig.hpp` | Config struct mirroring the CMake cache variables. |
| `MoteusDriverWrapper.{hpp,cpp}` | moteus over CAN-FD. Config push, pure-torque commands, stop. |
| `ImuDriver.{hpp,cpp}` | BMI270 over Linux I2C. Register access, config-blob upload, scaling. |

**Testing** — `include/testing/`, `src/testing/`

| File | Purpose |
|---|---|
| `MotorValidator.{hpp,cpp}` | The six health checks, with pass/fail thresholds in one struct. |
| `CsvLogger.{hpp,cpp}` | Appends telemetry rows to `logs/motor_test_<timestamp>.csv`. |

---

## How it fits together

Three layers, strictly one-directional:

```
        apps/          cube_balancer, imu_test, direct_motor_test
          |                        (entry points, argument parsing)
          v
    include/drivers/   MoteusDriverWrapper  |  ImuDriver
          |            (CAN-FD, I2C, vendor headers live HERE and nowhere else)
          v
     include/core/     StateEstimator, BalancingController
                       (pure math + data; no hardware, no vendor headers)
```

**The invariant: nothing in `core/` may include anything from `drivers/`.**

That is not a style preference. It is enforced by the build — `cube_core` is
its own static library compiled with no driver sources, so a violation shows
up as a link error rather than a review comment.

Two things it buys you:

1. **The control law is testable on a desktop.** No CAN bus, no IMU, no cube.
   You can feed `StateEstimator` a synthetic tilt sweep and assert on the
   output.
2. **Hardware is swappable.** `IImuSensor` and `IMotorDriver` are the only
   contracts. A simulated IMU, or a different motor controller, drops in
   without the control code noticing.

The dependency inversion is what makes this work: `drivers/` depends on
`core/` (it implements core's interfaces), never the reverse.

---

## One control cycle, traced

The balancer runs at 200 Hz — 5 ms per cycle. Here is exactly what happens,
naming each file and function:

```
  1. READ IMU          ImuDriver::read()                 src/drivers/ImuDriver.cpp
                       I2C burst read, 12 bytes from register 0x0C
                       (accel XYZ then gyro XYZ, one transaction so the
                        pair is coherent), scaled to m/s^2 and rad/s,
                        gyro bias subtracted
                         -> ImuData

  2. QUERY MOTOR       MoteusDriverWrapper::query()      src/drivers/MoteusDriverWrapper.cpp
                       CAN-FD round trip to controller ID 1
                         -> MotorState  (wheel position, velocity, torque,
                                         bus voltage, fault code)

  3. ESTIMATE STATE    StateEstimator::update()          src/core/StateEstimator.cpp
                       accel -> absolute tilt (atan2, drifts never, noisy)
                       gyro  -> tilt rate    (clean, but integrates to drift)
                       complementary blend at time constant tau
                       wheel speed copied straight from MotorState
                         -> BodyState  {theta, theta_dot, wheel_omega}

  4. DECIDE TORQUE     BalancingController::update()     src/core/BalancingController.cpp
                       safety envelope FIRST (fault, tilt, saturation)
                       then  tau = -(k_th*theta + k_thd*theta_dot + k_w*omega)
                       clamp, NaN guard
                         -> ControlOutput {torque_nm, armed, safety}

  5. COMMAND           MoteusDriverWrapper::sendTorque() src/drivers/MoteusDriverWrapper.cpp
                       moteus::PositionMode::Command with
                         position = NaN, kp_scale = 0, kd_scale = 0
                         feedforward_torque = torque_nm
                         watchdog_timeout = 20 * period
                         -> CAN-FD frame

  6. SLEEP             to an ABSOLUTE deadline, not a fixed interval
                       (a fixed sleep accumulates every overrun into
                        permanent drift in the real control rate)
```

### How the two sensors are coordinated

There is no separate sensor thread and no interrupt handling — both devices
are polled synchronously at the top of each cycle. At 200 Hz with a 400 Hz
IMU ODR there is always a fresh sample waiting, and a single-threaded loop
removes a whole class of race conditions.

**Wheel speed is not a third sensor.** It comes from the motor query in
step 2, derived on the board from the MA600 encoder and already PLL-filtered
there (`motor_position.sources.0.pll_filter_hz = 400`). Filtering it again on
the host would only add phase lag to the term that exists to stop the wheel
saturating.

**The motor is queried before the torque is computed**, not after. The wheel
speed feeding `k_omega` is then from this cycle, not the previous one — one
cycle of staleness in a momentum-management term is a real phase error at
200 Hz.

### Why `position = NaN, kp_scale = 0, kd_scale = 0`

This is the moteus idiom for pure torque. It switches **off** the board's own
position and velocity regulation, leaving `feedforward_torque` as the only
term. That is deliberate: the balancing law *is* the regulator, and a second
regulator underneath it would fight the outer loop.

Note the legacy driver does the opposite — it leaves the gain scales at 1.0
so the board's PID holds speed against load. Two different command shapes for
two different jobs.

---

## The configuration pipeline

Unusual enough to be worth reading before you change a motor parameter.

```
  CMakeLists.txt cache variable      e.g. SERVO_MAX_CURRENT_A = 15.0
            |
            |  configure_file() substitutes @VARS@
            v
  config/current_config.cfg.in       template, version controlled
            |
            v
  build/current_config.cfg           generated — NEVER edit this
            |
            |  path baked into the binary as DEFAULT_CONFIG_PATH
            v
  MoteusDriverWrapper::applyConfig() sends each line as `conf set <name> <value>`
            |                        over the moteus diagnostic channel
            v
  the controller's volatile memory   (or flash, with PERSIST_CONFIG_TO_FLASH)
```

So **changing a motor parameter means re-running cmake**, not editing a file:

```bash
cmake -S . -B build -DSERVO_MAX_CURRENT_A=8 -DAUX2_SPI_RATE_HZ=4000000
cmake --build build
```

Add a *key* → edit `config/current_config.cfg.in`.
Change a *value* → change the cache variable.

Two things that bite:

- **Without `-DPERSIST_CONFIG_TO_FLASH=ON`, every setting is lost on power
  cycle.** The driver re-pushing on each run hides this until something else
  talks to the board first.
- `current_config.cfg.in` sets **every** aux2 peripheral explicitly, including
  the disabled ones. That is deliberate: a peripheral left at a stale value
  claims pins another one needs and fails port validation. A stale
  `aux2.uart.mode 5` already caused `error.uart_pin_error` with
  `spi.active False` on this board.

---

## Build and run

```bash
cmake -S . -B build          # from the project root, NOT from inside build/
cmake --build build
ctest --test-dir build --output-on-failure
```

Seven targets: `cube_core` and `cube_drivers` (libraries), four executables,
and `moteus_test` (the upstream library's Boost.Test suite).

**Run them in this order.** Each one rules out a class of fault before the
next can be trusted:

```bash
# 1. Is the motor healthy?  Applies torque — clear the mechanism first.
./build/direct_motor_test --run-all        # exit code is the verdict
./build/direct_motor_test                  # interactive menu

# 2. Is the IMU sane, and which way is positive tilt?
./build/imu_test
./build/imu_test --device /dev/i2c-3 --address 69

# 3. Does the loop run without commanding torque?
./build/cube_balancer --dry-run

# 4. Live.  Only after 1–3 pass and the gains are computed.
./build/cube_balancer --k-theta 3.2 --k-theta-dot 0.31 --k-omega 0.0021

# Constant-speed spin, unrelated to balancing:
./build/moteus_driver --velocity 2.0
```

Transport flags differ between the C++ binaries and the Python tools:

```bash
./build/cube_balancer --fdcanusb /dev/ttyACM0     # C++
./moteus-venv/bin/moteus_tool --fdcanusb /dev/ttyACM0   # Python
./moteus-venv/bin/tview --can-iface socketcan --can-chan can0
```

---

## Implementing the scaffolds

Both scaffolded files carry the full derivation in a numbered `TODO(you)`
block — what to compute, in what order, and what specifically breaks if you
skip a step. Start there:

- [`src/core/StateEstimator.cpp`](src/core/StateEstimator.cpp) — eight steps,
  from wheel-speed passthrough to the dt-aware complementary blend.
- [`src/core/BalancingController.cpp`](src/core/BalancingController.cpp) — the
  safety envelope ordering, then `u = -Kx`.

Every `Config` field in the corresponding headers has a **what it does / how
to choose it / TODO** comment.

The gains need four measured quantities — body inertia about the pivot edge,
wheel inertia, mass, centre-of-mass distance.
[`docs/3d_scaling/README.md`](docs/3d_scaling/README.md) §3 has the
measurement procedures (a pendulum swing test for `I_b`) and §4 has a
runnable `scipy` Riccati solve that turns them into `K`, with sanity checks
and a staged tuning procedure.

Suggested order:

1. Implement `StateEstimator`. Verify with `imu_test` that tilt tracks a hand
   disturbance and has the right sign.
2. Measure the physical parameters (§3).
3. Compute `K` (§4).
4. Implement `BalancingController`.
5. `cube_balancer --dry-run`, watching θ and τ without energising the motor.
6. Live, at ~25% of computed gains, holding the cube.

---

## Safety

1. **Verify the tilt sign before enabling the motor.** A sign error in
   `gyro_axis` / `invert_theta` makes the controller drive the cube over
   instead of catching it. This is the single most dangerous unset value.
2. **`servo.max_current_A` can destroy hardware.** Use the *lower* of the
   motor's rated phase current and the board's rating.
3. **Know your real torque ceiling.** `K_t = 60/(2π·kv)`. At kv ≈ 373.6 that
   is ~0.0256 Nm/A, so 15 A gives ≈ 0.38 Nm — *less* than `MAX_TORQUE_NM`'s
   0.5. Setting a clamp above the real ceiling means the law believes it has
   authority it does not have.
4. **A reaction wheel needs `servopos` limits at `nan`.** It must spin freely
   through many revolutions; the default ±1.0 rev limits would block it. Safe
   here because the balancer commands pure torque with no position target —
   the real envelope is the tilt e-stop, torque clamp and wheel-speed limit.
5. **Always `SetStop()` on exit.** All apps install SIGINT/SIGTERM handlers;
   without one, Ctrl-C leaves the board holding its last command and drawing
   current.
6. **Set `watchdog_timeout`.** If the host dies, the board stops itself.
7. Run on a soft surface with nothing in the cube's arc, and keep the power
   cut in reach.

---

## Known issues

- **`imu_test` cannot verify a corrected axis mapping.** It default-constructs
  `StateEstimator`, so it can only ever exercise the default configuration —
  yet checking a *corrected* mapping is precisely its purpose. It prints raw
  accel/gyro instead, which is enough to choose the mapping but not to confirm
  it end to end.
- **`kv` disagrees across calibration runs**: 368.1, 344.6, 373.6 (newest).
  `CMakeLists.txt` cites 344.6. The derived no-load ceiling is understated by
  roughly 8%. See [`data/calibration/`](data/calibration/).
- **`MoteusConfig::motor_poles` and `encoder_cpr` are not cross-checked**
  against `SOURCE0_CPR` in CMake. They agree today by coincidence, and
  nothing would catch a drift.

---

## Hardware notes

This rig, specifically:

| Item | Value |
|---|---|
| Controller | moteus, CAN ID 1 |
| Aux port | **aux2 only — this board has no aux1** |
| Encoder | MA600, off-axis, SPI at 6 MHz, 65536 CPR |
| Motor | 24 poles, kv ≈ 373.6, direct drive (ratio 1.0) |
| IMU | BMI270 over I2C, addr 0x68 |

Any `conf set aux1...` from a generic moteus tutorial **will be rejected**.

**WSL2**: GUI apps (tview) work through WSLg. USB does not pass through
automatically — attach the fdcanusb from Windows PowerShell as Administrator:

```powershell
usbipd list
usbipd attach --wsl --busid <BUSID>
```

It appears as `/dev/ttyACM0`. Permission errors: `sudo usermod -aG dialout
$USER`, then log out and back in.

**There is no I2C on WSL2**, so the IMU only works on the target SBC.
`imu_test` says so explicitly rather than failing obscurely.

SocketCAN bring-up (CAN-FD, 1 Mbps arbitration / 5 Mbps data):

```bash
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set up can0
```

---

## Documentation

| Document | Contents |
|---|---|
| [`docs/SETUP.md`](docs/SETUP.md) | Hardware bring-up: wiring, encoder config, calibration, verification. Start here with new hardware. |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | Dependency graph, interface contracts, extension points, design rationale. |
| [`docs/3d_scaling/README.md`](docs/3d_scaling/README.md) | Measuring physical parameters, computing LQR gains, the BMI270 blob, and extending to 3D corner balancing (Cubli). |
| [`data/calibration/README.md`](data/calibration/README.md) | What a moteus calibration log contains and what the numbers mean. |
| [`CLAUDE.md`](CLAUDE.md) | Context for Claude Code sessions. |

## Dependencies

- CMake ≥ 3.10, a C++17 compiler
- `moteus/` — clone of `github.com/mjbots/moteus` (header-only C++ library;
  gitignored, re-clone rather than commit)
- `moteus-venv/` — Python venv with `moteus` + `moteus-gui` for `moteus_tool`
  and `tview` (gitignored)
- Boost.Test, optional — `sudo apt install libboost-test-dev` enables
  `moteus_test`

```bash
git clone https://github.com/mjbots/moteus.git
python3 -m venv moteus-venv && ./moteus-venv/bin/pip install moteus moteus-gui
```

## Licence

`third_party/bmi270_config.h` is Bosch Sensortec's, Apache-2.0 — see
[`third_party/README.md`](third_party/README.md). The moteus library is
Apache-2.0. The rest is yours.
