# CLAUDE.md

Guidance for Claude Code working in this repository.

## What this project is

A C++ host-side driver for a single **mjbots moteus** brushless servo
controller, driven over CAN-FD from a Linux (WSL2) machine. It is currently a
bring-up harness: it pushes a controller configuration at startup and runs a
100 Hz command/telemetry loop. It is the foundation for a more sophisticated
control layer, which does not exist yet.

**The moteus controller does the real motor control.** The firmware runs FOC
commutation and a position/velocity/torque cascade at ~30 kHz on the board
itself. This host program only sends setpoints over CAN and reads back state.
Any control software built here is an *outer* loop on top of the board's inner
loops — it must not try to replace them.

## Repository layout

```
README.md               teaching guide -- architecture, data flow, per-file reference
CLAUDE.md               this file
CMakeLists.txt          build + every controller setting as a cache variable
apps/                   entry points, one per executable
  cube_balancer.cpp         200 Hz balancing loop
  direct_motor_test.cpp     motor health checks (menu or --run-all)
  imu_test.cpp              IMU validation + gyro bias calibration
  legacy_moteus_driver.cpp  pre-HAL constant-velocity driver (was main.cpp)
include/core/           PLATFORM-AGNOSTIC control code -- no hardware headers
  Types.hpp                 ImuData, MotorState, BodyState
  StateEstimator.hpp        SCAFFOLD: complementary filter
  BalancingController.hpp   SCAFFOLD: LQR + safety envelope
  TelemetryFrame.hpp        118-byte binary wire format + CRC-16
  TelemetryRing.hpp         lock-free SPSC ring, O(1) push
  TelemetrySink.hpp         sink interface + drain policy (USB -> Xiao seam)
  interfaces/               IImuSensor, IMotorDriver
tests/test_telemetry_ring.cpp  frame/ring/drain tests, no Boost, no hardware
tools/telemetry/        capture.py, decode.py, scale.py, live.py, analyze.ipynb
include/drivers/        HARDWARE implementations of those interfaces
  MoteusConfig.hpp          config struct mirroring the CMake cache vars
  MoteusDriverWrapper.hpp   moteus over CAN-FD
  ImuDriver.hpp             BMI270 over Linux I2C
include/testing/        MotorValidator, CsvLogger
src/{core,drivers,testing}/ implementations mirroring include/
config/current_config.cfg.in  board config template -> build/current_config.cfg
data/calibration/       moteus-cal-*.log + README explaining the fields
docs/
  SETUP.md                  hardware bring-up (wiring -> calibration -> run)
  ARCHITECTURE.md           dependency graph, contracts, extension points
  3d_scaling/README.md      measurement, LQR gains, BMI270 blob, 3D extension
third_party/            bmi270_config.h (Bosch, Apache-2.0) + provenance README
logs/                   gitignored; CSV output from direct_motor_test
build/, moteus/, moteus-venv/   gitignored
```

**The layering rule: nothing in `core/` may include from `drivers/`.**
Enforced by the build -- `cube_core` is its own static library with no driver
sources, so a violation is a link error, not a review comment. `moteus.h`
reaches exactly one translation unit (`src/drivers/MoteusDriverWrapper.cpp`);
the header forward-declares `mjbots::moteus::Controller`.

## Current state: scaffolds vs. implemented

| Module | State |
|---|---|
| `drivers/MoteusDriverWrapper` | implemented, verified on hardware |
| `drivers/ImuDriver` | implemented, register map from the datasheet |
| `testing/MotorValidator`, `CsvLogger` | implemented |
| `apps/legacy_moteus_driver` | implemented |
| `core/StateEstimator` | **SCAFFOLD** -- `update()`, `accelAngle()` are TODO |
| `core/BalancingController` | **SCAFFOLD** -- `update()` is TODO |

The scaffolded modules carry their full derivation as numbered `TODO(you)`
comment blocks, and every `Config` field has a what-it-does / how-to-choose /
TODO comment. **Do not implement these unless asked** -- the user is writing
them deliberately, module by module, to retain control over the control law.

Tunables are `NAN`/`-1` sentinels, not defaults. `isConfigured()` and
`firstUnsetField()` gate them, and `cube_balancer` refuses to start naming the
first unset field. This is a safety property: NaN comparisons are all false,
so an unset `max_tilt_rad` would silently disable the e-stop rather than
erroring.

## Build targets

| Target | Source | What it does |
|---|---|---|
| `cube_core` | `src/core/` | static lib, no hardware deps |
| `cube_drivers` | `src/drivers/` | static lib, links `cube_core` + `moteus_lib` |
| `direct_motor_test` | `apps/` + `testing/` | motor health checks, CSV log |
| `imu_test` | `apps/imu_test.cpp` | BMI270 validation + bias calibration |
| `cube_balancer` | `apps/cube_balancer.cpp` | 200 Hz balancing loop |
| `moteus_driver` | `apps/legacy_moteus_driver.cpp` | constant-velocity driver |
| `moteus_test` | upstream | Boost.Test suite for the moteus library |
| `test_telemetry_ring` | `tests/` | frame/ring/drain tests; also `--emit` fixtures |

Teensy firmware envs relevant here: `pio run -e telemetry_test` builds the
binary-telemetry sketch. It **emits binary, not text** — opening it in a
serial monitor shows garbage and can block the capture.

```bash
cmake -S . -B build          # from project root, NOT from inside build/
cmake --build build
ctest --test-dir build --output-on-failure

./build/direct_motor_test --run-all      # exit code is the verdict
./build/imu_test                         # then live raw-sensor stream
./build/cube_balancer --dry-run          # refuses while gains are unset
./build/moteus_driver --velocity 2.0
```

## Hardware configuration (this specific rig)

| Item | Value | Set by |
|---|---|---|
| Controller CAN ID | 1 | `MOTEUS_CONTROLLER_ID` |
| Aux port | **aux2 only — this board has no aux1** | — |
| Encoder | MA600, off-axis, SPI, 65536 CPR | `AUX2_SPI_MODE=5`, `SOURCE0_CPR` |
| SPI rate | 6 MHz (not the 12 MHz default) | `AUX2_SPI_RATE_HZ` |
| Commutation + output source | source 0 (the MA600) | `COMMUTATION_SOURCE`, `OUTPUT_SOURCE` |
| Motor poles | 24 (from calibration) | firmware |
| Gearing | 1.0, direct drive | `ROTOR_TO_OUTPUT_RATIO` |

Any `conf set aux1...` found in generic moteus tutorials **will be rejected on
this board**. Use `aux2.*` throughout.

## The configuration pipeline

This is the project's one real piece of architecture, and it is unusual enough
to be worth internalising:

1. Every controller setting is a **CMake cache variable** in `CMakeLists.txt`
   (e.g. `SERVO_MAX_CURRENT_A`, `AUX2_SPI_RATE_HZ`).
2. `configure_file` substitutes them into `current_config.cfg.in`, producing
   `build/current_config.cfg` — plain `<name> <value>` lines.
3. The generated path is baked into the binary as `DEFAULT_CONFIG_PATH`.
4. At startup, `ApplyConfig()` reads that file and sends each line to the board
   as `conf set <name> <value>` over the diagnostic channel.

So **changing a motor parameter means re-running cmake**, not editing a file:

```bash
cmake -S . -B build -DSERVO_MAX_CURRENT_A=8 -DAUX2_SPI_RATE_HZ=4000000
cmake --build build
```

`build/current_config.cfg` is generated — never edit it. Edit
`current_config.cfg.in` (to add a *key*) or the cache variable (to change a
*value*). The generated file is also directly usable by the Python tool:
`moteus_tool -t 1 --write-config build/current_config.cfg`.

Two build options govern startup behaviour:

- `-DAPPLY_CONFIG_ON_STARTUP=OFF` — skip the config push (read-only run)
- `-DPERSIST_CONFIG_TO_FLASH=ON` — follow the push with `conf write`

Without `PERSIST_CONFIG_TO_FLASH`, **every setting lives in volatile memory and
is lost on power cycle.** The driver re-pushing on each run masks this until
something else talks to the board first.

`current_config.cfg.in` sets **every** aux2 peripheral explicitly, including the
disabled ones. This is deliberate, not verbosity: a peripheral left at a stale
value claims pins another needs and fails port validation. A stale
`aux2.uart.mode 5` already caused `error.uart_pin_error` with `spi.active
False` on this board. `CMakeLists.txt` has a configure-time `FATAL_ERROR` guard
for the UART/SPI combination, and another for `ROTOR_TO_OUTPUT_RATIO > 1.0`.

## The legacy driver (apps/legacy_moteus_driver.cpp)

Formerly `main.cpp`. Kept building because the HAL was extracted from it and
it is still the quickest way to spin the motor at constant speed. New work
belongs in the other `apps/` targets.

It contains, in order:

- `ConfigError` / `ControllerRejected` -- deliberately distinct so the catch
  blocks print the right diagnosis. A rejected setting means the board *is*
  reachable; do not print the transport hint for it.
- `ReadConfig()` -- parses the cfg, strips `#` comments, rejects bare names.
- `ApplyConfig()` -- pushes each setting, **retries once after 50 ms on
  `ERR`** (some settings make the firmware revalidate `motor_position`, and
  the previous reply can still be arriving and get misattributed as "unknown
  command"), then sleeps 2 ms between settings.
- SIGINT/SIGTERM handlers calling `SetStop()`.
- A 100 Hz **velocity-mode** loop: `position = NaN` with `command.velocity`
  set, which is how moteus does constant-speed running (there is no separate
  velocity mode). Sets `watchdog_timeout`, breaks on a non-zero fault.

All of that was carried into `MoteusDriverWrapper` verbatim -- the parser,
the retry timing, the exception types, the stop-on-exit discipline. **Do not
"clean up" the sleeps in `applyConfig`**; they are load-bearing against the
real board and the comments record the exact failure each one fixes.

`MoteusDriverWrapper::sendTorque()` uses a *different* command shape:
`position = NaN` with `kp_scale = kd_scale = 0` and `feedforward_torque` set.
That disables the board's own position/velocity regulation, which is required
-- the balancing law is the regulator, and a second one underneath would
fight it.

## Calibration state

`moteus-cal-AD4AN1QwUBYgOTNO-*.log` are dumps from `moteus_tool --calibrate`.
Newest is 2026-08-03. Key values from it:

```
poles                     24
winding_resistance        0.102 Ω
inductance_d / _q         4.58e-5 / 5.58e-5 H
kv                        373.6
fit_metric                19.95
current_quality_factor    115.1
serial / uuid             AD4AN1QwUBYgOTNO / b9fd22c5-...
```

Three calibrations exist (2026-08-02 ×2, 2026-08-03), suggesting bring-up
iteration. Calibration requires `servopos.position_min/max = nan` — the sweep is
issued as a *position mode* command and travel limits block it, producing a
`ZeroDivisionError` on `poles == 0`. See SETUP.md Phase 1.5's warning box.

## ⚠️ Config limits vs. a reaction wheel

The build cache currently holds the `CMakeLists.txt` defaults:

| Variable | Current | Consequence for balancing |
|---|---|---|
| `SERVOPOS_POSITION_MIN/MAX` | `-1.0` / `1.0` | **blocks the wheel** past one revolution |
| `SERVO_MAX_CURRENT_A` | `15.0` | ~0.38 Nm at `K_t = 0.0256` |
| `SERVO_MAX_VELOCITY` | `100.0` | fine |
| `MAX_TORQUE_NM` | `0.5` | above what 15 A actually delivers |

**A reaction wheel must spin freely through many revolutions**, so the ±1.0
travel limits are wrong for `cube_balancer` and must be `nan`:

```bash
cmake -S . -B build -DSERVOPOS_POSITION_MIN=nan -DSERVOPOS_POSITION_MAX=nan
cmake --build build
```

This is safe here specifically because the balancer commands pure torque
(`position = NaN`, `kp_scale = kd_scale = 0`) — `servopos` bounds only
constrain position-mode travel, and there is no position target to bound. The
safety envelope that matters is in `BalancingController` (tilt e-stop, torque
clamp, wheel-speed limit) plus the board's `servo.max_current_A`.

Note also that `MAX_TORQUE_NM = 0.5` exceeds the real ceiling implied by
`servo.max_current_A` and the motor's `K_t` (≈ 0.38 Nm at 15 A, ≈ 0.20 Nm at
8 A). The control law will saturate before reaching its nominal limit. Raise
the current limit only within **the lower of** the motor's and the board's
rating — exceeding the factory value can damage hardware.

## The moteus library (`moteus/`)

Upstream clone of `github.com/mjbots/moteus`, at `rust/v0.5.3-14-ge158ce7e`.
Contains the complete moteus project — firmware, hardware, and client
libraries. This project uses **only** the C++ client library:

```
moteus/lib/cpp/mjbots/moteus/
  moteus.h            Controller class, the main API  (47 KB)
  moteus_protocol.h   command/query structs, register encoding  (46 KB)
  moteus_transport.h  Fdcanusb, Socketcan, TransportFactory  (47 KB)
  moteus_multiplex.h  register multiplex wire format
  moteus_optional.h   Optional<T> shim
  moteus_tokenizer.h
  test/               Boost.Test suite, wired into our ctest
```

It is **header-only**. `CMakeLists.txt` exposes it as an INTERFACE target
`moteus_lib` that only adds the include directory, so sources just
`#include "moteus.h"`. Note the tests need the deeper `lib/cpp` root on the
path instead, because they include via `"mjbots/moteus/moteus.h"`.

Also present and useful for reference, but not built here:

```
moteus/lib/python/moteus/     Python client (async), what moteus_tool uses
moteus/lib/rust/              Rust client
moteus/fw/                    controller firmware (bldc_servo.cc, foc.h,
                              motor_position.h, moteus_controller.h)
moteus/docs/reference/        configuration.md, encoders.md, limits.md,
                              pinouts.md, theory.md  <- authoritative
moteus/utils/                 moteus_tool.py, tview.py, calibration scripts
moteus/lib/cpp/examples/      simple.cc, multiple_cycle.cc, wait_complete.cc,
                              nlateral_teleop.cc, bandwidth_test.cc
moteus/CLAUDE.md              upstream's own guidance (Bazel build, firmware)
```

Upstream builds with **Bazel**, not CMake. Do not try to build `moteus/`
itself for this project — only its headers are needed.

## C++ API surface for building a control layer

`moteus::Controller` (in `moteus.h`) is the whole interface. Constructed with
an `Options` struct: `id`, `source`, `bus`, `can_prefix`, per-mode `Format`
resolutions, `default_query`, and an optional `shared_ptr<Transport>` (left
unset, a shared auto-detecting singleton is built).

Blocking command methods, each returning `Optional<Result>`:

```
SetQuery                       read state without commanding
SetStop / SetBrake             stop; clears faults
SetZeroVelocity
SetPosition                    the primary control entry point
SetPositionWaitComplete
SetVFOC                        voltage-mode FOC
SetCurrent                     direct d/q current
SetStayWithin                  bounded-region control
SetOutputNearest / SetOutputExact    redefine current position
SetRequireReindex
SetRecapturePositionVelocity
SetClockTrim
SetWriteGpio / SeGpioRead / SetAuxPwmWrite
DiagnosticCommand              the `conf set ...` channel ApplyConfig uses
```

Each has an `AsyncStart*` counterpart and a `MakePosition`-style frame builder.

`PositionMode::Command` fields — this is the control interface:

```cpp
double position;                // revolutions (NaN = hold current)
double velocity;                // rev/s
double feedforward_torque;      // Nm
double kp_scale, kd_scale;      // 1.0 = configured gain; 0 for pure velocity
double maximum_torque;          // Nm ceiling for this command
double stop_position;
double watchdog_timeout;        // ** set this in any real control loop **
double velocity_limit;          // per-command trajectory limits, override
double accel_limit;             //   servo.default_*_limit
double fixed_voltage_override;
double ilimit_scale;
double fixed_current_override;
double ignore_position_bounds;
```

`Query::Result` — the telemetry available every cycle:

```cpp
Mode mode; int8_t fault;
double position, velocity, torque;
double q_current, d_current;
double abs_position, power;
double motor_temperature, temperature, voltage;
bool trajectory_complete; HomeState home_state;
int8_t aux1_gpio, aux2_gpio;
ItemValue extra[kMaxExtra];     // arbitrary extra registers
```

**For a multi-motor or latency-sensitive control loop, do not call
`SetPosition` per controller in a loop.** Build frames with `MakePosition()`
and issue them together through `transport()->Cycle()` /`BlockingCycle()` —
this is a single bus transaction instead of N round-trips. See
`moteus/lib/cpp/examples/multiple_cycle.cc` for the exact pattern (build a
`map<int, shared_ptr<Controller>>`, `SetStop()` all to clear faults, then cycle
a `vector<CanFdFrame>`).

Transports in `moteus_transport.h`: `Fdcanusb` and `Socketcan`, both deriving
from `TimeoutTransport`, selected at runtime by `TransportRegistry` /
`Controller::DefaultArgProcess`. `ThreadedEventLoop` backs the async path.

## Python tooling (`moteus-venv/`)

A venv with `moteus`, `moteus_gui`, PySide6, matplotlib, python-can, pyserial,
numpy/scipy, ipython. Used for bring-up, not runtime:

```bash
TOOL=./moteus-venv/bin/moteus_tool
$TOOL -t 1 --info
$TOOL -t 1 --dump-config | grep aux2
$TOOL -t 1 --calibrate --cal-motor-speed 1.0
$TOOL -t 1 --console                       # raw `d pos`, `conf set` REPL
./moteus-venv/bin/tview                    # live plotting GUI
```

`--cal-speed` is deprecated; `--cal-motor-speed` is current and is mechanical
Hz, not electrical rps.

In tview, the channels that matter for bring-up are `servo_stats.position`,
`.velocity`, `.filt_bus_V`, and `aux2.spi.active` (must read `True` — `False`
means a pin conflict, not a calibration problem).

## WSL2 specifics

GUI apps (tview) work through WSLg with no X server setup. USB does **not**
pass through automatically — attach the fdcanusb from Windows PowerShell as
Administrator:

```powershell
usbipd list
usbipd attach --wsl --busid <BUSID>
```

It then appears as `/dev/ttyACM0`. Permission errors: `sudo usermod -aG dialout
$USER`, then log out and back in.

SocketCAN bring-up (CAN-FD, 1 Mbps arbitration / 5 Mbps data):

```bash
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set up can0
```

## Safety rules — apply to any code that moves the motor

1. **`servo.max_current_A` can destroy hardware.** Use the *lower* of the
   motor's rated phase current and the controller's rating. A motor rated
   higher than the board does not raise the board's ceiling.
2. **Always `SetStop()` before exiting**, including on SIGINT. Without it the
   board holds position and keeps drawing current indefinitely.
3. **Set `watchdog_timeout`** on commands in a real control loop, so the board
   stops itself if the host dies.
4. With `servo.default_velocity_limit` / `default_accel_limit` unset, velocity
   changes instantaneously and can "grow to arbitrary magnitude" — the first
   position command slams to target. They are set here (2.0 / 10.0) for that
   reason.
5. `servo.max_velocity` is a power-derating backstop, **not** a trajectory
   limit. The trajectory limits are `default_velocity_limit` / `_accel_limit`.
6. Set `rotor_to_output_ratio` **before** tuning `servo.pid_position` — PID
   gains are in physical units at the output, i.e. after that scaling, so
   changing the ratio later silently changes what every gain means.
7. `id.id` is deliberately excluded from the pushed config: the driver
   addresses the board by CAN ID, so pushing a new one mid-session would
   disconnect it.

## Not yet done

- **Implement `StateEstimator::update()` and `accelAngle()`.** Scaffolded with
  an 8-step derivation in the .cpp. The user is writing these.
- **Implement `BalancingController::update()`.** Scaffolded with the safety
  ordering and the control law.
- **Measure the physical parameters** -- body/wheel inertia, mass, CoM
  distance. Procedure in `docs/3d_scaling/README.md` section 3. Everything
  downstream is blocked on this; nothing else is.
- **Compute the LQR gains** from those measurements, section 4.
- **Torque ceiling mismatch.** kv = 373.6 gives `K_t` = 0.0256 Nm/A, so
  `servo.max_current_A = 15` is a real ceiling of ~0.38 Nm. Reconcile against
  whatever `max_torque_nm` ends up being.
- **`servopos` limits must be `nan`** for a reaction wheel -- the default
  +/-1.0 rev would block continuous spinning. Safe because the balancer
  commands pure torque with no position target.
- **Balancer telemetry logging.** `direct_motor_test` writes CSV via
  `CsvLogger`; the balancer only prints. `CsvLogger` takes a
  `moteus::Query::Result`, so logging `BodyState` needs an overload or a
  parallel logger in `core/`.

## Conventions

- C++17, `-Wall -Wextra`, Google C++ style (matching upstream).
- Comments in the drivers and `CMakeLists.txt` explain *why*, often citing a
  specific failure that was actually hit on this hardware. Preserve that style
  — those comments are the project's real bug history.
- No trailing whitespace; blank lines fully empty.
- README.md and SETUP.md are hand-maintained. If driver behaviour changes,
  update them — they are already stale about the position command.
