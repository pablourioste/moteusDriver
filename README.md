# moteusDriver — reaction wheel balancing cube

C++ control software for a reaction-wheel inverted pendulum: a cube that balances
on one edge by accelerating a flywheel. Built on an
[mjbots moteus](https://github.com/mjbots/moteus) brushless controller over CAN-FD
and a Bosch BMI270 IMU.

**The moteus board does the real motor control.** Its firmware runs FOC commutation
and a position/velocity/torque cascade at ~30 kHz *on the controller itself*.
Everything in this repository is the **outer loop**: read tilt, decide a torque, send
it. Do not try to reimplement the inner loops here — you cannot beat 30 kHz over a
200 Hz link.

---

## Where to go

| I want to… | Read |
|---|---|
| **Set up my machine to build this** | [`docs/INSTALL.md`](docs/INSTALL.md) |
| Understand the design and the file layout | [`docs/2D_model/ARCHITECTURE.md`](docs/2D_model/ARCHITECTURE.md) |
| **Build the system step by step** | [`docs/2D_model/INTEGRATION.md`](docs/2D_model/INTEGRATION.md) |
| Bring up new hardware (wiring, encoder, calibration) | [`docs/2D_model/driver+encoder/SETUP.md`](docs/2D_model/driver+encoder/SETUP.md) |
| Measure inertia / compute LQR gains / go 3D | [`docs/3d_scaling/README.md`](docs/3d_scaling/README.md) |
| **Wire, test and calibrate the IMU** | [`docs/2D_model/IMU_SETUP.md`](docs/2D_model/IMU_SETUP.md) |
| Understand the IMU design decisions | [`docs/2D_model/IMU_BLUEPRINT.md`](docs/2D_model/IMU_BLUEPRINT.md) |
| Read a moteus calibration log | [`data/calibration/README.md`](data/calibration/README.md) |

---

## Status

The hardware layer is written and verified against the real rig. The control layer is
a **documented scaffold** — the structure and derivations are there, the bodies are
yours to write.

| Module | State |
|---|---|
| `drivers/MoteusDriverWrapper` | ✅ Implemented, verified on hardware |
| `drivers/ImuDriver` (BMI270 / I2C) | ✅ Implemented, register map from datasheet |
| `testing/MotorValidator`, `CsvLogger` | ✅ Implemented |
| `apps/legacy_moteus_driver` | ✅ Implemented, constant-velocity driver |
| `core/StateEstimator` | 🔨 **Scaffold** — `update()` is yours |
| `core/BalancingController` | 🔨 **Scaffold** — `update()` is yours |
| Teensy 4.1 port | 📋 Planned — see [INTEGRATION.md](docs/2D_model/INTEGRATION.md) |

**Why scaffolds and not defaults.** The filter time constant, the three LQR gains, the
axis mapping and the safety thresholds **cannot be guessed** — they depend on your
cube's inertia, mass and centre of mass. Shipping plausible numbers would make invented
values indistinguishable from measured ones, so they ship as `NaN` sentinels and the
code refuses to run:

```
$ ./build/cube_balancer --dry-run
Refusing to run: the control configuration is incomplete.

  StateEstimator::Config::tau is unset.
  BalancingController::Config::k_theta is unset.
```

This matters more than it looks. **NaN does not fail loudly on its own** — every
comparison against NaN is false, so `fabs(theta) > max_tilt_rad` would silently never
trip and the e-stop would simply be gone.

---

## Build and run

```bash
cmake -S . -B build          # from the project root, NOT from inside build/
cmake --build build
ctest --test-dir build --output-on-failure
```

**Run them in this order.** Each rules out a class of fault before the next can be
trusted:

```bash
# 1. Is the motor healthy?  Applies torque — clear the mechanism first.
./build/direct_motor_test --run-all        # exit code is the verdict

# 2. Is the IMU sane, and which way is positive tilt?
./build/imu_test

# 3. Does the loop run without commanding torque?
./build/cube_balancer --dry-run

# 4. Live.  Only after 1–3 pass and the gains are computed.
./build/cube_balancer --k-theta 3.2 --k-theta-dot 0.31 --k-omega 0.0021
```

Changing a motor parameter means **re-running cmake**, not editing a file — every
controller setting is a CMake cache variable pushed to the board at startup:

```bash
cmake -S . -B build -DSERVO_MAX_CURRENT_A=8
cmake --build build
```

Full explanation of that pipeline in
[`ARCHITECTURE.md`](docs/2D_model/ARCHITECTURE.md).

---

## Layout

```
apps/            entry points — one file per executable
include/core/    control logic, NO hardware dependencies
include/drivers/ hardware implementations of core's interfaces
include/testing/ hardware validation harness
src/             implementations, mirroring include/
config/          board configuration template
docs/            architecture, integration guide, 3D scaling
third_party/     Bosch BMI270 config blob (Apache-2.0)
```

**The invariant: nothing in `core/` may include anything from `drivers/`.** It is
enforced by the build — `cube_core` is a static library compiled with no driver
sources, so a violation is a link error, not a review comment. Full rationale and the
per-file reference in [`ARCHITECTURE.md`](docs/2D_model/ARCHITECTURE.md).

---

## Safety

Read this before energising anything.

1. **Verify the tilt sign before enabling the motor.** A sign error in `gyro_axis` /
   `invert_theta` makes the controller drive the cube over instead of catching it.
   This is the single most dangerous unset value.
2. **`servo.max_current_A` can destroy hardware.** Use the *lower* of the motor's
   rated phase current and the board's rating.
3. **Know your real torque ceiling.** `K_t = 60/(2π·kv)`. At kv ≈ 373.6 that is
   ~0.0256 Nm/A, so 15 A gives ≈ 0.38 Nm — *less* than `MAX_TORQUE_NM`'s 0.5.
4. **A reaction wheel needs `servopos` limits at `nan`.** It must spin freely through
   many revolutions; the default ±1.0 rev limits would block it.
5. **Always `SetStop()` on exit.** Without it, Ctrl-C leaves the board holding its
   last command and drawing current.
6. **Set `watchdog_timeout`.** If the host dies, the board stops itself.
7. Run on a soft surface with nothing in the cube's arc, and keep the power cut in
   reach.

---

## Hardware

| Item | Value |
|---|---|
| Controller | moteus, CAN ID 1 |
| Aux port | **aux2 only — this board has no aux1** |
| Encoder | MA600, off-axis, SPI at 6 MHz, 65536 CPR |
| Motor | 24 poles, kv ≈ 373.6, direct drive (ratio 1.0) |
| IMU | BMI270, addr 0x68 |

Any `conf set aux1...` from a generic moteus tutorial **will be rejected**.

**WSL2**: tview works through WSLg. USB does not pass through automatically — attach
the fdcanusb from Windows PowerShell as Administrator:

```powershell
usbipd list
usbipd attach --wsl --busid <BUSID>
```

**There is no I2C on WSL2**, so the IMU only works on the target hardware.

---

## Known issues

- **`imu_test` cannot verify a corrected axis mapping.** It default-constructs
  `StateEstimator`, so it only ever exercises the default configuration — yet checking
  a *corrected* mapping is precisely its purpose. It prints raw accel/gyro instead.
- **`kv` disagrees across calibration runs**: 368.1, 344.6, 373.6 (newest).
  `CMakeLists.txt` cites 344.6, understating the derived ceiling by ~8%.
- **`MoteusConfig::motor_poles` and `encoder_cpr` are not cross-checked** against
  `SOURCE0_CPR` in CMake. They agree today by coincidence.

---

## Dependencies

Full setup — including the Teensy toolchain and the WSL2 caveats — is in
[`docs/INSTALL.md`](docs/INSTALL.md). The short version:

- CMake ≥ 3.10, a C++17 compiler
- `moteus/` — clone of `github.com/mjbots/moteus` (header-only; gitignored)
- `moteus-venv/` — Python venv with `moteus` + `moteus-gui` for `moteus_tool` and
  `tview` (gitignored)
- PlatformIO ≥ 6 — Teensy firmware only. **Not** the apt package, which is broken;
  see [`docs/INSTALL.md`](docs/INSTALL.md) Part B
- Boost.Test, optional — `sudo apt install libboost-test-dev` enables `moteus_test`

```bash
git clone https://github.com/mjbots/moteus.git
python3 -m venv moteus-venv && ./moteus-venv/bin/pip install moteus moteus-gui
```

## Licence

`third_party/bmi270_config.h` is Bosch Sensortec's, Apache-2.0 — see
[`third_party/README.md`](third_party/README.md). The moteus library is Apache-2.0.
The rest is yours.
