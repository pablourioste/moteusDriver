# Install — development environment

Everything you need to build this repository, from a fresh machine.

| Document | Answers |
|---|---|
| **this file** | **What do I install to build the code?** |
| [2D_model/driver+encoder/SETUP.md](2D_model/driver+encoder/SETUP.md) | How do I bring up the moteus + encoder *hardware*? |
| [2D_model/IMU_SETUP.md](2D_model/IMU_SETUP.md) | How do I wire and calibrate the IMU? |
| [2D_model/INTEGRATION.md](2D_model/INTEGRATION.md) | In what order do I build the whole system? |

**Two build systems, one set of sources:**

```bash
cmake --build build          # Linux host binaries  (x86)
pio run -e teensy41          # Teensy firmware      (ARM Cortex-M7)
```

You do not need both. Install only what you are working on:

| I am working on… | Install |
|---|---|
| Control law, host tools, simulation | **Part A** only |
| Teensy firmware | **Part A** + **Part B** |
| Motor calibration, tview | **Part A** + **Part C** |

---

# Part A — Host build (required)

## A1. System packages

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3-venv
sudo apt install -y libboost-test-dev       # optional: enables `moteus_test`
```

Requires CMake ≥ 3.10 and a C++17 compiler.

## A2. The moteus client library

Header-only, and **gitignored** — clone it, do not commit it:

```bash
cd <repo root>
git clone https://github.com/mjbots/moteus.git
```

## A3. Build

```bash
cmake -S . -B build          # from the repo root, NOT inside build/
cmake --build build
ctest --test-dir build --output-on-failure
```

### ✅ Check A

- [ ] `build/cube_balancer`, `build/imu_test`, `build/direct_motor_test`,
      `build/moteus_driver` all exist
- [ ] `./build/cube_balancer --dry-run` runs and refuses with an unset-field message
      (that refusal is correct — the gains are deliberately unset)

---

# Part B — Teensy firmware

## B1. ⚠ Do not use the apt package

```bash
sudo apt remove platformio      # if you already installed it
```

Ubuntu ships PlatformIO **4.3.4** (2020). It crashes on import against modern `click`:

```
AttributeError: 'PlatformioCLI' object has no attribute 'resultcallback'.
Did you mean: 'result_callback'?
```

It also predates Teensy 4.1 support. It cannot be made to work.

## B2. Install PlatformIO in its own venv

A dedicated venv keeps PlatformIO's dependencies away from `moteus-venv` and from
system Python:

```bash
python3 -m venv ~/.platformio-venv
~/.platformio-venv/bin/pip install --upgrade pip
~/.platformio-venv/bin/pip install platformio
```

Put it on `PATH` — **add this to `~/.bashrc`**, or a new shell will find the broken
system copy again:

```bash
export PATH="$HOME/.platformio-venv/bin:$PATH"
```

### ✅ Check B2

```bash
which pio          # must be ~/.platformio-venv/bin/pio
pio --version      # must be 6.x, NOT 4.3.4
```

If `which pio` still says `/usr/bin/pio`, the apt package is shadowing it. Remove it
(B1) and open a new shell.

## B3. First build

```bash
pio run -e teensy41
```

The first run downloads the ARM toolchain and Teensy framework (~500 MB) into
`~/.platformio`. Takes a few minutes. Later builds are ~25 s cold, <1 s incremental.

> **Two different directories, easy to confuse:**
> `~/.platformio-venv` (~31 MB) is the `pio` **program**.
> `~/.platformio` (~513 MB) is the **toolchain and framework** it downloads.
> `.vscode/c_cpp_properties.json` points IntelliSense at the second one, so it only
> resolves after this first build succeeds.

### ✅ Check B3

- [ ] Ends with `[SUCCESS]`
- [ ] Reports Teensy 4.1 usage — roughly `FLASH: code:37708 … free for files:8074244`

That validates compiler, framework and board config with **no hardware attached**.

## B4. Flashing — WSL2 users read this

Building works anywhere. **Flashing from WSL2 does not, reliably:**

> Teensy upload reboots the board into its HalfKay bootloader, which enumerates as a
> *different* USB device (`16C0:0478`) from the running sketch (`16C0:0483`). A
> `usbipd` attachment binds to one device, so it **drops on every upload**.

Recommended split — same `platformio.ini` both sides:

| Task | Where | Command |
|---|---|---|
| Build | WSL2 | `pio run -e teensy41` |
| **Flash** | **Windows** | `pio run -e teensy41 -t upload` |
| **Serial** | **Windows** | `pio device monitor` |

Install PlatformIO on Windows through VS Code (Extensions → "PlatformIO IDE"); it
bundles its own Python and toolchain. Windows opens the repo in place:

```
\\wsl$\Ubuntu\home\<user>\projects\moteusDriver
```

No second clone.

**Native Linux users:** everything runs in one place. Add udev rules so uploads work
without root:

```bash
curl -fsSL https://www.pjrc.com/teensy/00-teensy.rules \
  | sudo tee /etc/udev/rules.d/00-teensy.rules
sudo udevadm control --reload-rules
```

**Staying pure-WSL** is possible but adds friction on every cycle:

```bash
usbipd list                              # from Windows PowerShell (Admin)
usbipd attach --wsl --busid <BUSID>      # ...repeat after EVERY upload
```

---

# Part C — Python tooling (moteus bring-up)

For `moteus_tool` (calibration, config push) and `tview` (live plotting). Runtime is
C++; this is bring-up only.

```bash
cd <repo root>
python3 -m venv moteus-venv
./moteus-venv/bin/pip install moteus moteus-gui
```

### ✅ Check C

```bash
./moteus-venv/bin/moteus_tool --help
```

**WSL2:** the fdcanusb needs USB passthrough — from Windows PowerShell as Admin:

```powershell
usbipd list
usbipd attach --wsl --busid <BUSID>
```

It then appears as `/dev/ttyACM0`. On permission errors:
`sudo usermod -aG dialout $USER`, then log out and back in.

`tview` works through WSLg with no X server setup.

---

# VS Code

`.vscode/c_cpp_properties.json` is committed and provides **two IntelliSense
configurations**:

| Configuration | For |
|---|---|
| **Linux** | `apps/`, `src/`, `include/` — host build, uses `build/compile_commands.json` |
| **Teensy41** | `firmware/` — cross-compiled, points at `~/.platformio` |

Switch with `Ctrl+Shift+P` → *C/C++: Select a Configuration*.

If `firmware/main.cpp` reports `cannot open source file "Arduino.h"`:

1. Confirm you are on the **Teensy41** configuration, not Linux
2. Confirm Part B3 completed — the framework must exist under `~/.platformio`

Recommended extensions: **C/C++** (ms-vscode.cpptools), **PlatformIO IDE** (Windows
side, for flashing).

---

# Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `pio`: `'PlatformioCLI' object has no attribute 'resultcallback'` | apt PlatformIO 4.3.4 | B1 + B2 |
| `which pio` → `/usr/bin/pio` | apt package shadowing the venv | `sudo apt remove platformio`, new shell |
| `pio: command not found` | PATH missing | `export PATH="$HOME/.platformio-venv/bin:$PATH"` in `~/.bashrc` |
| `cannot open source file "Arduino.h"` | Wrong IntelliSense config, or B3 not yet run | See VS Code section |
| CMake: `moteus.h not found` | `moteus/` not cloned | A2 |
| Upload fails / no Teensy Loader | WSL usbipd, charge-only cable, or needs PROGRAM button | B4 |
| `cube_balancer` refuses to start | **Correct** — gains are unset sentinels | See root README |
| No `/dev/i2c-*` | There is no I2C on WSL2 | The IMU runs on the Teensy — see IMU_SETUP.md |

---

# Summary — fresh machine, full setup

```bash
# A — host build
sudo apt install -y build-essential cmake git python3-venv libboost-test-dev
git clone https://github.com/mjbots/moteus.git
cmake -S . -B build && cmake --build build

# B — Teensy firmware
sudo apt remove -y platformio                     # if present
python3 -m venv ~/.platformio-venv
~/.platformio-venv/bin/pip install platformio
echo 'export PATH="$HOME/.platformio-venv/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
pio run -e teensy41

# C — Python tooling
python3 -m venv moteus-venv
./moteus-venv/bin/pip install moteus moteus-gui
```
