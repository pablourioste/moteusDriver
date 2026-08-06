# Rig measurements — the fill-in worksheet

**This is the only file you have to edit.** Take the rig to a scale and a knife
edge, write the numbers into the fenced block at the bottom, then run:

```bash
./moteus-venv/bin/python tools/lqr/solve_gains.py
```

It parses this file, solves the LQR problem, checks the result is actually
stable, and prints the `-D CTL_K_*` build flags to paste into
`[env:cube_balancer]` in `platformio.ini`. It refuses while any value is still
`nan`, and names the first one it finds.

Every value ships as the literal `nan` for the same reason the gains do: a
plausible-looking placeholder is exactly what makes an invented number look
authoritative. There are no defaults here to fall back on.

The procedures are in [README.md §3](README.md). This file is where the results
go, plus the raw readings behind them — a lone number with no working is one
nobody can check or re-derive six months later.

---

## Provenance

Fill this in at the same time as the numbers, not afterwards.

| | |
|---|---|
| Date measured | 03/08/2026 (masses, mechanical team) |
| Measured by | Mechanical team (masses); `cubli_panel_simscape_gates.m` (everything derived) |
| Ambient temperature | not recorded |
| Rig configuration | **Stage 1 planar panel** -- frame + encoder + wheel + motor, no base, WITH frame screws. This is the 1D/2D single-panel rig `firmware/full_case/cube_balancer` and this file's schema both target -- NOT the eventual 3D corner-balancing cube (`README.md` §5, unimplemented). |
| Scale used (and resolution) | not recorded in the source doc |

**Source, and why this isn't a bench measurement.** These values come from
*Simscape Panel Model — Build Guide*: masses weighed directly, everything else
(`l_total`, `I_pivot`, `I_w`) from a CAD-volume/back-solved-density model built
in `cubli_panel_params.m` and validated against the physical rig by
`cubli_panel_simscape_gates.m`'s six gates (hang test, swing period to
<0.1%, analytic A/B match to 1.7e-7, pole match, energy conservation, momentum
conservation) -- all passed. `I_w` extracted independently from swing periods
came out 1.6% from the CAD value, which is the actual accuracy bound here, not
the gates' individual tolerances.

Filled in via `solve_gains.py`'s **direct geometry path** (`l_total` +
`I_pivot`, added specifically to take numbers of this provenance without
inventing a fake `l`/`l_w` split -- see the tool's own comments), not the
default swing-test path (`l`, `l_w`, `T_swing`), because the Build Guide gives
the ASSEMBLY's combined CoM distance (`ℓ = 101.7 mm`) directly and does not
separately publish the body-only pivot-to-CoM distance or the pivot-to-wheel-
axis distance that the swing-test path's `l`/`l_w` fields need.

**Rig configuration** matters more than it looks. Record whether the wheel was
mounted, whether the battery/tether was attached, and where the Teensy and the
moteus were sitting. Every one of those is mass at a radius, and the whole
point of `l` is where the mass is. Re-measure if the layout changes.

---

## 1. Masses

Weigh the body **without** the wheel, then the wheel on its own. Three readings
each, lifting the item off the scale between them — a scale that has settled
reads repeatably even when it is wrong.

### `m_b` — body mass, wheel excluded

| Reading | kg |
|---|---|
| 1 | `______` |
| 2 | `______` |
| 3 | `______` |
| **mean → `m_b`** | `______` |

### `m_w` — wheel mass

| Reading | kg |
|---|---|
| 1 | `______` |
| 2 | `______` |
| 3 | `______` |
| **mean → `m_w`** | `______` |

---

## 2. Lengths

Both measured **from the pivot edge** — the edge the cube balances on, not the
centre of the cube and not the corner.

### `l` — pivot to body centre of mass

Balance the body (wheel off) across a knife edge, straightedge or drill bit,
parallel to the pivot edge. Where it balances is the CoM line; measure from the
pivot edge to there.

| Trial | m |
|---|---|
| 1 | `______` |
| 2 | `______` |
| **mean → `l`** | `______` |

Balance it twice, approaching from opposite sides. If the two disagree by more
than a few percent, there is friction in the knife edge — use something sharper.

### `l_w` — pivot to wheel axis

A direct measurement, no balancing needed: pivot edge to the centre of the motor
shaft.

**`l_w` = `______` m**

---

## 3. Body inertia `I_b`, by swing test

Hang the cube from its pivot edge so it swings freely as a physical pendulum,
and give it a **small** amplitude — under ~10°, or the small-angle assumption
behind the formula stops holding and the measured period comes out long.

Do this with the **wheel mounted but locked**, then subtract the wheel's
contribution. Time 20 full oscillations and divide; do it twice and average.

| Run | 20 oscillations (s) | → T = t/20 (s) |
|---|---|---|
| 1 | `______` | `______` |
| 2 | `______` | `______` |
| **mean T** | | `______` |

Then, for a physical pendulum:

```
I_pivot = (m_total · g · l_total · T²) / (4π²)      with g = 9.80665
I_b     = I_pivot − m_w · l_w²
```

where `m_total` and `l_total` are for the cube **as swung** (wheel included).
`solve_gains.py` does this arithmetic — record `T` in the block below and let
it compute, rather than doing it by hand here.

| | |
|---|---|
| **T_swing** (mean period, s) | `______` |

Only ~5% accuracy is needed. The LQR gains are not sensitive to this at the
third significant figure, and the tuning ramp on the rig absorbs the rest.

---

## 4. Wheel inertia `I_w`

Pick whichever applies and record the result.

- **Solid disc:** `I_w = ½ · m_w · r²`. Measure `r` and let the script compute
  it — fill in `wheel_radius_m` below and leave `I_w` as `nan`.
- **Spoked or rimmed wheel, or a CAD value:** fill in `I_w` directly and leave
  `wheel_radius_m` as `nan`. A CAD number is usually better than a bench
  measurement here.

Fill in exactly one of the two. The script refuses if both or neither are set,
because silently preferring one would hide a typo in the other.

| | |
|---|---|
| Wheel geometry (disc / spoked / rimmed / other) | `____________` |
| Where `I_w` came from, if given directly | `____________________` |

---

## 5. Not measured — computed or constant

Listed so nobody goes looking for them with a ruler. `solve_gains.py` prints
these; they are not inputs.

| Symbol | Value | Where from |
|---|---|---|
| `g` | 9.80665 m/s² | defined SI constant |
| `K_t` | 60 / (2π · kv) ≈ **0.0256** Nm/A | kv = 373.6, from `data/calibration/moteus-cal-AD4AN1QwUBYgOTNO-*.log` |
| torque ceiling | `K_t · servo.max_current_A` | the firmware reads `servo.max_current_A` back over CAN at boot and refuses to run if `CTL_MAX_TORQUE_NM` exceeds it |

---

## THE VALUES

Everything above is working; this is the answer. `solve_gains.py` reads **only**
this block — plain `name value` lines, the same shape as
`config/current_config.cfg.in`. Replace each `nan` with a number. Do not rename
the keys, and do not add a second copy of this block anywhere.

```measurements
# name              value      unit    where it came from
m_b                 0.1970     # kg    Simscape Build Guide, "Derived plant": m_panel
m_w                 0.0661     # kg    Simscape Build Guide, "Derived plant": m_wheel
l_total             0.1017     # m     Simscape Build Guide, "Derived plant": l (assembly CoM, wheel on)
I_pivot             3.399e-3   # kg m^2  Simscape Build Guide: Theta, locked-wheel 2nd moment about pivot
I_w                 1.858e-4   # kg m^2  Simscape Build Guide: I_w, wheel inertia about spin axis (CAD)
l                   nan        # m     unused -- direct geometry path (l_total, I_pivot) used instead
l_w                 nan        # m     unused -- direct geometry path (l_total, I_pivot) used instead
T_swing             nan        # s     unused -- direct geometry path (l_total, I_pivot) used instead
wheel_radius_m      nan        # m     unused -- I_w given directly
```

---

## After the solve

Paste the printed flags into `[env:cube_balancer]` in `platformio.ini`, and
record what was pasted here so this file stays the whole story:

| | |
|---|---|
| Gains pasted on | (this commit) |
| `k_theta` | -1.12294 |
| `k_theta_dot` | -0.123992 |
| `k_omega` | -0.00173205 |
| Q, R used (if not the defaults) | `--q 25 0.25 0.0025 --r 833.333333` -- Bryson's rule from `cubli_panel_simscape_gates.m` Gate 7: `theta_max=0.20 rad`, `rate_max=2.0 rad/s`, `omega_des=0.5*omega_cap=20 rad/s`, `rho=12`, `tau_cont=0.12`. NOT the script's own defaults (`100 1 0.01` / `1.0`) -- those were never used. |

**Cross-check against the independently-run MATLAB solve** (same `cubli_panel_simscape_gates.m`, Gate 7): `K = [-1.0998, -0.1232, -0.001732]`. Agrees with the `solve_gains.py` run above to within ~2% on `k_theta`/`k_theta_dot` and to 5 significant figures on `k_omega` -- the residual is consistent with the Build Guide only printing 4-5 significant figures for its intermediate quantities (`ℓ`, `m_total`, `Θ̄`), not a disagreement in the underlying math. Both independently confirmed stable (see `solve_gains.py`'s own eigenvalue check above); the Build Guide's own stated closed-loop poles (`-9.72 ± 0.76j, -5.05`) could NOT be reproduced from its own stated A, B, K -- recomputing gives three real poles instead, which looks like a transcription slip in that one line of the guide, not a problem with K. Worth a second look on the MATLAB side, but does not change what to paste here.

### The sign — record what N1 showed

The solve fixes the gain **magnitudes**. It cannot fix their **sign**: what a
positive `feedforward_torque` physically does to the cube depends on the motor
phase order and `SOURCE0_SIGN`, which no model can see. Note also that the sign
the solve produces contradicts `BalancingController.hpp`'s per-field comment —
see [README.md §4](README.md), "The sign is not decidable here". Treat both as
unverified until the rig says otherwise.

| | |
|---|---|
| N1 run on | `____________` |
| Tilting **toward** \_\_\_\_\_\_ made `theta` go | `positive / negative` |
| The shaft pushed | `the correcting way / the wrong way` |
| `EST_INVERT_THETA` ended up | `true / false` |
| Final sign of `k_theta` | `positive / negative` |

If the shaft pushed the wrong way, the fix is `EST_INVERT_THETA` (or
`SOURCE0_SIGN`) — **never** negating the control law. Negating it leaves the
damping term pointing the wrong way and buries the real bug somewhere worse.

Then the rig, per [README.md §4 "Tuning procedure"](README.md): the firmware
boots at `gain_scale` 0.1 and steps through 0.3, 0.6, 1.0 with `+`, one run per
step. Tune `k_omega` **last**, after the cube balances at all.

**A solve is not a validation.** These gains come from a linearised model of a
rig measured with a kitchen scale. They are a starting point that is the right
order of magnitude — nothing more.
