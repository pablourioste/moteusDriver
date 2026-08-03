# Motor calibration logs

Output of `moteus_tool -t 1 --calibrate`, one JSON file per run. These are
**measurements of the physical motor**, which is why they are version
controlled: they are the only record of how the characterisation has drifted,
and several numbers elsewhere in the project are derived from them.

Regenerate with:

```bash
./moteus-venv/bin/moteus_tool -t 1 --calibrate
./moteus-venv/bin/moteus_tool -t 1 --calibrate --cal-motor-speed 1.0   # slower
```

⚠️ The motor spins freely in both directions at speed during calibration.
Unbolt anything on the shaft — including the reaction wheel.

⚠️ Calibration needs `servopos.position_min/max` at `nan`. It issues
`d pos nan 0 nan c<current> b1`, which is a *position mode* command and so is
subject to the travel limits. With them at ±1 rev the phase sweep never
advances and calibration fails with a `ZeroDivisionError` on `poles == 0`.
See [`../../docs/SETUP.md`](../../docs/SETUP.md) Phase 1.5.

---

## This motor

| Field | Meaning | Used by |
|---|---|---|
| `poles` | Magnetic pole count (not pole *pairs*) | `MoteusConfig::motor_poles` |
| `kv` | RPM per volt, no load | `K_t = 60/(2π·kv)` → torque ceiling |
| `winding_resistance` | Phase resistance, Ω | firmware current loop |
| `inductance_d` / `_q` | Phase inductance, H | firmware current loop |
| `pid_dq_kp` / `_ki` | Current-loop gains the tool derived | firmware |
| `offset[64]` | Encoder-to-phase commutation table | firmware |
| `fit_metric` | Residual of that table fit — **lower is better** | quality check |
| `current_quality_factor` | Current measurement quality | quality check |

## Results across runs

| Run | poles | kv | R (mΩ) | L_d (µH) | fit_metric | quality |
|---|---|---|---|---|---|---|
| 2026-08-02 09:11 | 24 | 368.1 | 112.1 | 50.9 | 28.0 | 262.6 |
| 2026-08-02 14:29 | 24 | 344.6 | 99.0 | 46.6 | 32.1 | 370.7 |
| 2026-08-03 09:51 | 24 | **373.6** | 102.1 | 45.8 | **20.0** | 115.1 |

**`poles = 24` is solid** — identical across all three runs, which is what you
want from a quantity that is physically discrete and cannot be anything else.

**`kv` spreads about 8%** (344.6 → 373.6). Some of that is genuine
measurement noise; kv is inferred from back-EMF during a short spin and is
sensitive to temperature and to how freely the shaft turned. The newest run
also has the best `fit_metric` (20.0, lowest of the three), so **373.6 is the
value to trust**.

### Why the spread matters

The torque ceiling comes from kv:

```
K_t = 60 / (2π · kv)

  kv = 344.6  →  K_t = 0.0277 Nm/A  →  15 A gives 0.416 Nm
  kv = 373.6  →  K_t = 0.0256 Nm/A  →  15 A gives 0.384 Nm
```

An 8% error in kv is an 8% error in every torque number downstream — the
control law's `max_torque_nm`, the LQR input scaling, and the recoverable
tilt envelope that sets `max_tilt_rad`.

**Known issue:** `CMakeLists.txt` cites kv = 344.6, the *middle* of the three
and not the most recent. Its derived no-load speed is therefore understated
by roughly 8%. Worth re-deriving from 373.6.

### Reading the quality numbers

`fit_metric` is the residual of the 64-point commutation table fit. Lower is
better; a large value means the encoder-to-phase relationship was not
captured cleanly and commutation will be rough.

`current_quality_factor` varies widely here (115–371). Together with the best
`fit_metric` in the newest run, the picture is consistent: that run had the
cleanest table fit.

If you recalibrate and `fit_metric` climbs well above ~30, suspect a loose
encoder magnet, a marginal SPI cable, or mechanical load on the shaft — and
recalibrate before trusting the result.

---

## Device identity

All three logs report the same board — serial `AD4AN1QwUBYgOTNO`, UUID
`b9fd22c5-bc3e-4fa4-9028-c23955bc57b0`. The `git_hash` field records the
firmware build the calibration ran against; a firmware update is a good
reason to recalibrate.
