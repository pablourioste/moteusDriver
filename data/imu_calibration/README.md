# BMI270 calibration logs

Raw PuTTY session logs from `firmware/imu/imu_calibrate`, kept for the same
reason `data/calibration/` keeps the moteus ones: the numbers compiled
into the firmware are meaningless without the session that produced
them, and a calibration you cannot audit is not a calibration.

File naming mirrors the moteus logs: `imu-cal-<UTC timestamp>.log`.

## Current calibration — 2026-08-05T12:50:13

Source: [imu-cal-20260805T125013.log](imu-cal-20260805T125013.log)
IMU loose on the bench, not mounted. 23.5-23.7 °C.

```
-D GYRO_BIAS_X=0.002348
-D GYRO_BIAS_Y=-0.001140
-D GYRO_BIAS_Z=-0.000762

-D ACCEL_OFFSET_X=-0.092282
-D ACCEL_SCALE_X=0.991085
-D ACCEL_OFFSET_Y=-0.196510
-D ACCEL_SCALE_Y=0.991035
-D ACCEL_OFFSET_Z=0.050664
-D ACCEL_SCALE_Z=1.003787
```

Applied as `a_corrected = (a_raw - offset) / scale`, and
`omega_corrected = omega_raw - bias`.

### Why these are trusted

**Gyro.** 796 samples, worst sd 0.00142 rad/s -- an order of magnitude
under the 0.02 motion-rejection threshold, so the rig really was still.
`v` then confirmed the correction: residuals of -0.000086 / +0.000018 /
-0.000144 rad/s, against a 0.002 target.

An earlier session (12:26, superseded) measured the gyro twice and got
+0.002575 / -0.001301 / -0.001162. Agreement with this session to
~0.0002 rad/s across a 24-minute gap is the real evidence the number is
a bias and not an artefact.

**Accel.** All six positions captured within 0.9-1.6 deg of square, well
inside the 3 deg limit. Applying the fit back to the six captures
returns 9.8066 m/s^2 on every one -- worst error 0.0000, against
IMU_SETUP.md Check 6's threshold of 0.05.

### Reading the offsets

The offsets are genuinely large, and that is a real finding rather than
a measurement error:

| Axis | +g | -g | offset | scale |
|---|---|---|---|---|
| X | +9.6269 | -9.8115 | -0.0923 | 0.99108 |
| Y | +9.5222 | -9.9152 | -0.1965 | 0.99103 |
| Z | +9.8945 | -9.7931 | +0.0507 | 1.00379 |

Y is offset by ~0.2 m/s^2, i.e. 2% of g. What proves this is sensor
offset and not a tilted board: **tilt can only ever reduce a magnitude**
(it scales by cos theta), and at the measured 1.0-1.6 deg the maximum
possible loss is 0.0038 m/s^2. But -Y reads 9.9152 and +Z reads 9.8945,
both *above* g -- which no tilt can produce. The pairs are shifted, not
shrunk. That is the signature of offset.

Scales sit within 0.9% of 1.0, comfortably inside the sketch's 5%
warning band.

## What is NOT covered here

**Mounting misalignment.** These logs were taken with the IMU loose on
the bench. Once it is fixed into the cube, any angle between the sensor
frame and the body frame is a *rotation*, and it must be applied after
this offset/scale correction. Folding it into these offsets corrupts
both -- an offset that cancels the lean at one orientation is wrong at
every other one. Measure it geometrically at assembly.

**Temperature drift.** Gyro bias moves with temperature, which is why
`calibrateGyro()` prints the temperature beside it. These were taken at
23.5 °C on a cold bench. If the rig runs warm and the estimator drifts,
re-measure at operating temperature -- that is a property of gyros, not
a defect in these numbers.

## Superseded

A first session at 12:26 the same day produced an accel fit that was
**discarded**. Its six positions were taken 3-14 deg off square, and
because tilt scales readings by cos theta, the fit absorbed the tilt as
fake offset and scale (`ACCEL_OFFSET_Y=-0.194` was mostly positioning
error). Its gyro half was sound.

That session is why `captureAccelPosition()` now gates on off-axis
magnitude rather than on the target axis alone: the two failures are
indistinguishable on the target axis, but only one of them should be
rejected. See the comment block in
`firmware/imu/imu_calibrate/main.cpp`.
