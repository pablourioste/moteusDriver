// imu_axis_verify -- finds StateEstimator::Config's pivot_axis_x/y/z /
// invert_theta / theta_offset on the MOUNTED rig, by fitting them from
// three static poses. Prerequisite for cube_balancer / cube_balancer_torque,
// which refuse to boot while any of these are unset.
//
// WHY A FIT, NOT AN AXIS PICK: the old scheme asked "which raw sensor axis
// (0/1/2) is the pivot, and which two span the swing plane" -- which only
// works if the IMU happens to be mounted flush with the body frame. A
// tilted mount -- the IMU sitting at some angle on the panel, the normal
// case, not the exception -- has NO raw axis parallel to the physical
// balance edge, so no such pick can represent it.
//
// The fix follows from one fact: the pivot edge is horizontal and gravity
// is vertical, so gravity's component ALONG the pivot axis is exactly zero
// at every tilt angle, for ANY mounting. That means every accelerometer
// reading, held at any static tilt, lies in the plane perpendicular to the
// pivot axis. Two non-parallel readings span that plane, so their cross
// product gives the pivot axis directly:
//
//   pivot_axis = normalize(a_left x a_right)
//
// This sketch captures exactly the three poses you would naturally want to
// check anyway -- equilibrium, left limit, right limit -- and does that
// arithmetic for you, calling the real core/StateEstimator::accelAngle()
// (not a reimplementation) so what it reports is exactly what cube_balancer
// will compute. See README.md "Known issues".
//
//   pio run -e imu_axis_verify        (WSL) build, flash + monitor from Windows
//
// Wiring unchanged (IMU_SETUP.md step 1):
//   BMI270  VDD->3.3V  GND->GND  SCK->13  SDI->11  SDO->12  CSB->10
//
// Procedure:
//   1. Mount the IMU on the rig exactly as it will run.
//   2. 'l' -- raw live stream, just to confirm the sensor looks sane
//      (|a| near 9.81 m/s^2) before spending time on the fit.
//   3. Hold the rig AT the balance point, hold still, press 'e'.
//   4. Tilt to the LEFT limit, hold still, press '2'.
//   5. Tilt to the RIGHT limit, hold still, press '3'.
//   6. Press 'f' -- fits pivot_axis from the three captures and derives
//      theta_offset from the equilibrium pose, so theta reads ~0 there by
//      construction. It also flags a bad fit: left/right too close to
//      parallel, or the equilibrium pose not actually perpendicular to the
//      fitted axis (a sign a pose was mis-held, or the rig moved on
//      something other than the intended tilt).
//   7. 's' prints theta at all three poses. THE SIGN -- this cannot be
//      checked automatically. theta must be POSITIVE at whichever pose is
//      the side a positive wheel torque should correct. If it's reversed,
//      press 'i' (toggles invert_theta) and check 's' again -- no need to
//      re-tilt or re-capture, it recomputes from the stored poses. Getting
//      this wrong before N1 is the single most dangerous unset value in
//      the project -- see docs/3d_scaling/README.md s4, "The sign is not
//      decidable here".
//   8. 'd' -- live dashboard using the fitted mapping. Sweep slowly through
//      the full range and confirm theta moves smoothly with no jump near
//      either limit.
//   9. 's' any time prints paste-ready EST_* flags for
//      platformio.ini [env:cube_balancer].
//
// Menu:
//   l  raw live stream (accel/gyro, all 3 axes)
//   e  capture EQUILIBRIUM pose (hold still ~1.5 s)
//   2  capture LEFT limit pose
//   3  capture RIGHT limit pose
//   f  fit pivot_axis + theta_offset from the three poses
//   d  dashboard: live theta with the fitted mapping
//   i  toggle invert_theta (recomputes reported theta, no re-capture needed)
//   s  summary + paste-ready EST_* flags
//   ?  this menu
//
// tau, rate_cutoff_hz, nominal_dt, warmup_samples and accel_gate are NOT
// covered here -- accelAngle() does not use them. They stay separate
// TODOs; see StateEstimator.hpp.

#include <Arduino.h>

#include <cmath>
#include <string>

#include "core/StateEstimator.hpp"
#include "core/Types.hpp"
#include "embedded/Bmi270SpiDriver.hpp"

namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr double kGravity = 9.80665;

cube::Bmi270SpiDriver* g_imu = nullptr;

// Working config for the fit. pivot_axis_x/y/z start unset (NaN) -- unlike
// the old index-based scheme, there is no plausible arbitrary default to
// search from; the fit either has all three poses or it does not run.
cube::StateEstimator::Config g_cfg;
bool g_fitted = false;

struct Pose {
  bool valid = false;
  double a[3] = {NAN, NAN, NAN};
};
Pose g_eq, g_left, g_right;

struct Stats3 {
  double mean[3];
  double sd[3];
  long count;
};

// Mean and standard deviation of the accelerometer over `seconds`. sd is
// the motion detector -- the same role it plays in imu_calibrate.
Stats3 collectAccel(double seconds) {
  double sum[3] = {0, 0, 0};
  double sq[3] = {0, 0, 0};
  long count = 0;

  const uint32_t start = millis();
  const uint32_t duration_ms = static_cast<uint32_t>(seconds * 1000.0);
  while (millis() - start < duration_ms) {
    const cube::ImuData s = g_imu->read();
    if (!s.valid) { continue; }
    sum[0] += s.accel_x;  sq[0] += s.accel_x * s.accel_x;
    sum[1] += s.accel_y;  sq[1] += s.accel_y * s.accel_y;
    sum[2] += s.accel_z;  sq[2] += s.accel_z * s.accel_z;
    count++;
    delay(2);
  }

  Stats3 st;
  st.count = count;
  for (int i = 0; i < 3; i++) {
    st.mean[i] = (count > 0) ? sum[i] / count : NAN;
    const double var =
        (count > 0) ? fmax(0.0, sq[i] / count - st.mean[i] * st.mean[i]) : NAN;
    st.sd[i] = sqrt(var);
  }
  return st;
}

void countdown(const char* what, int seconds) {
  Serial.print("  ");
  Serial.print(what);
  Serial.print(" -- hold still: ");
  for (int i = seconds; i > 0; i--) {
    Serial.print(i);
    Serial.print(" ");
    delay(1000);
  }
  Serial.println("measuring...");
}

void capturePose(Pose* p, const char* label) {
  Serial.println();
  Serial.printf("--- Capture: %s ---\r\n", label);
  countdown(label, 3);
  const Stats3 st = collectAccel(1.5);

  if (st.count < 50) {
    Serial.printf("  FAIL -- only %ld samples; is the IMU responding?\r\n",
                  st.count);
    return;
  }
  const double worst_sd = fmax(st.sd[0], fmax(st.sd[1], st.sd[2]));
  if (worst_sd > 0.5) {
    Serial.printf("  FAIL -- moved (accel sd %.3f m/s^2). Hold steadier.\r\n",
                  worst_sd);
    return;
  }

  const double mag = sqrt(st.mean[0] * st.mean[0] + st.mean[1] * st.mean[1] +
                          st.mean[2] * st.mean[2]);
  p->valid = true;
  p->a[0] = st.mean[0];
  p->a[1] = st.mean[1];
  p->a[2] = st.mean[2];
  Serial.printf("  accel = %+.4f %+.4f %+.4f   |a| = %.4f\r\n", p->a[0],
                p->a[1], p->a[2], mag);
  if (fabs(mag - kGravity) > 0.5) {
    Serial.println("  WARNING |a| is far from 9.81 -- check the accel "
                    "calibration or that the rig was actually still.");
  }
  Serial.printf("  %s captured.\r\n", label);
  g_fitted = false;  // any previous fit is now stale
}

// The fit: pivot_axis = normalize(a_left x a_right), theta_offset from the
// equilibrium pose. See the file header for the derivation.
bool fitPivotAxis() {
  if (!g_eq.valid || !g_left.valid || !g_right.valid) {
    Serial.println("  Need all three poses captured first -- e, 2, 3.");
    return false;
  }

  double n[3] = {
      g_left.a[1] * g_right.a[2] - g_left.a[2] * g_right.a[1],
      g_left.a[2] * g_right.a[0] - g_left.a[0] * g_right.a[2],
      g_left.a[0] * g_right.a[1] - g_left.a[1] * g_right.a[0],
  };
  const double n_mag = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
  if (n_mag < 1.0) {  // |a_left| * |a_right| * sin(angle) ~= 96 * sin(angle)
    Serial.println("  FAIL -- left and right poses are nearly parallel.");
    Serial.println("  Tilt further apart before capturing 2 and 3.");
    return false;
  }
  n[0] /= n_mag;  n[1] /= n_mag;  n[2] /= n_mag;

  // Consistency check: gravity's component along the TRUE pivot axis is
  // exactly zero at every tilt angle (pivot horizontal, gravity vertical).
  // A large residual here means a pose was not a pure rotation about the
  // edge -- the rig slid, was lifted, or wasn't actually still.
  const double eq_dot_n = g_eq.a[0] * n[0] + g_eq.a[1] * n[1] + g_eq.a[2] * n[2];
  Serial.println();
  Serial.printf("  fitted pivot_axis = (%+.5f, %+.5f, %+.5f)\r\n", n[0], n[1],
                n[2]);
  Serial.printf("  equilibrium . pivot_axis = %+.4f m/s^2 (want near 0)\r\n",
                eq_dot_n);
  if (fabs(eq_dot_n) > 1.0) {
    Serial.println("  WARNING large residual -- the three poses may not");
    Serial.println("  share a single rotation axis. Re-capture, holding the");
    Serial.println("  rig still except for the intended tilt.");
  }

  g_cfg.pivot_axis_x = n[0];
  g_cfg.pivot_axis_y = n[1];
  g_cfg.pivot_axis_z = n[2];

  // theta_offset: read the RAW (pre-offset) accelAngle() at the equilibrium
  // pose and use it directly as the offset -- makes theta read ~0 there by
  // construction, via the exact formula cube_balancer runs.
  g_cfg.theta_offset = 0.0;
  cube::ImuData eq_sample;
  eq_sample.accel_x = g_eq.a[0];
  eq_sample.accel_y = g_eq.a[1];
  eq_sample.accel_z = g_eq.a[2];
  eq_sample.valid = true;
  const cube::StateEstimator probe(g_cfg);
  const double raw = probe.accelAngle(eq_sample);
  g_cfg.theta_offset = raw;

  Serial.printf("  theta_offset = %.5f rad (%.2f deg)\r\n", raw,
                raw * kRadToDeg);
  g_fitted = true;
  return true;
}

double thetaDegForPose(const Pose& p) {
  cube::ImuData s;
  s.accel_x = p.a[0];
  s.accel_y = p.a[1];
  s.accel_z = p.a[2];
  s.valid = true;
  const cube::StateEstimator probe(g_cfg);
  return probe.accelAngle(s) * kRadToDeg;
}

void showLive() {
  Serial.println();
  Serial.println("Raw live stream -- send any key to stop.");
  Serial.println("      ax      ay      az   |     gx      gy      gz   |    |a|");
  while (!Serial.available()) {
    const cube::ImuData s = g_imu->read();
    if (!s.valid) {
      Serial.println("  read() invalid -- bus error");
      delay(300);
      continue;
    }
    const double mag = sqrt(s.accel_x * s.accel_x + s.accel_y * s.accel_y +
                            s.accel_z * s.accel_z);
    Serial.printf("%8.3f%8.3f%8.3f | %7.3f%7.3f%7.3f | %7.3f\r\n", s.accel_x,
                  s.accel_y, s.accel_z, s.gyro_x, s.gyro_y, s.gyro_z, mag);
    delay(100);
  }
  while (Serial.available()) { Serial.read(); }
}

void showDashboard() {
  if (!g_fitted) {
    Serial.println("  No fit yet -- capture e/2/3 then press f first.");
    return;
  }
  Serial.println();
  Serial.println("Dashboard -- send any key to stop. Sweep left to right.");
  Serial.println("      ax      ay      az   |    theta (deg)");
  while (!Serial.available()) {
    const cube::ImuData s = g_imu->read();
    if (!s.valid) {
      Serial.println("  read() invalid -- bus error");
      delay(300);
      continue;
    }
    const cube::StateEstimator probe(g_cfg);
    const double theta_deg = probe.accelAngle(s) * kRadToDeg;
    Serial.printf("%8.3f%8.3f%8.3f | %10.3f\r\n", s.accel_x, s.accel_y,
                  s.accel_z, theta_deg);
    delay(100);
  }
  while (Serial.available()) { Serial.read(); }
}

void toggleInvert() {
  g_cfg.invert_theta = !g_cfg.invert_theta;
  Serial.printf("  invert_theta -> %s\r\n",
                g_cfg.invert_theta ? "true" : "false");
  if (g_fitted) {
    Serial.println("  Re-run 's' -- reported theta updates from the stored "
                    "poses, no re-capture needed.");
  }
}

void showSummary() {
  Serial.println();
  Serial.println("=== Axis fit summary ===");
  if (!g_fitted) {
    Serial.println("  Not fitted yet -- capture e/2/3, then press f.");
    return;
  }

  Serial.printf("  pivot_axis = (%+.5f, %+.5f, %+.5f)  invert=%s  "
                "theta_offset=%.5f rad\r\n",
                g_cfg.pivot_axis_x, g_cfg.pivot_axis_y, g_cfg.pivot_axis_z,
                g_cfg.invert_theta ? "true" : "false", g_cfg.theta_offset);
  Serial.println();

  const double eq_deg = thetaDegForPose(g_eq);
  const double left_deg = thetaDegForPose(g_left);
  const double right_deg = thetaDegForPose(g_right);
  Serial.printf("  equilibrium : %.2f deg\r\n", eq_deg);
  Serial.printf("  left  limit : %.2f deg\r\n", left_deg);
  Serial.printf("  right limit : %.2f deg\r\n", right_deg);

  Serial.println();
  const bool opposite_sign = (left_deg > 0) != (right_deg > 0);
  if (!opposite_sign) {
    Serial.println("  WARNING: left and right read the SAME sign. Re-check");
    Serial.println("  that the two poses were genuinely different tilts,");
    Serial.println("  and re-fit ('f') if you re-capture either one.");
  } else {
    const double asym = fabs(fabs(left_deg) - fabs(right_deg));
    Serial.printf("  left/right are opposite sign, good. |left|-|right| "
                  "= %.2f deg", asym);
    Serial.println(asym < 5.0 ? "  (roughly symmetric)"
                               : "  (large asymmetry -- check the rig is "
                                 "centred, or re-capture equilibrium)");
  }

  Serial.println();
  Serial.println("  THE SIGN -- not checked automatically. Tilt toward the");
  Serial.println("  side a POSITIVE wheel torque should correct: theta must");
  Serial.println("  read POSITIVE there. If not, press 'i', then 's' again");
  Serial.println("  -- no re-capture needed.");

  Serial.println();
  Serial.println("  ; paste into platformio.ini [env:cube_balancer]");
  Serial.printf("    -D EST_PIVOT_AXIS_X=%.6f\r\n", g_cfg.pivot_axis_x);
  Serial.printf("    -D EST_PIVOT_AXIS_Y=%.6f\r\n", g_cfg.pivot_axis_y);
  Serial.printf("    -D EST_PIVOT_AXIS_Z=%.6f\r\n", g_cfg.pivot_axis_z);
  Serial.printf("    -D EST_INVERT_THETA=%s\r\n",
                g_cfg.invert_theta ? "true" : "false");
  Serial.printf("    -D EST_THETA_OFFSET=%.6f\r\n", g_cfg.theta_offset);
  Serial.println();
  Serial.println("  tau, rate_cutoff_hz, nominal_dt, warmup_samples and");
  Serial.println("  accel_gate are separate TODOs -- see StateEstimator.hpp.");
}

void printMenu() {
  Serial.println();
  Serial.println("=== IMU axis fit (mounted rig, 3-pose method) ===");
  Serial.println("  l  raw live stream");
  Serial.println("  e  capture EQUILIBRIUM pose (hold still)");
  Serial.println("  2  capture LEFT limit pose");
  Serial.println("  3  capture RIGHT limit pose");
  Serial.println("  f  fit pivot_axis + theta_offset from the three poses");
  Serial.println("  d  dashboard: live theta with the fitted mapping");
  Serial.println("  i  toggle invert_theta");
  Serial.println("  s  summary + paste-ready EST_* flags");
  Serial.println("  ?  this menu");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println();
  Serial.println("=== IMU axis fit (3-pose method) ===");

  g_cfg.invert_theta = false;
#ifdef EST_INVERT_THETA
  g_cfg.invert_theta = EST_INVERT_THETA;
#endif

  // FromBuildDefaults() rather than a bare Config: it carries whatever
  // gyro bias / accel offset+scale calibration the build flags already
  // supply, so this sketch reads the same corrected samples cube_balancer
  // will.
  static cube::Bmi270SpiDriver imu(
      cube::Bmi270SpiDriver::Config::FromBuildDefaults());
  g_imu = &imu;

  Serial.print("initialising IMU... ");
  std::string error;
  if (!imu.initialize(&error)) {
    Serial.println("FAIL");
    Serial.println(error.c_str());
    while (true) { delay(1000); }
  }
  Serial.println("OK");

  printMenu();
}

void loop() {
  if (!Serial.available()) { return; }

  const char c = Serial.read();
  while (Serial.available()) { Serial.read(); }  // flush CR/LF
  if (c == '\n' || c == '\r') { return; }

  switch (c) {
    case 'l': showLive(); break;
    case 'e': capturePose(&g_eq, "equilibrium"); break;
    case '2': capturePose(&g_left, "left limit"); break;
    case '3': capturePose(&g_right, "right limit"); break;
    case 'f': fitPivotAxis(); break;
    case 'd': showDashboard(); break;
    case 'i': toggleInvert(); break;
    case 's': showSummary(); break;
    case '?': printMenu(); break;
    default:
      Serial.print("unknown: ");
      Serial.println(c);
      break;
  }
}
