# Python Probe Findings For C Inference

## Purpose

This is a handoff summary for implementing the DK1 inference path in C. It
collects the conclusions from the Python probe work and separates what looks
ready to port from what should remain experimental.

Detailed equations live in `docs/dk1_tracker_inference_algorithm.md`. This file
is the shorter engineering memo for the next implementation thread.

## Current Best Understanding

The DK1 IMU can support a useful orientation and head/neck pose-like estimate:

- gyro integration gives the high-rate orientation path,
- accelerometer initializes roll/pitch and provides residual diagnostics,
- magnetometer can provide continuous yaw correction after calibration,
- a Gamma head/neck model can produce eye midpoint and look direction,
- pivot acceleration can be estimated diagnostically,
- pivot position is not observable enough to use as real 6-DOF tracking.

The C implementation should therefore target orientation, heading correction,
look direction, and fixed-pivot or weakly constrained head/neck geometry first.
It should not present integrated pivot position as a reliable positional
tracker.

## Coordinate Convention To Port

Use the sane world convention adopted by the probe:

```text
+X = right
+Y = up
-Z = forward
g_W = (0, -9.80665, 0)
```

The DK1 accelerometer is treated as body-frame specific force. A stationary
level headset should be near:

```text
(0, +9.80665, 0)
```

This is not a contradiction. With `g_W = (0, -g, 0)`, stationary specific force
is:

```text
-R^T g_W
```

and is positive along body +Y for an identity orientation.

## Time Handling

The DK1 CSV/report path can contain repeated host timestamps because one USB
report contains multiple motion samples. The Python probe expands repeated
`host_time` groups into per-sample times before integration.

For C, preserve this behavior:

- keep the report timestamp for the host/report event,
- synthesize monotonic per-sample times inside repeated groups,
- reject or reuse the previous state for nonpositive time steps,
- cap unusually large integration steps for stability.

The probe cap is currently:

```text
MAX_DT_S = 0.02
```

## Gyro Bias And Orientation

Stationary recordings are useful for estimating gyro bias. The C path should
support an explicit bias vector, and eventually a stationary-calibration helper.

The current probe uses trapezoidal gyro integration:

```text
omega_i = 0.5 * ((gyro_{i-1} - bias) + (gyro_i - bias))
q_i = normalize(q_{i-1} * quat_from_body_rate_step(omega_i, dt))
```

This reduced drift slightly compared with using only the previous sample. It is
cheap and should be the C default.

Initial orientation is built by aligning the first normalized accelerometer
sample to world up:

```text
q0 = quat_from_two_vectors(normalize(accel_0), WORLD_UP)
```

This fixes initial roll/pitch. Initial yaw is arbitrary until magnetometer
north is established.

## Magnetometer Findings

The raw DK1 magnetometer path needs calibration and axis handling before it is
useful.

For `gentle_rotation_30s.csv`, the useful diagnostic mapping was:

```text
body mag (x, y, z) = raw mag (x, z, y)
```

which corresponds to:

```text
mag_axis_order = 0, 2, 1
mag_axis_signs = 1, 1, 1
```

The hard-iron sphere-fit center for that recording was approximately:

```text
(5470.7, 5644.7, 10044.0) raw counts
```

This was enough to reduce heading residuals dramatically, but it is only a
diagnostic hard-iron correction. It is not a full magnetometer calibration.

The C implementation should expose:

- hard-iron center,
- axis order,
- axis signs,
- heading residual diagnostics,
- a way to disable magnetometer correction entirely.

## Continuous Yaw Correction

The best current magnetometer correction method is:

1. Integrate gyro-only orientation first, or initialize expected north from an
   early window.
2. Calibrate each magnetometer sample.
3. Rotate measured magnetometer north into world coordinates with the current
   orientation.
4. Project it onto the horizontal plane.
5. Compare it with a fixed expected world-north vector.
6. Apply only a gradual yaw correction about world +Y.

Repeated magnetometer rows must not be counted as fresh measurements. The probe
stores a pending heading error only when a new magnetometer vector arrives, then
spends that pending correction gradually:

```text
fraction = 1 - exp(-correction_rate * dt)
heading_step = pending_heading_error * fraction
q = quat_from_axis_angle(WORLD_UP, -heading_step) * q
pending_heading_error -= heading_step
```

This makes the correction independent of IMU row rate and avoids overweighting
stale magnetometer samples.

On `gentle_rotation_30s.csv` with hard-iron fitting and axis order `0 2 1`, the
probe saw:

```text
rate 0.0 1/s: mean abs 2.087 deg, max 9.179 deg
rate 0.5 1/s: mean abs 1.828 deg, max 6.440 deg
rate 1.0 1/s: mean abs 1.616 deg, max 4.852 deg
rate 2.0 1/s: mean abs 1.252 deg, max 3.068 deg
rate 5.0 1/s: mean abs 0.688 deg, max 1.487 deg
```

Higher correction rates fit this recording better, but may follow local
magnetic distortion more aggressively. Keep the rate configurable.

## Body-Frame North Lesson

We tried directly comparing/averaging magnetometer vectors in the body frame.
That was misleading because the body frame rotates. The correct alternatives
are:

- map magnetometer samples into world coordinates before averaging, or
- track the expected inertial north vector in the body frame and compare against
  instantaneous body-frame magnetometer samples projected perpendicular to
  tracked up.

Do not average raw body-frame magnetometer vectors over long windows while the
headset is rotating.

## Head/Neck Geometry Findings

The shared geometry model is Gamma-shaped:

```text
r = (0, h, -ell)
```

where:

```text
h   = vertical neck-pivot to tracker/eye-midpoint distance
ell = forward neck-axis to tracker/eye-midpoint distance
```

The tracker is approximated at the eye midpoint.

Recent fits on the gentle rotation capture clustered around:

```text
h   ~= 72-74 mm
ell ~= 158-159 mm
```

The fixed-pivot gentle-rotation animation used:

```text
h = 72.6 mm
ell = 158.8 mm
mag correction rate = 2.0 1/s
```

Treat these as plausible values from one recording, not universal constants.

## Neural Network Findings

We trained temporal CNN regressors on synthetic fixed-length 5 second runs to
predict `h` and `ell`. The CNN was easy to overfit:

- training loss kept falling,
- validation loss remained high or improved only very slowly,
- reducing network size helped regularization but did not solve the core issue.

The important conclusion is that the geometry signal exists, but the raw CNN was
not reliably extracting it. The next C implementation should not depend on a
neural network for calibration.

The CNN work still helped by exposing the need for stronger physics-derived
features and better excitation/window selection.

## Physics Least-Squares Findings

For clean simulated data with known orientation and no pivot motion, the
geometry is identifiable with a simple linear least-squares fit:

```text
accel + R^T g_W ~= ([alpha]_x + [omega]_x [omega]_x) r
r = (0, h, -ell)
```

This baseline recovered `h` and `ell` when the trajectory had enough angular
velocity and angular acceleration.

When pivot acceleration is known in simulator debug columns, subtracting it
also makes the fit work. This confirms that the equations and sign convention
are basically sound.

## Pivot Motion Findings

Unknown pivot motion is the main unidentifiability problem.

The accelerometer residual contains both:

```text
rotational acceleration from alpha/omega around the neck lever arm
linear acceleration of the pivot
```

Without external position/velocity information, these terms can trade off
against each other. Long slow recordings are especially weak because angular
accelerations are small.

We tried nuisance models for pivot motion:

- no pivot motion,
- known pivot acceleration from simulator debug columns,
- constant pivot acceleration,
- constant jerk pivot acceleration.

Constant jerk can help on short windows, but it does not solve general real
motion because the real trajectory is not constant jerk over long durations.

The strongest practical result was: fit geometry on short high-excitation
windows instead of long slow windows.

## Fixed-Pivot Visualization Finding

Double-integrating inferred pivot acceleration can produce visually large,
unrealistic drift. For the gentle rotation recording, the person was seated and
actual translation should have been small, but one nearly straight segment with
low acceleration and nonzero inferred velocity dominated the integrated pivot
path.

For visualization and C pose output, prefer:

```text
orientation + fixed-pivot head/neck geometry
```

over:

```text
orientation + double-integrated pivot position
```

unless another sensor or constraint is added.

## Recommended C Implementation Order

1. Port vector/quaternion math.
2. Port y-up constants and accelerometer specific-force convention.
3. Port sample-time expansion for repeated report timestamps.
4. Port gyro-bias subtraction and trapezoidal orientation integration.
5. Port initial orientation from accelerometer.
6. Port predicted specific force and accelerometer residual diagnostics.
7. Port head/neck fixed-pivot eye midpoint and look direction.
8. Port magnetometer hard-iron/axis calibration.
9. Port world-frame expected north and heading residual diagnostics.
10. Port continuous magnetometer yaw correction with fresh-sample detection.
11. Port rotational acceleration and pivot-acceleration diagnostics.
12. Defer integrated pivot position or label it diagnostic only.
13. Later, add geometry fitting from selected high-excitation windows.

## Suggested C API Outputs

The real-time or offline C inference state should expose at least:

```text
time
orientation quaternion
unbiased gyro
angular acceleration
look direction world
eye midpoint world or fixed-pivot eye offset
predicted specific force
accelerometer residual
magnetometer calibrated vector
expected north world
heading residual degrees
mag correction rate and update count
pivot acceleration diagnostic
```

If pivot position is included, name it clearly as diagnostic/drift-prone.

## Validation Targets For C

Use the Python probe as the oracle for first C validation. For the same CSV and
parameters, C should match Python within small floating-point tolerances for:

- initial orientation,
- integrated quaternion path,
- predicted specific force,
- heading residuals,
- look direction,
- fixed-pivot eye midpoint,
- pivot acceleration diagnostic.

The most useful initial real-data command to mirror is:

```bash
cd /Users/jma/dev/OculusRiftDK1/probe
.venv/bin/python probe_head_model.py ../gentle_rotation_30s.csv \
  --stationary-bias ../stationary_30s.csv \
  --mag-fit-hard-iron \
  --mag-axis-order 0 2 1 \
  --mag-correction-rate 2 \
  --csv
```

## Things Not To Port Yet

Do not make these first-class C features yet:

- neural network geometry inference,
- double-integrated pivot position as real tracking,
- full 6-DOF pivot tracking,
- hard-coded recording-specific magnetometer center,
- long-window body-frame magnetometer averaging.

Keep them either out of the C implementation or behind explicit experimental
diagnostic flags.

## Source Anchors In The Python Probe

Current Python source anchors:

```text
probe/model/math3d.py
probe/model/head_neck.py
probe/inference/tracker/dk1_csv.py
probe/inference/tracker/state.py
probe/probe_head_model.py
probe/fit_head_neck_ls.py
probe/train_head_neck_cnn.py
```

Generated/synthetic dataset anchors:

```text
probe/datasets/hmd_lhs_3000_5s_sensor_readings.csv
probe/datasets/hmd_lhs_3000_5s_run_parameters.csv
probe/datasets/hmd_lhs_3000_5s_clean_no_pivot_sensor_readings.csv
probe/datasets/hmd_lhs_3000_5s_clean_no_pivot_run_parameters.csv
```

Real capture anchors:

```text
gentle_rotation_30s.csv
stationary_30s.csv
```

## Bottom Line

The C inference path should be an orientation and heading-corrected head/neck
model, not a full 6-DOF tracker. The reliable core is:

```text
gyro orientation + accelerometer roll/pitch initialization
+ calibrated magnetometer yaw correction
+ fixed-pivot Gamma head/neck geometry
```

The experimental layer is:

```text
accelerometer residual -> rotational acceleration diagnostic
-> short-window h/ell fitting
```

The unreliable layer is:

```text
unknown pivot acceleration -> double-integrated pivot position
```

Keep that boundary clear in the C API and diagnostics.
