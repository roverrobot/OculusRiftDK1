# DK1 Tracker Inference Algorithm

## Purpose

This note summarizes the current offline inference algorithm developed in the
Python probe under `probe/`. It is a snapshot of the method we are actually
testing, not a claim that the DK1 can provide full positional tracking.

The useful outputs are:

- headset orientation,
- world-space look direction,
- head/neck model points such as eye midpoint and neck-axis point,
- magnetometer heading diagnostics and optional yaw correction,
- accelerometer residuals,
- diagnostic pivot acceleration and drift-prone pivot position.

The current conclusion is important: orientation and heading correction are
reasonable, but pivot translation should not be treated as a calibrated 6-DOF
position estimate from DK1 IMU data alone.

## Coordinate Convention

Use a physical/VR-oriented world convention:

```text
+X = right
+Y = up
-Z = forward
```

Gravity in world coordinates is:

```math
g_W = (0, -g, 0), \qquad g = 9.80665 \mathrm{m/s^2}.
```

The DK1 accelerometer samples are handled as body-frame specific force. A
stationary, level headset therefore reports approximately:

```text
a_body ~= (0, +9.80665, 0)
```

This is consistent with a y-up world because the stationary specific force is:

```math
a_{\mathrm{specific},H} = -R^T g_W.
```

The quaternion `q(t)` maps headset/body-frame vectors into world-frame vectors:

```math
v_W = q(t) v_H q(t)^{-1}.
```

## Inputs

The inference pipeline consumes DK1 CSV rows with:

- host time,
- accelerometer,
- gyro,
- magnetometer,
- temperature.

Rows from the same report may share a repeated `host_time`. The probe expands
those repeated report times into per-sample times before integration.

Gyro bias is estimated from a separate stationary recording when available.

## Head/Neck Geometry

The shared model is a Gamma-shaped neck-to-eye model. In body coordinates:

```math
r_{NT} = (0, h, -\ell)
```

where:

```text
h   = vertical distance from neck pivot to tracker / eye midpoint
ell = forward distance from neck axis to tracker / eye midpoint
```

The tracker/IMU is currently approximated at the eye midpoint, so:

```math
r_{NT} \approx r_{NE}.
```

The default model remains a placeholder. Recent gentle-rotation window fits
clustered near:

```text
h   ~= 72-74 mm
ell ~= 158-159 mm
```

Those numbers are useful experimental anchors, not yet a general calibration.

## Initial Orientation

The first accelerometer sample is normalized and aligned with world up:

```math
q_0 = \mathrm{quat\_from\_two\_vectors}
\left(
  \frac{a_0}{|a_0|},
  (0,1,0)
\right).
```

This initializes roll and pitch from gravity/specific-force direction. Initial
yaw is still arbitrary until the magnetometer establishes an expected north
direction.

The initial predicted specific force is:

```math
\hat a_0 = -q_0^{-1} g_W q_0.
```

The initial residual is:

```math
e_0 = a_0 - \hat a_0.
```

## Gyro Orientation Integration

For each positive time step, the unbiased gyro is integrated into orientation.
The probe uses trapezoidal gyro sampling:

```math
\omega_i =
\frac12
\left[
  (\omega^{raw}_{i-1} - b_\omega)
  +
  (\omega^{raw}_{i} - b_\omega)
\right].
```

Then:

```math
\Delta q_i = \mathrm{quat\_from\_body\_rate\_step}(\omega_i, \Delta t),
```

```math
q_i = \mathrm{normalize}(q_{i-1}\Delta q_i).
```

Large time steps are capped for integration stability. Nonpositive time steps
reuse the previous orientation.

## Magnetometer Calibration And Diagnostics

The magnetometer path is deliberately explicit because the raw DK1 axes do not
appear to match the body convention directly.

The current diagnostic calibration supports:

- hard-iron center subtraction,
- optional hard-iron center fitting with an algebraic sphere fit,
- axis permutation,
- axis sign flips.

For `gentle_rotation_30s.csv`, the useful observed mapping was:

```text
body mag (x,y,z) = raw mag (x,z,y)
```

or:

```text
--mag-axis-order 0 2 1
```

The hard-iron sphere fit for that recording was approximately:

```text
(5470.7, 5644.7, 10044.0) raw counts
```

This is a recording-local diagnostic fit, not a full soft-iron calibration.

## Expected North

The probe first performs a gyro-only reference integration. It maps calibrated
magnetometer vectors into world coordinates:

```math
m_{W,i} = q_i m_{H,i} q_i^{-1}.
```

The initial expected north direction is the horizontal projection of the early
world-frame magnetometer average:

```math
n_W =
\mathrm{normalize}
\left(
  m_W - (m_W \cdot u_W)u_W
\right),
\qquad
u_W = (0,1,0).
```

This establishes a fixed world-frame north reference for diagnostics and yaw
correction.

## Continuous Magnetometer Yaw Correction

The current correction method maps each fresh measured magnetometer vector into
world coordinates using the current orientation estimate:

```math
\tilde n_{W,i} =
\mathrm{normalize}
\left(
  q_i m_{H,i} q_i^{-1}
  -
  \left[
    (q_i m_{H,i} q_i^{-1}) \cdot u_W
  \right]u_W
\right).
```

It then computes a signed heading error about world up:

```math
\theta_i =
\mathrm{atan2}
\left(
  u_W \cdot (n_W \times \tilde n_{W,i}),
  n_W \cdot \tilde n_{W,i}
\right).
```

Repeated magnetometer rows are not treated as fresh measurements. When a fresh
magnetometer reading arrives, the algorithm stores the current heading error as
a pending correction. Each subsequent gyro step applies only a fraction of that
pending error:

```math
f_i = 1 - \exp(-\lambda \Delta t),
```

```math
\Delta \theta_i = f_i \theta_{\mathrm{pending}}.
```

The orientation is corrected by a world-yaw quaternion:

```math
q_i \leftarrow
\mathrm{quat\_from\_axis\_angle}(u_W, -\Delta\theta_i) q_i.
```

Then:

```math
\theta_{\mathrm{pending}}
\leftarrow
\theta_{\mathrm{pending}} - \Delta\theta_i.
```

This makes the correction rate independent of the IMU row rate and avoids
overweighting repeated/stale magnetometer samples.

On `gentle_rotation_30s.csv`, using `--mag-fit-hard-iron --mag-axis-order 0 2 1`
gave these world-heading residuals:

```text
correction rate 0.0 1/s: mean abs 2.087 deg, max 9.179 deg
correction rate 0.5 1/s: mean abs 1.828 deg, max 6.440 deg
correction rate 1.0 1/s: mean abs 1.616 deg, max 4.852 deg
correction rate 2.0 1/s: mean abs 1.252 deg, max 3.068 deg
correction rate 5.0 1/s: mean abs 0.688 deg, max 1.487 deg
```

High correction rates fit this recording better, but they may also follow
magnetometer distortion more aggressively. The rate should remain a tunable
parameter.

## Body-Frame North Diagnostic

As a cross-check, the probe can track a fixed inertial north vector as seen in
the body frame:

```math
\frac{d n_H}{dt} = -\omega_H \times n_H.
```

In implementation, this is equivalent to applying the inverse gyro delta
rotation to the previous body-frame north vector.

The important lesson from the diagnostic work is that magnetometer samples must
not be averaged directly in body coordinates over a long window while the body
is rotating. Either map them into world first, or compare instantaneous
body-frame readings against the gyro-tracked body-frame north/up directions.

## Accelerometer Residual

Given the orientation estimate, predicted stationary specific force is:

```math
\hat a_i = -R_i^T g_W.
```

The measured residual is:

```math
e_i = a_i - \hat a_i.
```

This residual includes both tracker rotational acceleration due to the
head/neck lever arm and any true pivot linear acceleration.

## Rotational Acceleration Around The Neck Model

Angular acceleration is estimated by finite differencing unbiased gyro samples:

```math
\alpha_i =
\frac{\omega_i - \omega_{i-1}}{\Delta t}.
```

For a fixed neck-to-tracker vector `r_NT`, the expected tracker acceleration in
the body frame from rotation about the neck pivot is:

```math
a_{\mathrm{rot},H}
=
\alpha \times r_{NT}
+
\omega \times (\omega \times r_{NT}).
```

The diagnostic pivot acceleration is then:

```math
a_{N,H} = e_i - a_{\mathrm{rot},H}.
```

and in world coordinates:

```math
a_{N,W} = R_i a_{N,H}.
```

## Pivot Position Is Not A Reliable Output

The probe can double-integrate `a_N,W` to produce a diagnostic pivot position.
It uses trapezoidal integration and optional velocity damping:

```math
v_i \approx v_{i-1} + \frac12(a_{i-1}+a_i)\Delta t,
```

```math
p_i \approx p_{i-1} + \frac12(v_{i-1}+v_i)\Delta t.
```

However, this position is not well constrained. In real calibration recordings
we do not know absolute pivot velocity, and small acceleration bias creates
large position drift. A nearly straight segment with low acceleration but
nonzero inferred velocity can dominate the trajectory.

Therefore:

- use pivot acceleration and pivot position as diagnostics,
- do not treat inferred pivot position as real 6-DOF tracking,
- for head/neck visualization, it is often better to drop pivot motion and draw
  the rotating head/neck model about a fixed pivot.

## Eye Midpoint And Look Direction

Once orientation is known, the model can produce useful pose-like quantities
even when pivot translation is suppressed.

With optional pivot position:

```math
p_E(t) = p_N(t) + R(t)r_{NE}.
```

For fixed-pivot visualization:

```math
p_N(t) = 0,
```

so:

```math
p_E(t) = R(t)r_{NE}.
```

The world-space look direction is:

```math
d_W(t) = R(t)d_H.
```

The current default look vector is:

```math
d_H = (0,0,-1).
```

## Geometry Fitting Findings

For clean simulated data with no pivot motion and known orientation, the head
geometry can be fit as a linear least-squares problem:

```math
a_i + R_i^T g_W
\approx
\left(
  [\alpha_i]_\times + [\omega_i]_\times[\omega_i]_\times
\right) r.
```

With the Gamma model:

```math
r = (0,h,-\ell).
```

This physics baseline recovers `h` and `ell` when the trajectory has enough
angular excitation and pivot acceleration is absent or modeled.

With unknown pivot motion, geometry and pivot acceleration are confounded.
Fitting a constant-jerk pivot model can help over short windows, but it fails
when the true trajectory is not close to constant jerk. Short high-excitation
windows are more informative than long slow windows.

For real calibration, the current practical approach is:

- use stationary data for gyro bias,
- use magnetometer calibration and yaw correction for orientation,
- identify high angular-acceleration windows,
- fit `h` and `ell` from those windows,
- treat pivot translation as unobservable unless extra constraints or external
  measurements are added.

## Current Probe Command Shape

Typical orientation and heading diagnostic command:

```bash
cd /Users/jma/dev/OculusRiftDK1/probe
.venv/bin/python probe_head_model.py ../gentle_rotation_30s.csv \
  --stationary-bias ../stationary_30s.csv \
  --mag-fit-hard-iron \
  --mag-axis-order 0 2 1 \
  --mag-correction-rate 2 \
  --csv
```

For visualization, use fixed-pivot animation when the inferred pivot path is
dominated by integration drift.

## Open Questions

- The DK1 magnetometer needs better calibration than a sphere-fit hard-iron
  diagnostic if we want robust yaw correction across environments.
- The exact optical forward convention should remain empirically checked.
- Robust real-data `h` and `ell` fitting likely needs automatic window
  selection based on angular excitation and residual consistency.
- True pivot translation requires external constraints, known motion priors, or
  another sensor source; the IMU alone does not provide enough information.
