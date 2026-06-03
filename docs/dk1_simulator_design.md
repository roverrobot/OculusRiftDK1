# DK1 Head/Neck Geometry Simulator Design

## Purpose

This document locks down the design of a simulator for generating synthetic Oculus Rift DK1 tracker data. The simulator is intended for offline training and testing of head/neck geometry estimators.

The main calibration target is the geometry of a simple letter-Gamma-shaped head/neck model. The synthetic data should include smooth head rotations, smooth neck-pivot motion, sensor drift, and white noise. The output is a CSV table that resembles DK1 sensor readings.

The simulator is not intended to be a real-time tracker. It is a data generator for experiments.

---

## 1. Coordinate Convention

Use the DK1-oriented convention adopted in the current probe work:

```text
World frame W:
    +y is up
    gravity is g_W = (0, -g, 0)

Headset/body frame H:
    fixed to the headset/tracker

Orientation:
    R(t) maps headset-frame vectors to world-frame vectors
    q(t) is the equivalent quaternion
```

Gravity is

```math
g_W =
\begin{bmatrix}
0\\
-g\\
0
\end{bmatrix},
\qquad
g = 9.80665 \ \mathrm{m/s^2}.
```

The measured accelerometer specific force is approximately `+y` when the headset is stationary and aligned with the world frame.

---

## 2. Letter-Gamma-Shaped Head/Neck Model

Use a simplified letter-Gamma-shaped model.

The neck is treated as an effective vertical rotation axis. The tracker/IMU is assumed to be located at the midpoint of the two eyes. Thus the tracker point and eye-midpoint point are the same point.

Define:

```text
N = effective neck pivot / point on neck rotation axis
E = midpoint of the two eyes
T = tracker / IMU point
```

For this model,

```math
T = E.
```

Let

```math
u_H =
\begin{bmatrix}
0\\
1\\
0
\end{bmatrix}
```

be the headset-frame upward direction, and let

```math
d_H =
\begin{bmatrix}
0\\
0\\
-1
\end{bmatrix}
```

be the provisional headset-frame looking direction.

The neck-to-eye/tracker vector is

```math
r = h u_H + \ell d_H,
```

where:

```text
h   = vertical distance from neck pivot to eye/tracker midpoint
ell = forward distance from neck axis to eye/tracker midpoint
```

With the default axes above,

```math
r =
\begin{bmatrix}
0\\
h\\
-\ell
\end{bmatrix}.
```

The synthetic geometry parameters to sample are therefore at least:

```text
h
ell
```

Optionally, a small lateral offset can be added later:

```math
r = s e_x + h u_H + \ell d_H,
```

where `s` is a side offset. For the first simulator, set `s = 0`.

---

## 3. Eye Positions and Looking Direction

Since the tracker is at the eye midpoint,

```math
p_E(t) = p_T(t) = p_N(t) + R(t)r.
```

The world-frame looking direction is

```math
d_W(t)=R(t)d_H.
```

If interpupillary distance is included, with `d_IPD` in meters,

```math
r_L = r +
\begin{bmatrix}
-d_{\mathrm{IPD}}/2\\
0\\
0
\end{bmatrix},
\qquad
r_R = r +
\begin{bmatrix}
d_{\mathrm{IPD}}/2\\
0\\
0
\end{bmatrix}.
```

For geometry training of `h` and `ell`, IPD is not essential. It can be fixed or omitted.

---

## 4. Smooth Trajectory Space

Each simulation run lasts `T` seconds.

The simulator generates smooth trajectories using a finite basis. For a scalar trajectory component `x(t)`, use

```math
x(t)
=
c_0
+
c_1 t
+
c_2 t^2
+
c_3 t^3
+
\sum_{k=1}^{3}
\left[
a_k \sin(k\pi t/T)
+
b_k \cos(k\pi t/T)
\right].
```

Equivalently, the trajectory space is spanned by

```text
1, t, t^2, t^3,
sin(k*pi*t/T), cos(k*pi*t/T), k = 1,2,3.
```

The proposed `k = 0,1,2,3` is equivalent if we remember that `sin(0)=0` and `cos(0)=1`; the `k=0` cosine term is just the constant basis.

This gives 10 scalar coefficients per trajectory component.

### 4.1 Head Orientation Trajectories

Define smooth yaw, pitch, and roll trajectories:

```math
\psi(t)    = \mathrm{yaw}(t),
```

```math
\theta(t)  = \mathrm{pitch}(t),
```

```math
\phi(t)    = \mathrm{roll}(t).
```

Each is a linear combination of the basis functions above.

Then construct the orientation matrix

```math
R(t)=R_y(\psi(t))R_x(\theta(t))R_z(\phi(t)),
```

or another fixed Euler order chosen by the implementation. The Euler order must be documented and kept consistent.

The most important requirement is consistency: `R(t)`, gyro, angular acceleration, gravity projection, and accelerometer readings must all use the same convention.

### 4.2 Neck Pivot Translation

The neck pivot position in world coordinates is

```math
p_N(t)=
\begin{bmatrix}
p_{N,x}(t)\\
p_{N,y}(t)\\
p_{N,z}(t)
\end{bmatrix}.
```

Each component is also a linear combination of the same smooth basis functions.

The neck pivot velocity and acceleration are obtained by differentiation:

```math
v_N(t)=\dot p_N(t),
```

```math
a_N(t)=\ddot p_N(t).
```

The headset-frame neck-pivot acceleration is

```math
a_{N,H}(t)=R(t)^T a_N(t).
```

---

## 5. Derivatives

The polynomial derivatives are

```math
\frac{d}{dt}(t)=1,
\qquad
\frac{d}{dt}(t^2)=2t,
\qquad
\frac{d}{dt}(t^3)=3t^2,
```

and

```math
\frac{d}{dt}\sin(k\pi t/T)
=
(k\pi/T)\cos(k\pi t/T),
```

```math
\frac{d}{dt}\cos(k\pi t/T)
=
-(k\pi/T)\sin(k\pi t/T).
```

Second derivatives are

```math
\frac{d^2}{dt^2}(t^2)=2,
\qquad
\frac{d^2}{dt^2}(t^3)=6t,
```

```math
\frac{d^2}{dt^2}\sin(k\pi t/T)
=
-(k\pi/T)^2\sin(k\pi t/T),
```

```math
\frac{d^2}{dt^2}\cos(k\pi t/T)
=
-(k\pi/T)^2\cos(k\pi t/T).
```

For angular velocity, the recommended robust implementation is:

```math
[\omega_H(t)]_\times = R(t)^T \dot R(t).
```

Here `[omega]_x` is the skew-symmetric matrix

```math
[\omega]_\times =
\begin{bmatrix}
0 & -\omega_z & \omega_y\\
\omega_z & 0 & -\omega_x\\
-\omega_y & \omega_x & 0
\end{bmatrix}.
```

Thus

```math
\omega_H =
\begin{bmatrix}
[\omega]_{\times,32}\\
[\omega]_{\times,13}\\
[\omega]_{\times,21}
\end{bmatrix}.
```

Angular acceleration in the headset frame is

```math
\alpha_H(t)=\dot\omega_H(t).
```

This can be computed analytically if desired, but a high-quality finite difference on the smooth generated trajectory is sufficient for the first simulator.

---

## 6. Ideal Sensor Signals

### 6.1 Gyro

The ideal gyro is body-frame angular velocity:

```math
\omega_H^{\mathrm{ideal}}(t)=\omega_H(t).
```

### 6.2 Accelerometer

The rotational acceleration of the tracker/eye midpoint in headset coordinates is

```math
a_{\mathrm{rot},H}(t)
=
\alpha_H(t)\times r
+
\omega_H(t)\times(\omega_H(t)\times r).
```

The gravitational acceleration in headset coordinates is

```math
g_H(t)=R(t)^Tg_W.
```

The ideal accelerometer specific force is

```math
a_H^{\mathrm{ideal}}(t)
=
a_{N,H}(t)
+
\alpha_H(t)\times r
+
\omega_H(t)\times(\omega_H(t)\times r)
-
R(t)^Tg_W.
```

This is the core simulator equation.

### 6.3 Magnetometer

The magnetometer is simulated at a lower rate, approximately 220 Hz.

Let the world-frame magnetic field be a fixed vector

```math
m_W.
```

The ideal magnetometer reading is

```math
m_H^{\mathrm{ideal}}(t)=R(t)^T m_W.
```

A simple default is to choose a normalized field vector, for example

```math
m_W =
\begin{bmatrix}
0.2\\
0.0\\
0.5
\end{bmatrix},
```

then normalize it or scale it to a plausible magnitude. The absolute magnitude is less important than the directional information for initial experiments.

---

## 7. Sensor Drift and White Noise

The measured sensors include white noise and drift.

### 7.1 Gyro Measurement

The measured gyro is

```math
y_{\omega,k}
=
\omega_{H,k}
+
b_{\omega,k}
+
\eta_{\omega,k}.
```

The white noise is

```math
\eta_{\omega,k}\sim N(0,\sigma_\omega^2 I_3).
```

The gyro bias/drift is

```math
b_{\omega,k}
=
b_{\omega,0}
+
b_{\omega,k}^{\mathrm{rw}},
```

with initial constant bias

```math
b_{\omega,0}\sim N(0,\sigma_{\omega,b0}^2I_3),
```

and random-walk drift

```math
b_{\omega,k+1}^{\mathrm{rw}}
=
b_{\omega,k}^{\mathrm{rw}}
+
\sigma_{\omega,\mathrm{rw}}\sqrt{\Delta t}\xi_{\omega,k},
\qquad
\xi_{\omega,k}\sim N(0,I_3).
```

### 7.2 Accelerometer Measurement

The measured accelerometer is

```math
y_{a,k}
=
a_{H,k}^{\mathrm{ideal}}
+
b_{a,k}
+
\eta_{a,k}.
```

White noise:

```math
\eta_{a,k}\sim N(0,\sigma_a^2I_3).
```

Bias/drift:

```math
b_{a,k}
=
b_{a,0}
+
b_{a,k}^{\mathrm{rw}},
```

with

```math
b_{a,0}\sim N(0,\sigma_{a,b0}^2I_3),
```

and

```math
b_{a,k+1}^{\mathrm{rw}}
=
b_{a,k}^{\mathrm{rw}}
+
\sigma_{a,\mathrm{rw}}\sqrt{\Delta t}\xi_{a,k},
\qquad
\xi_{a,k}\sim N(0,I_3).
```

### 7.3 Magnetometer Measurement

At magnetometer sample times,

```math
y_{m,k}
=
m_{H,k}^{\mathrm{ideal}}
+
b_{m,k}
+
\eta_{m,k}.
```

White noise:

```math
\eta_{m,k}\sim N(0,\sigma_m^2I_3).
```

Optional bias/drift:

```math
b_{m,k}
=
b_{m,0}
+
b_{m,k}^{\mathrm{rw}}.
```

For the first simulator, magnetometer drift can be disabled by default.

---

## 8. Sampling Rates

Generate the main IMU stream at 1000 Hz:

```text
gyro:          1000 Hz
accelerometer: 1000 Hz
```

Generate magnetometer readings at approximately 220 Hz.

Since 1000/220 is not an integer, use separate timestamp grids:

```math
t^{\mathrm{imu}}_n = n/1000,
```

```math
t^{\mathrm{mag}}_j = j/220.
```

For CSV output with one row per IMU sample, use the most recent magnetometer value in every row and include a flag:

```text
mag_valid = 1 if a new magnetometer sample occurred on this IMU row, else 0
```

---

## 9. Simulation Parameters

For each run, sample or specify the following parameters.

### Geometry parameters

```text
h
ell
optional lateral offset s
optional IPD
```

### Trajectory coefficients

For yaw, pitch, roll:

```text
coeff_yaw[10]
coeff_pitch[10]
coeff_roll[10]
```

For neck pivot translation:

```text
coeff_px[10]
coeff_py[10]
coeff_pz[10]
```

### Sensor nuisance parameters

```text
gyro white noise level
accel white noise level
mag white noise level

gyro initial bias
accel initial bias
mag initial bias

gyro random-walk drift level
accel random-walk drift level
mag random-walk drift level
```

### Global simulation settings

```text
T = duration per run, seconds
M = number of runs
imu_rate_hz = 1000
mag_rate_hz = 220
random seed
```

---

## 10. Output Dataset

For `M` runs of duration `T` seconds at 1000 Hz, the number of IMU rows is approximately

```math
N_{\mathrm{rows}} = M \times T \times 1000.
```

Strictly, this is the number of IMU time rows. The total number of independent simulation runs is `M`; therefore the CSV should include a `run_id` column so sequences can be grouped.

For example, if `T = 10` seconds and `M = 1000`, then

```math
N_{\mathrm{rows}} = 10{,}000{,}000.
```

That is large but manageable if written carefully. For early experiments, use smaller `M`.

### 10.1 CSV Columns

The output CSV should include at least:

```text
run_id
sample_index
time_s

accel_x
accel_y
accel_z

gyro_x
gyro_y
gyro_z

mag_x
mag_y
mag_z
mag_valid

temp_c
```

For supervised training, include the true target geometry on every row:

```text
h
ell
```

Optionally include simulator internals for debugging:

```text
true_qw
true_qx
true_qy
true_qz

true_omega_x
true_omega_y
true_omega_z

true_alpha_x
true_alpha_y
true_alpha_z

true_pivot_x
true_pivot_y
true_pivot_z

true_pivot_accel_x
true_pivot_accel_y
true_pivot_accel_z

gyro_bias_x
gyro_bias_y
gyro_bias_z

accel_bias_x
accel_bias_y
accel_bias_z
```

For neural-network training, it may be better to save a compact format such as `.npz` in addition to CSV, because a large CSV may become slow. CSV is useful for inspection and interoperability.


### 10.2 Parameter Metadata Output

The sensor-reading CSV alone is not sufficient for supervised training or debugging. For each of the `M` simulation runs, the simulator must also record the exact parameter values used to generate that run.

There are two acceptable designs:

```text
Recommended:
    sensor_readings.csv      one row per IMU sample
    run_parameters.csv       one row per simulation run

Acceptable for small experiments:
    one combined CSV with parameter columns repeated on every sensor row
```

The recommended two-file design is preferable because the trajectory coefficients, drift parameters, and noise levels are run-level quantities. Repeating them on every 1000 Hz sensor row makes the sensor CSV much larger than necessary.

The two files are joined by:

```text
run_id
```

#### 10.2.1 `sensor_readings.csv`

This file contains one row per IMU sample. It should include the measured or simulated sensor stream and only a small number of target columns if convenient.

Required columns:

```text
run_id
sample_index
time_s

accel_x
accel_y
accel_z

gyro_x
gyro_y
gyro_z

mag_x
mag_y
mag_z
mag_valid

temp_c
```

Useful target columns that may be repeated in this file for convenience:

```text
h
ell
```

For large datasets, even `h` and `ell` can be omitted from `sensor_readings.csv` and read only from `run_parameters.csv`.

Optional debugging columns:

```text
true_qw
true_qx
true_qy
true_qz

true_omega_x
true_omega_y
true_omega_z

true_alpha_x
true_alpha_y
true_alpha_z

true_pivot_x
true_pivot_y
true_pivot_z

true_pivot_accel_x
true_pivot_accel_y
true_pivot_accel_z

gyro_bias_x
gyro_bias_y
gyro_bias_z

accel_bias_x
accel_bias_y
accel_bias_z

mag_bias_x
mag_bias_y
mag_bias_z
```

These debugging columns are not required for neural-network input, but they are useful for validating the simulator and developing physics-based baselines.

#### 10.2.2 `run_parameters.csv`

This file contains one row per simulation run. It should include every random or specified parameter used to generate that run.

Required identifying columns:

```text
run_id
seed
T
imu_rate_hz
mag_rate_hz
```

Geometry parameters:

```text
h
ell
s
ipd_m

up_x
up_y
up_z
look_x
look_y
look_z

r_x
r_y
r_z
```

Here `s` is optional lateral offset. For the first Γ-shaped model, set `s = 0`. The vector `r = (r_x,r_y,r_z)` is the resulting neck-to-eye/tracker vector.

Sensor noise and drift parameters:

```text
gyro_noise_std
accel_noise_std
mag_noise_std

gyro_bias0_x
gyro_bias0_y
gyro_bias0_z

accel_bias0_x
accel_bias0_y
accel_bias0_z

mag_bias0_x
mag_bias0_y
mag_bias0_z

gyro_bias_rw_std
accel_bias_rw_std
mag_bias_rw_std
```

Temperature parameters, if temperature drift is enabled:

```text
temp0_c
temp_delta_c
temp_tau_s
temp_noise_std

gyro_temp_coeff_x
gyro_temp_coeff_y
gyro_temp_coeff_z

accel_temp_coeff_x
accel_temp_coeff_y
accel_temp_coeff_z
```

Trajectory coefficient columns should be explicit and machine-readable. For example, use the following naming convention:

```text
yaw_c0
yaw_c1
yaw_c2
yaw_c3
yaw_sin1
yaw_cos1
yaw_sin2
yaw_cos2
yaw_sin3
yaw_cos3

pitch_c0
pitch_c1
pitch_c2
pitch_c3
pitch_sin1
pitch_cos1
pitch_sin2
pitch_cos2
pitch_sin3
pitch_cos3

roll_c0
roll_c1
roll_c2
roll_c3
roll_sin1
roll_cos1
roll_sin2
roll_cos2
roll_sin3
roll_cos3

pivot_x_c0
pivot_x_c1
pivot_x_c2
pivot_x_c3
pivot_x_sin1
pivot_x_cos1
pivot_x_sin2
pivot_x_cos2
pivot_x_sin3
pivot_x_cos3

pivot_y_c0
pivot_y_c1
pivot_y_c2
pivot_y_c3
pivot_y_sin1
pivot_y_cos1
pivot_y_sin2
pivot_y_cos2
pivot_y_sin3
pivot_y_cos3

pivot_z_c0
pivot_z_c1
pivot_z_c2
pivot_z_c3
pivot_z_sin1
pivot_z_cos1
pivot_z_sin2
pivot_z_cos2
pivot_z_sin3
pivot_z_cos3
```

The coefficient convention must match the trajectory definition:

```math
x(t)
=
c_0+c_1t+c_2t^2+c_3t^3
+
\sum_{k=1}^{3}
\left[
\mathrm{sin}_k\sin(k\pi t/T)
+
\mathrm{cos}_k\cos(k\pi t/T)
\right].
```

#### 10.2.3 Combined CSV Option

For small experiments, the simulator may instead produce a single combined CSV. In that case, include all `run_parameters.csv` columns in every corresponding sensor row.

This is simple, but inefficient:

```text
combined.csv:
    run_id, sample_index, time_s, sensor columns, h, ell, coefficients, noise parameters, ...
```

The combined design is acceptable for quick inspection or small `M`, but the two-file design should be used for training-scale datasets.

#### 10.2.4 NPZ/HDF5 Option for Training

For neural-network training, a compact array format is preferable. A recommended `.npz` layout is:

```text
X                 shape (M, N, C), sensor inputs
Y                 shape (M, 2), targets [h, ell]
run_id            shape (M,)
time_s            shape (N,)
feature_names     shape (C,)
parameter_table   optional structured array or JSON sidecar
```

The CSV files remain useful for inspection and interoperability, while `.npz` is better for fast training.

---

## 11. Does the Design Make Sense?

Yes. The design is coherent.

The key points are:

```text
1. The Γ-shaped model gives a small number of geometry targets.
2. Smooth basis trajectories produce realistic, differentiable motion.
3. The same trajectory defines orientation, gyro, angular acceleration, gravity projection, and accelerometer readings.
4. Pivot movement is explicitly included rather than ignored.
5. Drift and white noise make the training problem realistic.
6. 1000 Hz IMU and 220 Hz magnetometer rates match the intended DK1-like data stream.
7. M runs of T seconds produce a supervised dataset for geometry estimation.
```

The only adjustment I recommend is scaling the trigonometric basis by `T` as `sin(k*pi*t/T)` and `cos(k*pi*t/T)`. Without the `/T`, the trajectory basis depends awkwardly on the units and duration of the run. Using `t/T` or `pi*t/T` makes the coefficients easier to sample consistently across different `T`.

---

## 12. Suggested First Implementation

Start with:

```text
M = 100
T = 5 seconds
imu_rate_hz = 1000
mag_rate_hz = 220

geometry:
    h   sampled from [0.07, 0.18]
    ell sampled from [0.04, 0.16]

trajectory:
    yaw/pitch/roll basis coefficients sampled with amplitude limits
    pivot translation basis coefficients sampled with small displacement limits

noise:
    use stationary DK1 data to estimate gyro and accel white-noise levels

drift:
    start with small constant bias
    then add random-walk drift

output:
    CSV for inspection
    NPZ for training
```

Then train a first network to recover `(h, ell)` from synthetic sequences. Compare against a physics-based least-squares baseline later.
