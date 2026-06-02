# Oculus Rift DK1 Head/Neck Tracking Model

## Purpose

This note summarizes a user-space tracking model for the Oculus Rift DK1 headset tracker. The goal is not true 6-DOF positional tracking, but a practical inertial model that estimates:

- headset orientation,
- angular velocity and angular acceleration,
- approximate linear acceleration,
- pseudo-position from a simple head/neck pivot model,
- gyro drift using slow accelerometer/magnetometer correction.

The DK1 tracker provides synchronized gyro and accelerometer samples, plus lower-rate magnetometer readings. This makes it feasible to build a CPU-side sliding-window estimator.

---

## 1. Coordinate Frames

Use three coordinate frames:

```text
W = world frame
    z-axis upward
    gravity vector g_W = (0, 0, -g), with g ≈ 9.81 m/s²

H = headset/tracker frame
    axes fixed to the headset tracker

N = neck pivot frame
    origin near an effective head/neck pivot
```

Let

```math
R(t) \in SO(3)
```

be the rotation from headset coordinates to world coordinates. Therefore,

```math
x_W = R(t)x_H,
```

and

```math
x_H = R(t)^T x_W.
```

---

## 2. Head/Neck Geometry

Assume the tracker is located at a fixed vector from an effective neck pivot:

```math
r_H = \begin{bmatrix} r_x \\ r_y \\ r_z \end{bmatrix},
```

where `r_H` is expressed in headset coordinates.

Let the neck pivot position in world coordinates be

```math
p_N(t).
```

Then the tracker position is

```math
p_T(t) = p_N(t) + R(t)r_H.
```

This separates motion into two parts:

1. translation of the neck/body pivot, `p_N(t)`, and
2. tracker motion caused by head rotation around the pivot.

If the neck pivot is assumed stationary, all tracker translation comes from rotation around the neck. If the user leans or moves their torso, `p_N(t)` changes.

---

## 3. Angular Velocity and Angular Acceleration

Let

```math
\omega_H(t)
```

be headset angular velocity expressed in headset coordinates. This is the physical quantity measured by the gyro, up to bias, scale error, and noise.

Let

```math
\alpha_H(t) = \dot{\omega}_H(t)
```

be angular acceleration, also expressed in headset coordinates.

The acceleration of the tracker caused by rotation around the neck pivot is

```math
a_{rot,H}
=
\alpha_H \times r_H
+
\omega_H \times (\omega_H \times r_H).
```

The first term is tangential acceleration; the second is centripetal acceleration.

If the neck pivot itself has world-frame acceleration

```math
a_{N,W} = \ddot{p}_N(t),
```

then in headset coordinates:

```math
a_{N,H} = R(t)^T a_{N,W}.
```

Therefore, the physical tracker acceleration in headset coordinates is

```math
a_{T,H}
=
R(t)^T a_{N,W}
+
\alpha_H \times r_H
+
\omega_H \times (\omega_H \times r_H).
```

---

## 4. Expected Accelerometer Reading

An accelerometer measures **specific force**, not pure physical acceleration. It measures physical acceleration minus gravity, expressed in the sensor frame.

With gravity vector

```math
g_W = (0,0,-g),
```

the ideal accelerometer reading is

```math
f_H
=
R(t)^T(a_{T,W} - g_W).
```

Using the head/neck model:

```math
f_H
=
R(t)^T a_{N,W}
+
\alpha_H \times r_H
+
\omega_H \times(\omega_H \times r_H)
-
R(t)^T g_W.
```

So the calibrated accelerometer model is

```math
\tilde a(t)
=
a_{N,H}(t)
+
\alpha_H(t) \times r_H
+
\omega_H(t) \times (\omega_H(t) \times r_H)
-
R(t)^T g_W
+
\epsilon_a(t).
```

With scale/cross-axis calibration and bias:

```math
y_a(t)
=
C_a
\left[
a_{N,H}(t)
+
\alpha_H(t) \times r_H
+
\omega_H(t) \times (\omega_H(t) \times r_H)
-
R(t)^T g_W
\right]
+
b_a
+
\eta_a(t).
```

Here:

```text
y_a   = raw or partially calibrated accelerometer vector
C_a   = accelerometer scale/cross-axis matrix
b_a   = accelerometer bias
η_a   = accelerometer noise
```

After calibration, we prefer to work with

```math
\tilde a(t) = C_a^{-1}(y_a(t)-b_a).
```

---

## 5. Expected Gyro Reading

The ideal gyro reading is angular velocity in headset coordinates:

```math
\omega_H(t).
```

With scale/cross-axis calibration, bias, and noise:

```math
y_\omega(t)
=
C_\omega \omega_H(t)
+
b_\omega(t)
+
\eta_\omega(t).
```

After calibration:

```math
\tilde\omega(t)
=
C_\omega^{-1}(y_\omega(t)-b_\omega(t)).
```

The gyro is used for short-term orientation integration:

```math
\dot q
=
\frac12 q \otimes
\begin{bmatrix}
0 \\
\tilde\omega
\end{bmatrix}.
```

---

## 6. Gyro Drift Model

Over a short 10–20 ms window, gyro drift can be treated as a constant vector:

```math
y_\omega(t) = C_\omega \omega_H(t) + b_\omega + \eta_\omega(t).
```

Over a longer session, gyro bias should be modeled as slowly varying:

```math
b_{\omega,k+1} = b_{\omega,k} + w_k,
```

where `w_k` is small process noise.

A temperature-dependent empirical model can optionally be added:

```math
b_\omega(T)
=
b_{\omega,0} + K_T(T-T_0) + b_{slow}(t).
```

However, the temperature-to-bias relation should be learned empirically; it should not be assumed known.

Important identifiability point:

- In a very short window, gyro bias is confounded with the initial angular velocity.
- A linear gyro-bias drift is confounded with angular acceleration.
- Therefore, gyro bias should usually be estimated in a slower filter, not freely fitted inside each 10–20 sample regression window.

---

## 7. Local Sliding-Window Motion Model

Use a short window of `N = 10–20` synchronized gyro/accelerometer samples. At 1000 Hz, this corresponds to about 10–20 ms.

Let local time be

```math
\tau = t - t_0.
```

Assume angular acceleration is locally linear in time:

```math
\alpha_H(\tau) = \alpha_0 + \beta \tau.
```

Then angular velocity is locally quadratic:

```math
\omega_H(\tau)
=
\omega_0
+
\alpha_0\tau
+
\frac12\beta\tau^2.
```

Assume neck-pivot translational acceleration, expressed in headset coordinates, is locally linear:

```math
a_{N,H}(\tau) = a_0 + j\tau.
```

The fast local variables are therefore:

```text
a_0      linear acceleration intercept, 3 variables
j        linear jerk, 3 variables
α_0      angular acceleration intercept, 3 variables
β        angular jerk, 3 variables
```

Total fast variables:

```text
3 + 3 + 3 + 3 = 12
```

If constant gyro bias is counted as a local variable, add 3 more variables:

```text
b_ω      gyro drift/bias vector, 3 variables
```

Then the full local count is 15. In practice, it is better to treat `b_ω` as a slow external state.

---

## 8. Sliding-Window Observation Equations

After calibration and bias correction, the gyro model is

```math
\tilde\omega(\tau)
=
\omega_0
+
\alpha_0\tau
+
\frac12\beta\tau^2
+
\epsilon_\omega.
```

The accelerometer model is

```math
\tilde a(\tau)
=
a_0+j\tau
+
(\alpha_0+\beta\tau)\times r_H
+
\omega(\tau)\times(\omega(\tau)\times r_H)
-
R(\tau)^Tg_W
+
\epsilon_a.
```

This is the core local estimator.

The term

```math
(\alpha_0+\beta\tau)\times r_H
```

accounts for tangential acceleration caused by changing head angular velocity.

The term

```math
\omega(\tau)\times(\omega(\tau)\times r_H)
```

accounts for centripetal acceleration caused by rotation around the neck pivot.

The residual after subtracting gravity and rotational acceleration is interpreted as approximate translational acceleration of the neck/head system.

---

## 9. Number of Observations in a 20-Sample Window

Each synchronized gyro/accelerometer sample provides:

```text
gyro:          3 scalar readings
accelerometer: 3 scalar readings
```

So each sample gives 6 scalar observations.

For 20 samples:

```math
20 \times 6 = 120
```

scalar observations.

Compared with the 12 fast variables, or 15 variables if local gyro bias is included, the system is overdetermined in scalar count. However, identifiability still matters because gravity, pivot error, gyro drift, and translational acceleration can imitate each other.

---

## 10. Magnetometer Model

The magnetometer is lower-rate than gyro/accelerometer and should mainly be used as a slow heading/yaw correction.

Let

```math
m_W
```

be the local Earth magnetic field in world coordinates.

The calibrated magnetometer model is

```math
\tilde m(t)
=
R(t)^T m_W
+
\epsilon_m(t).
```

With hard-iron and soft-iron calibration:

```math
y_m(t)
=
C_m R(t)^T m_W + b_m + \eta_m(t).
```

The magnetometer residual is

```math
e_m(t_i)
=
\tilde m_i - R(t_i)^T m_W.
```

The magnetometer can help estimate yaw drift and gyro bias over longer time scales. It should not normally be treated as a high-rate acceleration observation.

A combined objective could include magnetometer residuals:

```math
\min_x
\sum_i \|e_{\omega,i}\|^2_{W_\omega}
+
\sum_i \|e_{a,i}\|^2_{W_a}
+
\sum_{i\in M} \|e_{m,i}\|^2_{W_m},
```

where `M` is the set of unique magnetometer samples in the window.

In practice:

```text
Fast 10–20 ms estimator:
    gyro + accelerometer
    estimate α_0, β, a_0, j

Slower orientation/bias estimator:
    gyro integration
    accelerometer gravity residual
    magnetometer heading residual
    estimate R and b_ω
```

---

## 11. Recommended Estimator Architecture

```text
HID input thread
    - read 62-byte HID input reports
    - unpack bundled gyro/accelerometer samples
    - unpack magnetometer and temperature
    - timestamp samples

Ring buffer
    - store recent calibrated IMU samples
    - avoid blocking render thread

Fast tracking thread, roughly 500–1000 Hz
    - maintain orientation from gyro integration
    - fit local gyro polynomial
    - estimate angular acceleration and angular jerk
    - subtract gravity and rotational acceleration from accelerometer
    - estimate local translational acceleration and jerk
    - update pseudo-position from neck model

Slow correction layer
    - estimate gyro bias
    - estimate accelerometer bias
    - use stationary detection
    - use accelerometer for pitch/roll drift
    - use magnetometer for yaw drift
    - optionally use temperature for empirical bias correction

Render thread, 60–120 Hz or more
    - query latest predicted pose
    - never wait for HID
    - use orientation and neck-model pseudo-position
```

---

## 12. Pose Prediction

For rendering, predict pose at display time rather than simply using the latest sample.

Given current orientation `q`, angular velocity `ω`, and angular acceleration `α`, predict forward by `Δt`:

```math
\omega(t+\Delta t)
\approx
\omega(t) + \alpha(t)\Delta t.
```

A simple orientation prediction can integrate the local angular-velocity model over the prediction interval.

The pseudo-position from the neck model is

```math
p_T(t) = p_N(t) + R(t)r_H.
```

If approximate translational acceleration is retained, one can also use

```math
p_N(t+\Delta t)
\approx
p_N(t)
+
v_N(t)\Delta t
+
\frac12 a_N(t)\Delta t^2.
```

But this should be treated cautiously because inertial position integration drifts rapidly without external reference.

---

## 13. What the Model Can and Cannot Do

This model can provide:

- robust orientation tracking,
- pitch/roll drift correction using gravity,
- yaw drift correction using magnetometer when reliable,
- approximate translational acceleration,
- plausible neck-model pseudo-position,
- improved gravity subtraction during head rotation.

This model cannot provide reliable absolute position by itself. Without external tracking, true translational position remains weakly identifiable and will drift if acceleration is integrated.

Therefore, the output should be described as:

```text
orientation:                 reliable enough for DK1-style tracking
angular velocity:            reliable short term
angular acceleration:        estimated over sliding window
linear acceleration:         approximate residual estimate
pseudo-position:             neck-model approximation
absolute position:           not reliable without external reference
```

---

## 14. Implementation Notes

The computation is small enough for CPU execution.

A 20-sample window has about 120 gyro/accelerometer scalar observations. The estimator involves small-vector operations, cross products, polynomial fits, and small least-squares problems. CPU vectorization or Apple Accelerate/vDSP should be sufficient.

GPU/Metal should be reserved for:

- stereo rendering,
- lens distortion correction,
- possible late orientation-only reprojection.

The tracking estimator should first be implemented on the CPU with careful timestamping, no unnecessary allocation, and nonblocking communication with the render thread.

---

## 15. Summary of Core Equations

Head/neck position model:

```math
p_T(t) = p_N(t) + R(t)r_H.
```

Gyro model:

```math
y_\omega(t) = C_\omega\omega_H(t) + b_\omega(t) + \eta_\omega(t).
```

Accelerometer model:

```math
y_a(t)
=
C_a
\left[
R(t)^T a_{N,W}(t)
+
\alpha_H(t)\times r_H
+
\omega_H(t)\times(\omega_H(t)\times r_H)
-
R(t)^T g_W
\right]
+
b_a
+
\eta_a(t).
```

Local angular model:

```math
\alpha_H(\tau)=\alpha_0+\beta\tau,
```

```math
\omega_H(\tau)=\omega_0+\alpha_0\tau+\frac12\beta\tau^2.
```

Local translational acceleration model:

```math
a_{N,H}(\tau)=a_0+j\tau.
```

Magnetometer model:

```math
y_m(t)=C_mR(t)^Tm_W+b_m+\eta_m(t).
```

Fast local variable count:

```text
12 = 3(a_0) + 3(j) + 3(α_0) + 3(β)
```

Including constant local gyro bias:

```text
15 = 12 + 3(b_ω)
```

Recommended practical split:

```text
12 fast variables + 3 slow gyro-bias variables
```

