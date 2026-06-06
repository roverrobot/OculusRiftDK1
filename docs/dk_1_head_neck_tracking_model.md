# Oculus Rift DK1 Head/Neck Tracking Model, Revised with Eye Direction

## Purpose

This note summarizes a user-space tracking model for the Oculus Rift DK1 headset tracker. The goal is not true 6-DOF positional tracking, but a practical inertial model that estimates:

- headset orientation,
- angular velocity and angular acceleration,
- approximate linear acceleration,
- plausible eye positions from a head/neck kinematic model,
- the world-space looking direction,
- gyro drift using slower accelerometer/magnetometer correction.

The DK1 tracker provides synchronized gyro and accelerometer samples, plus lower-rate magnetometer readings. This makes it feasible to build a CPU-side estimator with a fast inertial layer and a slower correction layer.

The main revision from the earlier model is that the head is not represented only by a single neck-to-tracker vector. We distinguish:

- the effective neck pivot,
- the center of the head,
- the center of the eyes,
- the left and right eye positions,
- the tracker/IMU point,
- the headset-frame looking direction.

This distinction is useful because the accelerometer model depends on the neck-to-tracker vector, while rendering depends on the neck-to-eye vectors and the looking direction.

---

## 1. Coordinate Frames and Points

Use the following coordinate frames:

```text
W = world frame
    +x points right
    +y points up
    -z points forward
    gravity vector g_W = (0, -g, 0), with g ≈ 9.81 m/s²

H = headset/tracker frame
    axes fixed to the headset tracker
    provisional +y points up when the headset is level
    provisional -z points in the looking direction

N = effective neck pivot frame
    origin near the effective center of head rotation
```

Use the following anatomical or device points:

```text
N = effective neck pivot
C = center of head
E = center of eyes
L = left eye
R = right eye
T = tracker / IMU point
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

For quaternion-based code, let `q(t)` represent the same orientation: it rotates headset-frame vectors into world-frame vectors.

---

## 2. Refined Head/Neck Geometry

The earlier model used a single vector from the neck pivot to the tracker. That remains important for accelerometer prediction, but it is not sufficient for rendering eye poses.

We define the following fixed vectors, expressed in headset/head coordinates:

```math
r_{NT} = \text{vector from neck pivot } N \text{ to tracker } T,
```

```math
r_{NC} = \text{vector from neck pivot } N \text{ to head center } C,
```

```math
r_{CE} = \text{vector from head center } C \text{ to eye center } E.
```

Then

```math
r_{NE} = r_{NC} + r_{CE}
```

is the vector from neck pivot to the center of the eyes.

Let the interpupillary distance be

```math
d_{\mathrm{IPD}}.
```

Assuming the headset-frame `x` axis is the left-right axis, the left and right eye offsets from the eye center are

```math
r_{EL} =
\begin{bmatrix}
-\frac12 d_{\mathrm{IPD}} \\
0 \\
0
\end{bmatrix},
\qquad
r_{ER} =
\begin{bmatrix}
\frac12 d_{\mathrm{IPD}} \\
0 \\
0
\end{bmatrix}.
```

Therefore the left and right eye vectors relative to the neck pivot are

```math
r_{NL} = r_{NE} + r_{EL},
```

```math
r_{NR} = r_{NE} + r_{ER}.
```

Let the neck pivot position in world coordinates be

```math
p_N(t).
```

Then the world positions of the tracker, head center, eye center, left eye, and right eye are

```math
p_T(t) = p_N(t) + R(t)r_{NT},
```

```math
p_C(t) = p_N(t) + R(t)r_{NC},
```

```math
p_E(t) = p_N(t) + R(t)r_{NE},
```

```math
p_L(t) = p_N(t) + R(t)r_{NL},
```

```math
p_R(t) = p_N(t) + R(t)r_{NR}.
```

This separates the model into two conceptually different uses:

```text
accelerometer prediction:
    depends on r_NT, the neck-to-tracker vector

rendering / eye-pose prediction:
    depends on r_NE, r_NL, r_NR, and the looking direction
```

---

## 3. Looking Direction

The second key rendering quantity, besides the eye midpoint position, is the world-space looking direction.

Let

```math
d_H
```

be a unit vector in headset coordinates pointing in the intended forward-looking direction. Then the world-space looking direction is

```math
d_W(t) = R(t)d_H.
```

The current provisional default is

```math
d_H =
\begin{bmatrix}
0\\
0\\
-1
\end{bmatrix}.
```

This matches the current probe convention where headset `-z` points out through the eye midpoint. The final tracker-to-display convention should still be confirmed before rendering-facing pose output is treated as calibrated.

A more complete optical calibration could replace `d_H` with a fixed headset-to-optical-frame rotation:

```math
R_{\mathrm{HO}},
```

so that the rendered eye orientation is

```math
R_{\mathrm{eye}}(t)=R(t)R_{\mathrm{HO}}.
```

For the first implementation, using a single forward vector `d_H` is sufficient.

The two key inferred/rendered quantities are therefore:

```math
p_E(t)=p_N(t)+R(t)r_{NE},
```

and

```math
d_W(t)=R(t)d_H.
```

---

## 4. Approximate Default Geometry

The actual values should eventually be fitted or user-configured. For a first implementation, use conservative placeholder values in meters:

```text
neck_to_tracker      r_NT ≈ (0.000, 0.100, -0.160)
neck_to_head_center  r_NC ≈ (0.000, 0.100,  0.000)
head_center_to_eye   r_CE ≈ (0.000, 0.000, -0.160)
IPD                  d_IPD ≈ 0.064
```

These defaults form a letter-Gamma-shaped placeholder model. With the y-up convention, positive `y` is above the neck pivot, and negative `z` is the provisional looking direction. The tracker/IMU is approximated as being at the eye midpoint, so the default has `r_NT ≈ r_NE`. These values should be fitted or user-configured later; they are not anatomical facts.

The important design decision is not the exact numbers, but the separation between:

```text
r_NT        used for tracker acceleration
r_NE        used for eye-center pseudo-position
r_NL, r_NR  used for stereo eye poses
d_H         used for looking direction
```

---

## 5. Angular Velocity and Angular Acceleration

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

For any point fixed to the head/headset with offset vector `r` from the neck pivot, the rotational acceleration in headset coordinates is

```math
a_{\mathrm{rot},H}(r)
=
\alpha_H \times r
+
\omega_H \times (\omega_H \times r).
```

The first term is tangential acceleration, and the second term is centripetal acceleration.

For the tracker/IMU point specifically,

```math
a_{\mathrm{rot},T,H}
=
\alpha_H \times r_{NT}
+
\omega_H \times (\omega_H \times r_{NT}).
```

For the eye center,

```math
a_{\mathrm{rot},E,H}
=
\alpha_H \times r_{NE}
+
\omega_H \times (\omega_H \times r_{NE}).
```

The tracker acceleration is what matters for the accelerometer measurement. The eye-center acceleration is useful later for prediction and pseudo-position.

If the neck pivot itself has world-frame acceleration

```math
a_{N,W}(t) = \ddot p_N(t),
```

then in headset coordinates:

```math
a_{N,H}(t) = R(t)^T a_{N,W}(t).
```

The physical acceleration of the tracker in headset coordinates is therefore

```math
a_{T,H}
=
R(t)^T a_{N,W}
+
\alpha_H \times r_{NT}
+
\omega_H \times (\omega_H \times r_{NT}).
```

---

## 6. Expected Accelerometer Reading

The DK1 dump/probe convention treats the calibrated accelerometer vector as body-frame specific force. With the y-up world convention, a stationary aligned headset measures approximately `(0, +9.8, 0)` because gravity is subtracted from physical acceleration before rotating into the body frame.

With gravity vector

```math
g_W = (0, -g, 0),
```

the ideal DK1/probe specific-force reading at the tracker is

```math
a_{H,\mathrm{specific}}
=
R(t)^T(a_{T,W} - g_W).
```

Using the refined head/neck model:

```math
a_{H,\mathrm{specific}}
=
R(t)^T a_{N,W}
+
\alpha_H \times r_{NT}
+
\omega_H \times(\omega_H \times r_{NT})
-
R(t)^T g_W.
```

So the calibrated accelerometer model is

```math
\tilde a(t)
=
a_{N,H}(t)
+
\alpha_H(t) \times r_{NT}
+
\omega_H(t) \times (\omega_H(t) \times r_{NT})
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
\alpha_H(t) \times r_{NT}
+
\omega_H(t) \times (\omega_H(t) \times r_{NT})
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

The residual

```math
\tilde a(t)
-
\left[
\alpha_H(t) \times r_{NT}
+
\omega_H(t) \times(\omega_H(t)\times r_{NT})
-
R(t)^T g_W
\right]
```

is interpreted as approximate neck/body translational acceleration in headset coordinates.

---

## 7. Expected Gyro Reading

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

Here `q` is assumed to map headset-frame vectors to world-frame vectors, and `\tilde\omega` is assumed to be expressed in the headset/body frame. If `\omega` is instead represented in world coordinates, or if the implementation uses a world-to-headset quaternion, the multiplication order and sign convention must be adjusted.

---

## 8. Gyro Drift Model

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
b_{\omega,0} + K_T(T-T_0) + b_{\mathrm{slow}}(t).
```

The temperature-to-bias relation should be learned empirically; it should not be assumed known.

Important identifiability point:

- In a very short window, gyro bias is confounded with initial angular velocity.
- A linear gyro-bias drift is confounded with angular acceleration.
- Therefore, gyro bias should usually be estimated in a slower filter, not freely fitted inside each 10–20 sample regression window.

---

## 9. Local Sliding-Window Motion Model

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

## 10. Sliding-Window Observation Equations

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
(\alpha_0+\beta\tau)\times r_{NT}
+
\omega(\tau)\times(\omega(\tau)\times r_{NT})
-
R(\tau)^T g_W
+
\epsilon_a.
```

This is the core local estimator.

The term

```math
(\alpha_0+\beta\tau)\times r_{NT}
```

accounts for tangential acceleration caused by changing head angular velocity.

The term

```math
\omega(\tau)\times(\omega(\tau)\times r_{NT})
```

accounts for centripetal acceleration caused by rotation around the neck pivot.

The residual after subtracting rotational acceleration and the gravity-derived specific-force term is interpreted as approximate translational acceleration of the neck/head system.

---

## 11. Eye Pose Model for Rendering

The inertial sensor estimates orientation. The head/neck model converts this orientation into approximate eye positions and a looking direction.

Given the neck pivot world position `p_N(t)`, the left and right eye positions are

```math
p_L(t) = p_N(t) + R(t)r_{NL},
```

```math
p_R(t) = p_N(t) + R(t)r_{NR}.
```

The eye-center position is

```math
p_E(t) = \frac12(p_L(t)+p_R(t)) = p_N(t)+R(t)r_{NE}.
```

The looking direction is

```math
d_W(t)=R(t)d_H.
```

The simplest DK1-style model takes the eye orientations to be the same as the headset orientation:

```math
R_L(t) = R_R(t) = R(t).
```

This is an approximation. Later, a fixed display/optical alignment rotation can be added:

```math
R_L(t)=R(t)R_{HL},
\qquad
R_R(t)=R(t)R_{HR}.
```

This gives a DK1-style pseudo-6DOF pose:

```text
orientation:
    from gyro integration plus accel/mag correction

eye translation:
    from neck-pivot model

looking direction:
    from R(t)d_H

true body translation:
    only approximate, unless external tracking is added
```

If an approximate neck/body translational acceleration estimate is retained, the neck pivot can be propagated as

```math
p_N(t+\Delta t)
\approx
p_N(t)
+
v_N(t)\Delta t
+
\frac12 a_N(t)\Delta t^2.
```

However, this should be treated cautiously because pure inertial position integration drifts rapidly without external reference. In a DK1-only implementation, `p_N(t)` should usually be fixed, externally provided, or only weakly updated, not freely integrated for long durations.

---

## 12. Number of Observations in a 20-Sample Window

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

The additional head geometry parameters

```text
r_NT, r_NC, r_CE, IPD, d_H
```

should not all be estimated freely in the fast 10–20 ms window. They should be treated as fixed parameters or slowly fitted calibration parameters.

A compact grouping of the model parameters is:

```text
Rendered outputs:
    p_E(t)       eye midpoint world position
    d_W(t)       world-space looking direction

Dynamic state:
    R(t) or q(t)
    omega_H(t)
    alpha_H(t)
    p_N(t), v_N(t), a_N(t) if used

Head/neck geometry:
    r_NT
    r_NC
    r_CE
    r_NE = r_NC + r_CE
    d_IPD
    d_H

Sensor calibration:
    C_omega, b_omega
    C_a, b_a
    C_m, b_m
    optional temperature-bias relation

World/environment:
    g_W
    m_W

Fast local fit:
    alpha_0, beta, a_0, j
```

---

## 13. Magnetometer Model

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

Head/neck model:
    fixed or slowly fitted geometry
    compute tracker rotational acceleration, eye poses, and looking direction
```

---

## 14. Recommended Estimator Architecture

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
    - use r_NT to compute rotational acceleration at the tracker
    - subtract gravity and tracker rotational acceleration from accelerometer
    - estimate local translational acceleration and jerk
    - update pseudo-position from head/neck model

Slow correction layer
    - estimate gyro bias
    - estimate accelerometer bias
    - use stationary detection
    - use accelerometer for pitch/roll drift
    - use magnetometer for yaw drift
    - optionally use temperature for empirical bias correction
    - slowly fit head/neck geometry if desired

Render thread, 60–120 Hz or more
    - query latest predicted pose
    - never wait for HID
    - use orientation, looking direction, and left/right eye poses from the head/neck model
```

---

## 15. Pose Prediction

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
p_E(t) = p_N(t) + R(t)r_{NE},
```

the stereo eye positions are

```math
p_L(t) = p_N(t) + R(t)r_{NL},
```

```math
p_R(t) = p_N(t) + R(t)r_{NR},
```

and the predicted looking direction is

```math
d_W(t)=R(t)d_H.
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

## 16. What the Model Can and Cannot Do

This model can provide:

- robust orientation tracking,
- pitch/roll drift correction using gravity,
- yaw drift correction using magnetometer when reliable,
- approximate translational acceleration,
- plausible eye-center and stereo-eye pseudo-positions,
- looking direction from orientation and a headset-frame forward vector,
- improved gravity subtraction during head rotation.

This model cannot provide reliable absolute position by itself. Without external tracking, true translational position remains weakly identifiable and will drift if acceleration is integrated.

Therefore, the output should be described as:

```text
orientation:                 reliable enough for DK1-style tracking
angular velocity:            reliable short term
angular acceleration:        estimated over sliding window
linear acceleration:         approximate residual estimate
eye pseudo-position:         neck/head model approximation
looking direction:           orientation-derived, with fixed headset-frame direction
absolute position:           not reliable without external reference
```

---

## 17. Implementation Notes

The computation is small enough for CPU execution.

A 20-sample window has about 120 gyro/accelerometer scalar observations. The estimator involves small-vector operations, cross products, polynomial fits, quaternion rotation, and small least-squares problems. CPU vectorization or Apple Accelerate/vDSP should be sufficient.

GPU/Metal should be reserved for:

- stereo rendering,
- lens distortion correction,
- possible late orientation-only reprojection.

The tracking estimator should first be implemented on the CPU with careful timestamping, no unnecessary allocation, and nonblocking communication with the render thread.

A useful first code module is:

```c
typedef struct DK1HeadModel {
    DK1Vector3 neck_to_tracker;      // r_NT
    DK1Vector3 neck_to_head_center;  // r_NC
    DK1Vector3 head_center_to_eye;   // r_CE
    double ipd_m;
    DK1Vector3 look_dir_head;        // d_H, unit vector
} DK1HeadModel;
```

If the current implementation does not yet include `look_dir_head`, it can still compute eye positions. The looking direction should be added before rendering-facing pose output is finalized.

This module can provide:

```text
neck_to_eye_center()
left_eye_from_neck()
right_eye_from_neck()
eye_positions_world()
tracker_rotational_accel()
looking_direction_world()
```

The geometry parameters should be adjustable and should not be hard-coded into the estimator.

---

## 18. Summary of Core Equations

Head/neck position model:

```math
r_{NE}=r_{NC}+r_{CE},
```

```math
r_{NL}=r_{NE}+\begin{bmatrix}-d_{\mathrm{IPD}}/2\\0\\0\end{bmatrix},
```

```math
r_{NR}=r_{NE}+\begin{bmatrix}d_{\mathrm{IPD}}/2\\0\\0\end{bmatrix}.
```

Tracker, eye-center, and stereo-eye positions:

```math
p_T(t) = p_N(t) + R(t)r_{NT},
```

```math
p_E(t) = p_N(t) + R(t)r_{NE},
```

```math
p_L(t) = p_N(t) + R(t)r_{NL},
```

```math
p_R(t) = p_N(t) + R(t)r_{NR}.
```

Looking direction:

```math
d_W(t)=R(t)d_H.
```

Gyro model:

```math
y_\omega(t) = C_\omega\omega_H(t) + b_\omega(t) + \eta_\omega(t).
```

Accelerometer model at the tracker:

```math
y_a(t)
=
C_a
\left[
R(t)^T a_{N,W}(t)
+
\alpha_H(t)\times r_{NT}
+
\omega_H(t)\times(\omega_H(t)\times r_{NT})
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
12 fast variables
+ 3 slow gyro-bias variables
+ slowly fitted or user-configured head/neck geometry
+ headset-frame looking direction d_H
```
