# Oculus Rift DK1 Distortion Mesh Algorithm

This note summarizes the mathematical model for generating the Oculus Rift DK1 lens-distortion mesh, following the Oculus SDK 0.4.4 mesh-based approach.

The emphasis here is on the mathematics, not on SDK code structure.

Primary sources:

```text
https://raw.githubusercontent.com/federico-mammano/Oculus-SDK-0.4.4-beta-OpenGL-Demo/master/OculusSDK/LibOVR/Src/OVR_Stereo.cpp
https://raw.githubusercontent.com/federico-mammano/Oculus-SDK-0.4.4-beta-OpenGL-Demo/master/OculusSDK/LibOVR/Src/Util/Util_Render_Stereo.cpp
```

---

## 1. Basic idea

For each eye, the SDK 0.4.4 distortion mesh is generated from a regular grid in **render-target source NDC**.

Each grid point is first mapped into tangent-eye-angle space, then inverse-distorted to find its final position on the physical screen.

Thus the mesh positions are generally not a regular grid in distorted screen space.

Each mesh vertex stores:

1. its position on the screen,
2. three corresponding tangent-eye-angle coordinates for the rendered eye image:
   - one for red,
   - one for green,
   - one for blue.
3. a vignette/shade value, and
4. a timewarp interpolation value.

The reason for three sampling coordinates is chromatic aberration correction.

Conceptually,

\[
\text{regular grid coordinate} = \text{render-target source coordinate},
\]

while

\[
\text{mesh vertex position} = \text{inverse-distorted screen position}.
\]

At rendering time, the shader maps the per-channel tangent-eye-angle coordinates through an eye-to-source scale/offset, samples the rendered eye texture separately for the three color channels, applies the vignette/shade, and recombines them.

---

## 2. DK1 physical parameters

The DK1 display parameters are approximately:

\[
\begin{aligned}
W_s &= 0.1498 \text{ m},\\
H_s &= 0.0936 \text{ m},\\
G_s &= 0,\\
L_s &= 0.0635 \text{ m},\\
C_t &= 0.0468 \text{ m}.
\end{aligned}
\]

where:

- \(W_s\) is the physical screen width,
- \(H_s\) is the physical screen height,
- \(G_s\) is the screen gap,
- \(L_s\) is the lens separation,
- \(C_t\) is the vertical distance from the top of the screen to the lens center.

The pixel resolution is:

\[
1280 \times 800.
\]

Each eye uses approximately half the screen:

\[
640 \times 800.
\]

---

## 3. Lens centers

Use per-eye normalized device coordinates, where each eye viewport has:

\[
x \in [-1,1], \qquad y \in [-1,1].
\]

The visible width of one eye is

\[
W_e = \frac{1}{2}(W_s - G_s).
\]

The horizontal lens center, measured from the left edge of one eye's visible region, is

\[
C_x^{\text{meters}} = \frac{1}{2}(W_s - L_s).
\]

Thus the per-eye normalized lens center is

\[
c_x = 2\frac{C_x^{\text{meters}}}{W_e} - 1,
\]

and

\[
c_y = 2\frac{C_t}{H_s} - 1.
\]

For DK1,

\[
W_e = 0.0749,
\]

and

\[
C_x^{\text{meters}} = 0.04315.
\]

Therefore,

\[
c_x \approx 0.152203,
\qquad
c_y = 0.
\]

The lens centers are therefore:

\[
c_L = (0.152203, 0),
\]

for the left eye, and

\[
c_R = (-0.152203, 0),
\]

for the right eye.

In per-eye normalized texture coordinates, these correspond to:

\[
c_L^{uv} = (0.5761015, 0.5),
\]

and

\[
c_R^{uv} = (0.4238985, 0.5).
\]

In full-screen pixel coordinates for a \(1280 \times 800\) screen, the physical lens centers are approximately:

\[
(368.76, 400)
\]

and

\[
(911.24, 400).
\]

---

## 4. Eye cups A/B/C

The DK1 has three physical eye-cup sets:

- A: tallest cups,
- B: middle cups,
- C: shortest cups.

They affect the physical viewing experience, especially for users with different eyesight.

However, in the later SDK DK1 distortion path, A/B/C are treated as the same DK1 distortion family. They do **not** select three separate distortion coefficient tables.

Thus, for the mathematical mesh generation model here, the eye cup choice is stored as a profile parameter but does not directly change the distortion curve.

The mesh changes mainly through:

\[
\text{eye relief},
\]

not directly through A/B/C.

---

## 5. Eye relief and dial positions

The 0.4.4 profile path represents DK1 eye relief with an 11-position dial value. Hardware/profile data may also supply per-eye maximum eye-to-plate distances, so the final eye-relief values can still be per-eye even when the dial offset is shared.

Let the dial index be

\[
d \in \{0,1,\ldots,10\}.
\]

The SDK ultimately feeds a physical eye-relief distance into `GenerateLensConfigFromEyeRelief`.

For the tuned DK1 anchor range used by this note, a convenient dial-to-relief model is:

\[
e(d) = e_0 + (d - 5) \cdot 0.001,
\]

where

\[
e_0 = 0.012760465 \text{ m}.
\]

Thus,

\[
e(d) = 0.012760465 + (d - 5)10^{-3}.
\]

The 11 eye-relief values are:

\[
\begin{array}{c|c}
d & e(d) \text{ in meters}\\
\hline
0 & 0.007760465\\
1 & 0.008760465\\
2 & 0.009760465\\
3 & 0.010760465\\
4 & 0.011760465\\
5 & 0.012760465\\
6 & 0.013760465\\
7 & 0.014760465\\
8 & 0.015760465\\
9 & 0.016760465\\
10 & 0.017760465
\end{array}
\]

In general, mesh generation should use the final relief value for each eye:

\[
e_L,
\qquad
e_R.
\]

The 0.4.4 profile path can also compute relief from a maximum eye-to-plate distance and then subtract \((10-d)\cdot1\text{ mm}\). The important input to the distortion generator is the resulting eye-relief distance, not the dial number itself.

---

## 6. Radial distortion curve

In the SDK, the radial scale \(D(r^2)\) maps from a distorted screen-side tangent-eye-angle coordinate to an undistorted source tangent-eye-angle coordinate:

\[
q_t = q_d D(\|q_d\|^2),
\]

where:

- \(q_d \in \mathbb{R}^2\) is the distorted screen-side coordinate after applying lens center and `TanEyeAngleScale`,
- \(q_t \in \mathbb{R}^2\) is the undistorted tangent-eye-angle coordinate used to sample the rendered eye image,
- \(D(r^2)\) is the radial distortion scale,
- \(r = \|q_d\|\).

Equivalently, in scalar radius form,

\[
\rho_t = \rho_d D(\rho_d^2).
\]

Mesh placement uses the inverse of this function: given a source tangent-eye-angle coordinate \(q_t\), find the distorted screen-side coordinate \(q_d\).

The later SDK DK1 path does **not** use a single 10th-degree polynomial. Instead, it uses an 11-value table that defines a **piecewise cubic Catmull-Rom spline** in the variable \(r^2\).

---

## 7. DK1 distortion anchor tables

There are three eye-relief anchor curves.

### 7.1 Minimum eye relief

\[
e_{\min} = 0.007760465.
\]

\[
R_{\max} = \sqrt{1.8}.
\]

\[
K^{\min} =
\begin{bmatrix}
1.0000,&
1.06505,&
1.14725,&
1.2705,&
1.48,&
1.87,&
2.534,&
3.6,&
5.1,&
7.4,&
11.0
\end{bmatrix}.
\]

### 7.2 Middle eye relief

\[
e_{\mathrm{mid}} = 0.012760465.
\]

\[
R_{\max} = 1.
\]

\[
K^{\mathrm{mid}} =
\begin{bmatrix}
1.0,&
1.032407264,&
1.07160462,&
1.11998388,&
1.1808606,&
1.2590494,&
1.361915,&
1.5014339,&
1.6986004,&
1.9940577,&
2.4783147
\end{bmatrix}.
\]

### 7.3 Maximum eye relief

\[
e_{\max} = 0.017760465.
\]

\[
R_{\max} = 1.
\]

\[
K^{\max} =
\begin{bmatrix}
1.0102,&
1.0371,&
1.0831,&
1.1353,&
1.2,&
1.2851,&
1.3979,&
1.56,&
1.8,&
2.25,&
3.0
\end{bmatrix}.
\]

---

## 8. Interpolating the distortion table

For an intermediate eye relief \(e\), interpolate between the neighboring anchor tables.

If

\[
e_{\min} \le e \le e_{\mathrm{mid}},
\]

define

\[
t = \frac{e - e_{\min}}{e_{\mathrm{mid}} - e_{\min}}.
\]

The SDK first chooses lower and upper anchor curves and computes \(t\). If the requested eye relief is outside the sampled range, it clamps to the nearest anchor instead of extrapolating.

The maximum modeled radius is linearly interpolated:

\[
R_{\max}^{\mathrm{interp}}
=
(1-t)R_{\max}^{\min}
+
tR_{\max}^{\mathrm{mid}}.
\]

The first control value \(K_0\), which controls the initial slope, is directly interpolated:

\[
K^{\mathrm{interp}}_0
=
(1-t)K^{\min}_0 + tK^{\mathrm{mid}}_0.
\]

The remaining control values are not directly interpolated entry by entry. Instead, the SDK samples both neighboring distortion curves on the new shared radius grid:

\[
r_i^2 = \frac{i}{10}\left(R_{\max}^{\mathrm{interp}}\right)^2,
\qquad i=1,\ldots,10.
\]

Then:

\[
K^{\mathrm{interp}}_i
=
(1-t)D_{\min}(r_i^2) + tD_{\mathrm{mid}}(r_i^2),
\qquad i=1,\ldots,10.
\]

If

\[
e_{\mathrm{mid}} \le e \le e_{\max},
\]

define

\[
t = \frac{e - e_{\mathrm{mid}}}{e_{\max} - e_{\mathrm{mid}}},
\]

the same process is used with the middle and maximum anchor curves.

That is, \(R_{\max}^{\mathrm{interp}}\) and \(K^{\mathrm{interp}}_0\) are linearly interpolated, but \(K^{\mathrm{interp}}_1,\ldots,K^{\mathrm{interp}}_{10}\) are derived by sampling and blending the neighboring curves at the common interpolated radius grid.

This distinction matters because the minimum-relief DK1 curve uses:

\[
R_{\max}^{\min} = \sqrt{1.8},
\]

while the middle and maximum curves use \(R_{\max}=1\).

The interpolated table \(K^{\mathrm{interp}}\) is the forward distortion curve for the selected eye relief.

It is used to evaluate

\[
D(r^2).
\]

It is also the curve from which one may build an approximate inverse distortion curve.

---

## 9. Catmull-Rom interpretation of \(K\)

The 11 values in \(K\) do not define a degree-10 polynomial.

Instead, they define a piecewise cubic spline.

Let

\[
s = 10\frac{r^2}{R_{\max}^2}.
\]

The value \(s\) ranges over approximately

\[
0 \le s \le 10.
\]

Let

\[
k = \lfloor s \rfloor,
\qquad
u = s-k,
\]

where \(u \in [0,1]\).

The integer \(k\) selects a spline interval, and \(u\) is the local interpolation coordinate within that interval.

Thus:

\[
s \in [0,1] \quad \text{uses cubic segment 0},
\]

\[
s \in [1,2] \quad \text{uses cubic segment 1},
\]

and so on.

Each segment is cubic in \(u\).

The distortion scale is therefore a piecewise cubic function of \(r^2\):

\[
D(r^2) = \operatorname{CR}\left(K^{\mathrm{interp}},\, 10\frac{r^2}{R_{\max}^2}\right),
\]

where \(\operatorname{CR}\) denotes Catmull-Rom spline evaluation.

---

## 10. Cubic segment formula

A Catmull-Rom segment can be written in cubic Hermite form.

For a segment between values \(p_0\) and \(p_1\), with endpoint slopes \(m_0\) and \(m_1\), define:

\[
h_{00}(u) = 2u^3 - 3u^2 + 1,
\]

\[
h_{10}(u) = u^3 - 2u^2 + u,
\]

\[
h_{01}(u) = -2u^3 + 3u^2,
\]

\[
h_{11}(u) = u^3 - u^2.
\]

Then the spline segment is

\[
H(u)
=
h_{00}(u)p_0
+
h_{10}(u)m_0
+
h_{01}(u)p_1
+
h_{11}(u)m_1.
\]

For an interior segment,

\[
p_0 = K_k,
\qquad
p_1 = K_{k+1},
\]

and the Catmull-Rom slopes are

\[
m_0 = \frac{1}{2}(K_{k+1} - K_{k-1}),
\]

\[
m_1 = \frac{1}{2}(K_{k+2} - K_k).
\]

The segment between \(K_9\) and \(K_{10}\) is special. The SDK uses:

\[
p_0 = K_9,
\qquad
p_1 = K_{10},
\]

\[
m_0 = \frac{1}{2}(K_{10} - K_9),
\qquad
m_1 = K_{10} - K_9.
\]

If the scaled value reaches or exceeds the last control point, the SDK extends the curve as a straight line from \(K_{10}\):

\[
p_0 = K_{10},
\qquad
m_0 = K_{10} - K_9,
\]

\[
p_1 = p_0 + m_0,
\qquad
m_1 = m_0.
\]

The first segment is special. The curve is taken to pass through scale \(1\) at radius zero. Thus, at \(s=0\),

\[
D(0) = 1.
\]

The first segment uses

\[
p_0 = 1,
\qquad
p_1 = K_1,
\]

with a starting slope influenced by \(K_0\). In the SDK-style construction,

\[
m_0 = K_1 - K_0,
\]

\[
m_1 = \frac{1}{2}(K_2 - K_0).
\]

Thus \(K_0\) is not simply the value of the distortion curve at zero radius. Instead, it controls the initial slope.

This is why the table \(K\) should be interpreted as a spline-control table, not as polynomial coefficients.

---

## 11. Channel-dependent chromatic distortion

The green channel uses the base distortion curve:

\[
D_G(r^2) = D(r^2).
\]

For DK1, the chromatic aberration parameters are approximately:

\[
(C_0,C_1,C_2,C_3)
=
(-0.006, 0, 0.014, 0).
\]

The red channel uses

\[
D_R(r^2)
=
D(r^2)(1+C_0+C_1r^2),
\]

and the blue channel uses

\[
D_B(r^2)
=
D(r^2)(1+C_2+C_3r^2).
\]

For DK1 this simplifies to

\[
D_R(r^2) = 0.994D(r^2),
\]

\[
D_G(r^2) = D(r^2),
\]

\[
D_B(r^2) = 1.014D(r^2).
\]

Thus, for a distorted screen-side coordinate \(q_d\), the channel-specific tangent-eye-angle sample coordinates are

\[
q_R = q_dD_R(\|q_d\|^2),
\]

\[
q_G = q_dD_G(\|q_d\|^2),
\]

\[
q_B = q_dD_B(\|q_d\|^2).
\]

---

## 12. Inverse distortion

There are two directions to keep separate.

The SDK's spline scale maps a distorted screen-side tangent-eye-angle coordinate \(q_d\) to an undistorted source tangent-eye-angle coordinate \(q_t\):

\[
q_t = q_dD(\|q_d\|^2).
\]

The inverse radial distortion asks for \(q_d\) given \(q_t\):

\[
q_d = D^{-1}_{\mathrm{radial}}(q_t).
\]

Because the distortion is radial, this reduces to a one-dimensional radius solve:

\[
\rho_t = \rho_dD(\rho_d^2).
\]

Once \(\rho_d\) is found,

\[
q_d = q_t\frac{\rho_d}{\rho_t},
\]

with the special case \(q_d=0\) when \(\rho_t=0\).

### 12.1 SDK direct inverse

The SDK's exact inverse function starts with:

\[
s = 0.25\rho_t,
\qquad
\Delta = 0.25\rho_t.
\]

It then performs 20 local search steps. Let:

\[
f(s)=sD(s^2),
\qquad
d=|\rho_t-f(s)|.
\]

At each step the SDK evaluates:

\[
s_{\mathrm{up}}=s+\Delta,
\qquad
s_{\mathrm{down}}=s-\Delta,
\]

and compares:

\[
d_{\mathrm{up}}=|\rho_t-f(s_{\mathrm{up}})|,
\qquad
d_{\mathrm{down}}=|\rho_t-f(s_{\mathrm{down}})|.
\]

If \(d_{\mathrm{up}} < d\), it moves to \(s_{\mathrm{up}}\). Otherwise, if \(d_{\mathrm{down}} < d\), it moves to \(s_{\mathrm{down}}\). If neither candidate improves the error, it halves the step:

\[
\Delta \leftarrow \frac{1}{2}\Delta.
\]

This is not Newton's method; it is a fixed-count local search around an initial low estimate.

For an offline mesh generator, using this direct inverse is the safest way to match the SDK's source-grid-to-screen-position step.

### 12.2 SDK approximate inverse table

The SDK can also build an approximate inverse spline table, `InvK`.

For \(i=1,\ldots,10\), it samples radii across the valid source tangent-eye-angle radius domain:

\[
\rho_{t,i} =
\frac{i}{10}R_{\max}^{\mathrm{inverse}},
\]

where \(R_{\max}^{\mathrm{inverse}}\) corresponds to the SDK's `MaxInvR`, the source tangent-eye-angle radius reached at the modeled maximum distorted radius:

\[
R_{\max}^{\mathrm{inverse}} = R_{\max}D(R_{\max}^2).
\]

then computes the direct inverse:

\[
\rho_{d,i} = D^{-1}_{\mathrm{radial}}(\rho_{t,i}),
\]

and stores an inverse scale:

\[
\operatorname{InvK}_i = \frac{\rho_{d,i}}{\rho_{t,i}}.
\]

The SDK sets:

\[
\operatorname{InvK}_0 = 1.
\]

The source marks this as a TODO, so for mesh-data generation the direct inverse is the better reference behavior. The important distinction is:

- \(K\) defines the forward distortion curve,
- `InvK` is a derived approximation to its inverse.

---

## 13. SDK mesh grid

The SDK mesh uses a fixed source grid:

\[
64 \times 64
\]

cells per eye.

This produces:

\[
65 \times 65
\]

vertices per eye.

For grid coordinate:

\[
x = 0,\ldots,64,
\qquad
y = 0,\ldots,64,
\]

the source/render-target NDC coordinate is:

\[
s =
\begin{pmatrix}
2x/64 - 1\\
2y/64 - 1
\end{pmatrix}.
\]

This source coordinate is first mapped to tangent-eye-angle space using the render-target transform:

\[
q_t =
\frac{s-o_{\mathrm{src}}}{a_{\mathrm{src}}},
\]

where \(o_{\mathrm{src}}\) and \(a_{\mathrm{src}}\) are the eye-to-source NDC offset and scale used by the SDK.

For the DK1-only generator in this repository, the source transform is derived
from the visible lens radius \(R_L\), eye relief \(e\), configured IPD \(I\),
and lens separation \(L_s\):

\[
a_{\mathrm{src}} =
\begin{pmatrix}
e/R_L\\
e/R_L
\end{pmatrix}.
\]

The horizontal source offset is:

\[
o_{\mathrm{src},x} = \Delta_x/R_L,
\]

where \(\Delta_x\) is the eye offset to the right relative to the lens center:

\[
\Delta_x =
\begin{cases}
(I-L_s)/2, & \text{right eye},\\
-(I-L_s)/2, & \text{left eye}.
\end{cases}
\]

Thus:

\[
q_t.x = s_x R_L/e - \Delta_x/e,
\qquad
q_t.y = s_y R_L/e.
\]

Then the SDK inverse-distorts this tangent-eye-angle coordinate to find the distorted tangent-eye-angle coordinate that would land on this source sample:

\[
q_d = D^{-1}_{\mathrm{radial}}(q_t).
\]

Finally, it maps that distorted tangent-eye-angle coordinate into per-eye screen NDC:

\[
p_s =
\frac{q_d}{a_{\mathrm{tan}}}
+ c,
\]

where:

- \(a_{\mathrm{tan}}\) is `TanEyeAngleScale`,
- \(c\) is the per-eye lens center in screen NDC.

The result is clamped:

\[
p_s.x,p_s.y \in [-1,1].
\]

So the SDK mesh is a regular grid in source space, not in final screen space.

### 13.1 Full-framebuffer screen position

The mesh vertex stores full-framebuffer NDC, not just per-eye NDC.

For the left eye:

\[
P_x = 0.5p_s.x - 0.5,
\]

and for the right eye:

\[
P_x = 0.5p_s.x + 0.5.
\]

For both eyes:

\[
P_y = -p_s.y.
\]

The sign flip appears because the SDK's distortion mesh screen position uses a different vertical convention than the per-eye screen NDC used during construction.

---

## 14. Per-channel sampling coordinates

After computing the vertex screen position, the SDK recomputes tangent-eye-angle sampling coordinates for that screen position.

First:

\[
q_d = (p_s-c)a_{\mathrm{tan}}.
\]

Then:

\[
r^2 = \|q_d\|^2.
\]

For each channel:

\[
q_R = q_dD_R(r^2),
\]

\[
q_G = q_dD_G(r^2),
\]

\[
q_B = q_dD_B(r^2).
\]

These are stored in the vertex as:

- `TanEyeAnglesR`,
- `TanEyeAnglesG`,
- `TanEyeAnglesB`.

The shader later converts them to render-target source coordinates using the eye-to-source scale and offset, then samples the rendered eye texture separately for red, green, and blue.

A renderer may pre-convert these to UVs instead, but the SDK mesh data itself stores tangent-eye-angle coordinates.

---

## 15. Triangle indices

The SDK emits the grid indices in Morton order rather than simple row-major order.

For each Morton-decoded cell, the SDK computes:

\[
F = x(64+1)+y.
\]

Then it alternates the diagonal by quadrant:

\[
(x < 32) \ne (y < 32).
\]

When that expression is true, the triangles are:

\[
(F,F+1,F+66),
\qquad
(F+66,F+65,F).
\]

Otherwise the triangles are:

\[
(F,F+1,F+65),
\qquad
(F+1,F+66,F+65).
\]

The variable names here follow the SDK source. If a new implementation stores vertices in a different row/column order, the exact offsets may change, but the same alternating diagonal pattern should be preserved.

---

## 16. SDK mesh sizes and vertex layout

For \(64 \times 64\) cells:

\[
\text{vertices per eye} = 65 \cdot 65 = 4225,
\]

\[
\text{triangles per eye} = 64 \cdot 64 \cdot 2 = 8192,
\]

\[
\text{indices per eye} = 8192 \cdot 3 = 24576.
\]

The SDK vertex stores:

1. `ScreenPosNDC`,
2. `TimewarpLerp`,
3. `Shade`,
4. `TanEyeAnglesR`,
5. `TanEyeAnglesG`,
6. `TanEyeAnglesB`.

For a DK1-only mesh exporter that does not implement rolling-shutter timewarp, `TimewarpLerp` can be stored but ignored by the shader.

The `Shade` value is a vignette factor. In the SDK mesh generator it combines:

- a fade near the DK1 lens border,
- a fade near the source render-target edge.

---

## 17. Complete SDK-style mathematical flow

For each eye:

1. Determine eye relief.
2. Select the lower and upper DK1 anchor curves.
3. Interpolate \(R_{\max}\), \(K_0\), and the sampled \(K_1,\ldots,K_{10}\) curve values as described above.
4. Define:

\[
D(r^2)
=
\operatorname{CR}
\left(
K,
10\frac{r^2}{R_{\max}^2}
\right).
\]

5. Define chromatic channel scales:

\[
D_R(r^2)=0.994D(r^2),
\]

\[
D_G(r^2)=D(r^2),
\]

\[
D_B(r^2)=1.014D(r^2).
\]

6. For each \(65 \times 65\) source-grid vertex, compute:

\[
s =
\begin{pmatrix}
2x/64 - 1\\
2y/64 - 1
\end{pmatrix}.
\]

7. Convert source NDC to tangent-eye-angle space:

\[
q_t =
\frac{s-o_{\mathrm{src}}}{a_{\mathrm{src}}},
\]

8. Directly inverse-distort this coordinate:

\[
q_d = D^{-1}_{\mathrm{radial}}(q_t).
\]

9. Convert the distorted tangent-eye-angle coordinate to per-eye screen NDC:

\[
p_s = \frac{q_d}{a_{\mathrm{tan}}}+c.
\]

10. Clamp \(p_s\) to \([-1,1]\).
11. Store full-framebuffer vertex position:

\[
P =
\begin{pmatrix}
0.5p_s.x - 0.5 + x_{\mathrm{eyeOffset}}\\
-p_s.y
\end{pmatrix},
\]

where:

\[
x_{\mathrm{eyeOffset}} =
\begin{cases}
0, & \text{left eye},\\
1, & \text{right eye}.
\end{cases}
\]

12. Recompute channel tangent-eye-angle coordinates from \(p_s\):

\[
q_{\mathrm{screen}}=(p_s-c)a_{\mathrm{tan}},
\]

\[
q_R=q_{\mathrm{screen}}D_R(\|q_{\mathrm{screen}}\|^2),
\]

\[
q_G=q_{\mathrm{screen}}D_G(\|q_{\mathrm{screen}}\|^2),
\]

\[
q_B=q_{\mathrm{screen}}D_B(\|q_{\mathrm{screen}}\|^2).
\]

13. Store:

\[
P,\quad q_R,\quad q_G,\quad q_B,\quad \operatorname{Shade},\quad \operatorname{TimewarpLerp}.
\]

14. Generate Morton-ordered triangle indices with the SDK's alternating diagonal rule.

---

## 18. Main points to remember

The table \(K\) is not a polynomial coefficient vector.

It defines a piecewise cubic Catmull-Rom spline in \(r^2\).

The SDK's \(D(r^2)\) scale maps distorted screen-side tangent coordinates to undistorted source tangent-eye-angle coordinates.

The interpolated table \(K^{\mathrm{interp}}\) is the forward screen-to-source distortion curve for the selected eye relief.

The mesh positions are generated by starting from a regular source/render-target grid, converting to tangent-eye-angle space, and directly inverse-distorting that coordinate into screen space.

The SDK's approximate `InvK` table exists, but the 0.4.4 mesh generator calls the direct inverse path when placing mesh vertices.

The A/B/C eye cups are not separate distortion tables in the later DK1 SDK path.

Generate each eye from its final eye-relief value; the 0.4.4 profile path applies a shared dial offset but can start from per-eye eye-to-plate distances.

The mesh stores different tangent-eye-angle sample coordinates for red, green, and blue, plus `Shade` and `TimewarpLerp`.
