# S-Curve (Minimum-Jerk) Derivation for `gen_traj.py`

This note explains exactly how the trajectory profile in `gen_traj.py` is derived, including where the coefficients come from and how they are used for each generated trajectory.

## 1. Problem Setup Per Trajectory

For each trajectory, the code samples a target joint vector:

$$
q_{end} = [q_{1,end}, q_{2,end}, q_{3,end}]^T
$$

with fixed start point:

$$
q_{start} = [q_{1,start}, q_{2,start}, q_{3,start}]^T
$$

Define displacement per joint:

$$
\Delta q_j = q_{j,end} - q_{j,start}
$$

Then all joints follow the same normalized scalar profile $f(t)$, scaled by each joint's displacement.

## 2. Why a 5th-Order Polynomial?

A rest-to-rest smooth motion requires these 6 boundary constraints:

- Position at start and end.
- Velocity at start and end equals zero.
- Acceleration at start and end equals zero.

So we need at least 6 polynomial coefficients, which means degree 5:

$$
f(\tau) = a_0 + a_1\tau + a_2\tau^2 + a_3\tau^3 + a_4\tau^4 + a_5\tau^5,
\quad \tau = \frac{t}{T}
$$

Boundary conditions in normalized form:

$$
f(0)=0,\; f(1)=1,\; f'(0)=0,\; f'(1)=0,\; f''(0)=0,\; f''(1)=0
$$

From conditions at $\tau=0$:

$$
a_0=0,\; a_1=0,\; a_2=0
$$

So:

$$
f(\tau)=a_3\tau^3+a_4\tau^4+a_5\tau^5
$$

Apply end conditions at $\tau=1$:

$$
a_3 + a_4 + a_5 = 1
$$
$$
3a_3 + 4a_4 + 5a_5 = 0
$$
$$
6a_3 + 12a_4 + 20a_5 = 0
$$

Solving gives:

$$
a_3=10,\quad a_4=-15,\quad a_5=6
$$

Therefore:

$$
\boxed{f(\tau)=10\tau^3-15\tau^4+6\tau^5}
$$

These are the exact coefficients used in code.

## 3. Time Derivatives and Code Formulas

In `min_jerk(t, T)`:

$$
\tau=\frac{t}{T}
$$

### Position profile

$$
f(\tau)=10\tau^3-15\tau^4+6\tau^5
$$

### Velocity profile

Differentiate with chain rule $\frac{d}{dt}=\frac{1}{T}\frac{d}{d\tau}$:

$$
\dot f(t)=\frac{30\tau^2-60\tau^3+30\tau^4}{T}
$$

### Acceleration profile

$$
\ddot f(t)=\frac{60\tau-180\tau^2+120\tau^3}{T^2}
$$

These match the script exactly:

- `f = 10*tau**3 - 15*tau**4 + 6*tau**5`
- `fd = (30*tau**2 - 60*tau**3 + 30*tau**4) / T`
- `fdd = (60*tau - 180*tau**2 + 120*tau**3) / T**2`

## 4. Derivation of the Constants 1.875 and 5.77

The timing constraints in code use:

- `1.875` for velocity bound.
- `5.77` for acceleration bound.

These come from maxima of normalized derivatives.

### 4.1 Velocity constant (1.875)

Normalized velocity shape:

$$
g(\tau)=30\tau^2-60\tau^3+30\tau^4
$$

Its maximum occurs at $\tau=0.5$ (symmetry or by setting derivative to zero), giving:

$$
g(0.5)=30(0.25)-60(0.125)+30(0.0625)=1.875
$$

Hence:

$$
\max |\dot f| = \frac{1.875}{T}
$$

So per joint:

$$
\max|\dot q_j| = \frac{1.875\,|\Delta q_j|}{T}
$$

which yields the code bound:

$$
T \ge \frac{1.875\,|\Delta q_j|}{v_{max}}
$$

and across joints:

$$
T_{vel}=\max_j\left(\frac{1.875\,|\Delta q_j|}{v_{max}}\right)
$$

### 4.2 Acceleration constant (5.77)

Normalized acceleration shape:

$$
h(\tau)=60\tau-180\tau^2+120\tau^3
$$

Set derivative to zero:

$$
h'(\tau)=60-360\tau+360\tau^2=0
$$
$$
6\tau^2-6\tau+1=0
\Rightarrow
\tau=\frac{3\pm\sqrt3}{6}
$$

At those points, the peak magnitude is:

$$
\max |h(\tau)| = \frac{10}{\sqrt3} \approx 5.7735
$$

So:

$$
\max |\ddot f| = \frac{5.7735}{T^2} \approx \frac{5.77}{T^2}
$$

and per joint:

$$
\max|\ddot q_j| = \frac{5.77\,|\Delta q_j|}{T^2}
$$

Thus:

$$
T \ge \sqrt{\frac{5.77\,|\Delta q_j|}{a_{max}}},
\quad
T_{acc}=\max_j\sqrt{\frac{5.77\,|\Delta q_j|}{a_{max}}}
$$

## 5. Final Trajectory Equations Used for Each Path

For each sampled endpoint and selected $T_{total}$:

$$
q_j(t)=q_{j,start}+f(t)\,\Delta q_j
$$
$$
\dot q_j(t)=\dot f(t)\,\Delta q_j
$$
$$
\ddot q_j(t)=\ddot f(t)\,\Delta q_j
$$

Vector form (matching `np.outer` implementation):

$$
q(t)=q_{start}+f(t)\,\Delta q
$$
$$
\dot q(t)=\dot f(t)\,\Delta q
$$
$$
\ddot q(t)=\ddot f(t)\,\Delta q
$$

where $\Delta q = q_{end}-q_{start}$.

## 6. How This Maps to `gen_traj.py`

Per generated trajectory:

1. Sample `q_end` within joint limits.
2. Compute $\Delta q = |q_{end}-q_{start}|$ for timing constraints.
3. Compute `T_vel` and `T_acc` from the derived constants.
4. Set `T_min = max(T_vel, T_acc)`.
5. Sample `T_rand` from `{12, 16, 20, 24}` and set `T_total = max(T_min, T_rand)`.
6. Build `t = np.arange(0, T_total + dt, dt)`.
7. Evaluate `f, fd, fdd = min_jerk(t, T_total)`.
8. Scale by signed displacement `dq = q_end - q_start` to produce `q`, `qd`, `qdd`.

That is why every trajectory has the same polynomial coefficients, but different shape in joint space due to different $\Delta q$ and $T_{total}$.
