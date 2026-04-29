# PID Trajectory Controller - Usage Guide

## Overview
The enhanced PID trajectory controller supports two modes:
1. **Generate trajectories dynamically** with specified target and duration
2. **Load pre-computed trajectories** from CSV files in row-wise format

Both modes use minimum jerk trajectory profiles for smooth motion and PID feedback control for robust tracking.

---

## Installation & Dependencies

```bash
pip install numpy pandas rclpy geometry_msgs sensor_msgs std_msgs
```

---

## Usage Modes

### Mode 1: Generate Trajectory On-the-Fly

Generate a minimum jerk trajectory to a target configuration:

```bash
python3 pid_controller_trajectory.py \
    --target 30 60 90 \
    --T 24 \
    --kp 50 200 150 \
    --ki 5 25 20 \
    --kd 12 35 10
```

**Parameters:**
- `--target Q1 Q2 Q3`: Target joint angles in degrees
- `--T DURATION`: Trajectory duration in seconds (optional, auto-computed if omitted)
- `--start Q1 Q2 Q3`: Start configuration (default: 0 45 135)
- `--kp, --ki, --kd`: PID gains (defaults provided)
- `--torque-limits`: Maximum torques [N⋅m] (default: 2.0 45.0 10.0)
- `--dt`: Control period in seconds (default: 0.01 for 100 Hz)
- `--vel-filter-alpha`: Velocity filter smoothing (0-1, default: 0.25)
- `--no-feedforward`: Disable feedforward from reference acceleration

**Example: Move to 45° on all joints in 20 seconds**
```bash
python3 pid_controller_trajectory.py \
    --target 45 45 45 \
    --T 20
```

---

### Mode 2: Load Trajectory from CSV File

Load a pre-computed trajectory (e.g., from Dataset/Trajectories):

```bash
python3 pid_controller_trajectory.py \
    --trajectory-csv /path/to/trajectory.csv \
    --kp 50 200 150
```

**CSV Format (Row-wise):**
```
t,        t0,       t1,       t2, ...
dp1,      dp1_0,    dp1_1,    dp1_2, ...
dp2,      dp2_0,    dp2_1,    dp2_2, ...
dp3,      dp3_0,    dp3_1,    dp3_2, ...
dv1,      dv1_0,    dv1_1,    dv1_2, ...
dv2,      dv2_0,    dv2_1,    dv2_2, ...
dv3,      dv3_0,    dv3_1,    dv3_2, ...
da1,      da1_0,    da1_1,    da1_2, ... (optional)
da2,      da2_0,    da2_1,    da2_2, ... (optional)
da3,      da3_0,    da3_1,    da3_2, ... (optional)
```

**Example: Load a trajectory and track it**
```bash
python3 pid_controller_trajectory.py \
    --trajectory-csv /home/priyankan/Desktop/FYP-Puma_560/Dataset/Trajectories/path_603_traj.csv \
    --kp 50 200 150
```

---

## Control Strategy

### Trajectory Tracking Control Law:

$$\tau = K_p (q_{ref} - q) + K_d (\dot{q}_{ref} - \dot{q}) + K_i \int (q_{ref} - q) dt + K_{ff} \ddot{q}_{ref}$$

Where:
- **$q_{ref}$, $\dot{q}_{ref}$, $\ddot{q}_{ref}$**: Reference position, velocity, acceleration from trajectory
- **$q$, $\dot{q}$**: Current joint position and velocity
- **$K_p, K_d, K_i$**: Proportional, derivative, integral gains
- **$K_{ff}$**: Feedforward gain (0.1 × acceleration, optional)

### Features:
1. **Minimum Jerk Profile**: Smooth acceleration/deceleration minimizes vibration
2. **Feedforward Control**: Uses reference trajectory acceleration for faster convergence
3. **Integral Anti-Windup**: Clipped at ±0.75 rad to prevent instability
4. **Velocity Filtering**: Low-pass filter (α=0.25) smooths noisy velocity estimates
5. **Torque Saturation**: All torques clipped to joint limits

---

## Logging & Output

Logs are saved automatically to `./logs/` directory:

**Log Format (CSV, row-wise):**
```
t,   time values (seconds)
dp1, reference position j1 (rad)
dp2, reference position j2 (rad)
dp3, reference position j3 (rad)
q1,  actual position j1 (rad)
q2,  actual position j2 (rad)
q3,  actual position j3 (rad)
qd1, actual velocity j1 (rad/s)
qd2, actual velocity j2 (rad/s)
qd3, actual velocity j3 (rad/s)
tau1, applied torque j1 (N⋅m)
tau2, applied torque j2 (N⋅m)
tau3, applied torque j3 (N⋅m)
```

### Plotting Results:

Use the plot utility to visualize tracking performance:

```bash
python3 /path/to/plot.py /path/to/log.csv
```

---

## Trajectory Generation Constraints

When auto-computing trajectory duration:

- **Maximum velocity**: 2.0 rad/s
- **Maximum acceleration**: 7.0 rad/s²
- **Formula**: 
  - $T_{vel} = \frac{1.875 \cdot \Delta q}{v_{max}}$
  - $T_{acc} = \sqrt{\frac{5.77 \cdot \Delta q}{a_{max}}}$
  - $T = \max(T_{vel}, T_{acc})$

For example, moving 1 radian requires minimum:
- Time for velocity: 0.9375 seconds
- Time for acceleration: 0.759 seconds
- **Minimum duration: 0.9375 seconds**

---

## PID Tuning Tips

### Increasing Response Speed:
- Increase $K_p$ (proportional gain)
- Increase $K_d$ (derivative gain to damp oscillations)

### Reducing Steady-State Error:
- Increase $K_i$ (integral gain)
- Enable feedforward (`--no-feedforward` not used)

### Reducing Oscillation:
- Increase $K_d$ (derivative damping)
- Decrease velocity filter alpha (more filtering)

### Preventing Integral Windup:
- Clipping already active at ±0.75 rad
- Reduce $K_i$ if windup still occurs

**Default Gains (tuned for PUMA-560):**
```
Kp = [50,   200,  150]
Ki = [5,    25,   20]
Kd = [12,   35,   10]
```

---

## Example Workflows

### Workflow 1: Generate and Track Custom Trajectory
```bash
# Move from home (0, 45, 135)° to target (30, 60, 90)° in 15 seconds
python3 pid_controller_trajectory.py \
    --target 30 60 90 \
    --T 15 \
    --start 0 45 135
```

### Workflow 2: Track Pre-Generated Dataset Path
```bash
# Load and track one of the generated dataset trajectories
python3 pid_controller_trajectory.py \
    --trajectory-csv ../../../Dataset/Trajectories/path_603_traj.csv \
    --kp 50 200 150 \
    --kd 12 35 10
```

### Workflow 3: Custom PID Gains
```bash
# Use aggressive tuning
python3 pid_controller_trajectory.py \
    --target 45 45 45 \
    --T 10 \
    --kp 80 250 200 \
    --ki 10 40 30 \
    --kd 15 50 15
```

---

## Troubleshooting

### Issue: Trajectory not loading
**Solution:** Verify CSV format matches expected row-wise structure with headers: `t, dp1, dp2, dp3, dv1, dv2, dv3`

### Issue: High tracking error
**Solution:** 
1. Increase $K_p$ (proportional gain)
2. Increase trajectory duration with `--T`
3. Check joint velocity limits (default 2 rad/s)

### Issue: Oscillations/Instability
**Solution:**
1. Decrease $K_d$ or increase velocity filter (α closer to 0)
2. Check torque limits (`--torque-limits`)
3. Reduce $K_p$

### Issue: Slow response
**Solution:**
1. Increase $K_p$ and $K_d$
2. Decrease trajectory duration `--T`
3. Enable feedforward (default)

---

## File Structure

```
pid_controller/
├── pid_controller_trajectory.py    ← New trajectory-tracking controller
├── pid_controller.py               ← Original point-to-point controller
├── logs/                           ← Auto-generated logs
└── TRAJECTORY_GUIDE.md             ← This file
```

---

## References

- **Minimum Jerk Profile**: Hogan, N. (1984), "Impedance Control of Robot Manipulators"
- **PUMA-560 Kinematics**: DH parameters from standard reference
- **PID Control**: standard feedback control theory

