# PID Controller - Quick Reference

## Two Controllers Available

### 1. Point-to-Point: `pid_controller.py`
Direct movement to target (original)

```bash
python3 pid_controller.py --target 45 60 90
```

### 2. Trajectory Tracking: `pid_controller_trajectory.py`
Smooth path following (NEW)

```bash
# Generate trajectory
python3 /home/priyankan/Desktop/FYP-Puma_560/arm_bot/src/scripts/controller/pid_controller/pid_controller_trajectory.py --target 45 60 90 --T 20

# Load from CSV
python3 /home/priyankan/Desktop/FYP-Puma_560/arm_bot/src/scripts/controller/pid_controller/pid_controller_trajectory.py --trajectory-csv /home/priyankan/Desktop/FYP-Puma_560/Dataset/Trajectories/path_1122_traj.csv

python3 /home/priyankan/Desktop/FYP-Puma_560/arm_bot/src/scripts/controller/pid_controller/pid_controller_trajectory.py --trajectory-csv /home/priyankan/Downloads/path_004_traj.csv
```

---

## Essential Parameters

| Argument | Values | Default | Example |
|----------|--------|---------|---------|
| `--target` | Q1 Q2 Q3 (deg) | - | `0 45 90` |
| `--T` | Duration (sec) | auto | `24` |
| `--kp` | Gains [j1 j2 j3] | [50, 200, 150] | `75 250 200` |
| `--ki` | Gains [j1 j2 j3] | [5, 25, 20] | `7.5 37.5 30` |
| `--kd` | Gains [j1 j2 j3] | [12, 35, 10] | `18 52 15` |

---

## Common Commands

```bash
# Move to position in 10 seconds
python3 pid_controller_trajectory.py --target 30 45 90 --T 10

# Use aggressive gains
--kp 75 250 200 --ki 7.5 37.5 30 --kd 18 52 15

# Load dataset trajectory
--trajectory-csv ../../../Dataset/Trajectories/path_603_traj.csv

# Disable feedforward
--no-feedforward

# Custom velocity filter
--vel-filter-alpha 0.1
```

---

## Control Law

**Trajectory Tracking:**
$$\tau = K_p(q_{ref} - q) + K_d(\dot{q}_{ref} - \dot{q}) + K_i\int(q_{ref} - q)dt + 0.1\ddot{q}_{ref}$$

**Point-to-Point:**
$$\tau = K_p(q_{target} - q) + K_d(0 - \dot{q}) + K_i\int(q_{target} - q)dt$$

---

## CSV Format (for loading trajectories)

```
t,   0,      0.01,    0.02
dp1, 0,      0.001,   0.005     ← Position in radians
dp2, 0.785,  0.786,   0.787
dp3, 2.356,  2.358,   2.361
dv1, 0,      0.1,     0.2       ← Velocity in rad/s
dv2, 0,      0.15,    0.3
dv3, 0,      0.12,    0.24
```

---

## Quick Tuning

| Problem | Fix |
|---------|-----|
| Too slow | Increase Kp/Kd |
| Oscillates | Increase Kd |
| High error | Increase Ki |
| Jerky | Use trajectory mode with --T |

---

## Log Location & Analysis

Logs saved to: `./logs/pid_[traj_]log_TIMESTAMP.csv`

View plots:
```bash
python3 /path/to/plot.py logs/pid_traj_log_*.csv
```

---

## Key Differences

| Aspect | Point-to-Point | Trajectory |
|--------|---|---|
| Speed | Fast | Smooth |
| Path | Direct line | Minimum jerk |
| Control | Error only | Error + feedforward |
| Log file | `pid_log_*.csv` | `pid_traj_log_*.csv` |

