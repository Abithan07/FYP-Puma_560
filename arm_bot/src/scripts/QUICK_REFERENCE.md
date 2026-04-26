# Computed Torque Control - Quick Reference Card

## Quick Start (Copy-Paste)

```bash
# Terminal 1: Start Gazebo
cd ~/Desktop/FYP-Puma_560/arm_bot && source install/setup.bash
ros2 launch arm_bot gazebo.launch.py

# Terminal 2: Run CTC (in another terminal)
cd ~/Desktop/FYP-Puma_560/arm_bot && source install/setup.bash
ros2 run arm_bot torque_publisher_ctc.py \
  --csv-path src/scripts/script_resources/trajectory_complete.csv

# Terminal 3: Analyze results (after execution)
cd ~/Desktop/FYP-Puma_560 && python3 arm_bot/src/scripts/analyze_ctc_performance.py \
  arm_bot/src/scripts/script_resources/path_001_trajectory_ctc_log_1.csv --summary
```

---

## Common Commands

### The 3 Main Control Configurations

#### 1) Full Computed Torque Control
Uses both the model term and feedback term.

```bash
ros2 run arm_bot torque_publisher_ctc.py \
  --csv-path src/scripts/script_resources/path_001_trajectory.csv
```

#### 2) Model-Based Only
Uses inverse dynamics only, no feedback correction.

```bash
ros2 run arm_bot torque_publisher_ctc.py \
  --csv-path src/scripts/script_resources/path_001_trajectory.csv \
  --no-feedback
```

#### 3) Feedback-Only
Uses PD-style feedback only, no model feedforward.

```bash
ros2 run arm_bot torque_publisher_ctc.py \
  --csv-path src/scripts/script_resources/path_001_trajectory.csv \
  --no-model
```

### Run CTC with Default Gains
```bash
ros2 run arm_bot torque_publisher_ctc.py --csv-path trajectory.csv
```

### Run with Custom Control Gains
```bash
ros2 run arm_bot torque_publisher_ctc.py \
  --csv-path trajectory.csv \
  --kp 30 100 50 \
  --kd 4 10 5
```

### Run Model-Based Only (No Feedback)
```bash
ros2 run arm_bot torque_publisher_ctc.py \
  --csv-path trajectory.csv \
  --no-feedback
```

### Run Feedback-Only (No Model)
```bash
ros2 run arm_bot torque_publisher_ctc.py \
  --csv-path trajectory.csv \
  --no-model
```

### Save Log to Specific Location
```bash
ros2 run arm_bot torque_publisher_ctc.py \
  --csv-path trajectory.csv \
  --log-path /tmp/my_experiment.csv
```

### Example: Tune Only Proportional Gains
```bash
ros2 run arm_bot torque_publisher_ctc.py \
  --csv-path src/scripts/script_resources/path_001_trajectory.csv \
  --kp 40 120 60
```

### Example: Tune Kp and Kd Together
```bash
ros2 run arm_bot torque_publisher_ctc.py \
  --csv-path src/scripts/script_resources/path_001_trajectory.csv \
  --kp 40 120 60 \
  --kd 6 12 6
```

### Example: Add Integral Action
```bash
ros2 run arm_bot torque_publisher_ctc.py \
  --csv-path src/scripts/script_resources/path_001_trajectory.csv \
  --ki 0.1 0.5 0.5
```

### Example: Change Dynamics Evaluation Frame
```bash
ros2 run arm_bot torque_publisher_ctc.py \
  --csv-path src/scripts/script_resources/path_001_trajectory.csv \
  --dynamics-frame desired
```

### Example: Set Torque Limits
```bash
ros2 run arm_bot torque_publisher_ctc.py \
  --csv-path src/scripts/script_resources/path_001_trajectory.csv \
  --torque-limits 90 90 45
```

### Example: Slow Down Velocity Filtering
```bash
ros2 run arm_bot torque_publisher_ctc.py \
  --csv-path src/scripts/script_resources/path_001_trajectory.csv \
  --vel-filter-alpha 0.1
```

### Example: Skip Stabilization for Nearly Aligned Start Pose
```bash
ros2 run arm_bot torque_publisher_ctc.py \
  --csv-path src/scripts/script_resources/path_001_trajectory.csv \
  --skip-stabilization-threshold-deg 2.0
```

### Default Log Naming
```text
path_001_trajectory.csv -> path_001_trajectory_ctc_log_1.csv
path_001_trajectory.csv -> path_001_trajectory_ctc_log_2.csv
path_001_trajectory.csv -> path_001_trajectory_ctc_log_3.csv
```

---

## Analysis Commands

### Print Summary Statistics
```bash
python3 arm_bot/src/scripts/analyze_ctc_performance.py ctc_log_*.csv --summary
```

### Generate All Plots
```bash
python3 arm_bot/src/scripts/analyze_ctc_performance.py ctc_log_*.csv --output-dir results/
```

### Example: Use a Specific Log File
```bash
python3 arm_bot/src/scripts/analyze_ctc_performance.py \
  arm_bot/src/scripts/script_resources/path_001_trajectory_ctc_log_1.csv \
  --summary
```

### Example: Generate Plots for One Run
```bash
python3 arm_bot/src/scripts/analyze_ctc_performance.py \
  arm_bot/src/scripts/script_resources/path_001_trajectory_ctc_log_1.csv \
  --plots --output-dir arm_bot/src/scripts/plots
```

### Generate Single Plot
```bash
python3 arm_bot/src/scripts/analyze_ctc_performance.py ctc_log_*.csv \
  --plot-errors errors.png
```

### Generate Multiple Specific Plots
```bash
python3 arm_bot/src/scripts/analyze_ctc_performance.py ctc_log_*.csv \
  --plot-trajectory results/traj.png \
  --plot-errors results/err.png \
  --plot-torques results/tau.png \
  --plot-velocity results/vel.png
```

---

## File Locations

| What | Where |
|------|-------|
| Main controller | `arm_bot/src/scripts/torque_publisher_ctc.py` |
| Inverse dynamics | `arm_bot/src/scripts/inverse_dynamics_model.py` |
| Analysis tool | `arm_bot/src/scripts/analyze_ctc_performance.py` |
| Example trajectory | `arm_bot/src/scripts/script_resources/trajectory_complete.csv` |
| Control guide | `arm_bot/src/scripts/README_CTC.md` |
| Log output | `path_<id>_trajectory_ctc_log_<run>.csv` |

---

## Control Law Summary

**Full Computed Torque Control:**
```
τ = D(q)·q̈ + C(q,q̇) + G(q) + Kp·(q_des - q_act) + Kd·(q̇_des - q̇_act)
  = τ_model                 + τ_fb
```

**Control Terms:**
- `D(q)`: Inertia matrix (3×3 mass/geometry)
- `C(q,q̇)`: Coriolis/centrifugal vector
- `G(q)`: Gravity vector
- `Kp`: Proportional gains [30, 100, 50]
- `Kd`: Derivative gains [4, 10, 5]
- `e_pos = q_des - q_act`: Position tracking error
- `e_vel = q̇_des - q̇_act`: Velocity tracking error

---

## Typical Parameter Values

| Parameter | Joint 1 | Joint 2 | Joint 3 | Unit |
|-----------|---------|---------|---------|------|
| Kp (Prop) | 30 | 100 | 50 | - |
| Kd (Deriv) | 4 | 10 | 5 | - |
| τ_max | 100 | 100 | 50 | N⋅m |
| Mass | 0.01 | 17.4 | 4.8 | kg |

---

## Troubleshooting Quick Fixes

| Problem | Quick Fix |
|---------|-----------|
| Joint stuck during init | Increase `--skip-stabilization-threshold-deg` |
| Large tracking error | Increase `--kp` values |
| Oscillations | Increase `--kd` or decrease `--kp` |
| CSV file not found | Use absolute path, check with `ls` |
| Module import error | Verify in `arm_bot/src/scripts/` directory |
| Wrong control mode | Check if `--no-feedback` or `--no-model` is set |
| Analysis tool fails | Ensure pandas/matplotlib: `pip3 install pandas matplotlib` |

---

## Joint Angle Limits

| Joint | Min (°) | Max (°) | Range (°) |
|-------|---------|---------|-----------|
| 1 | -100 | +100 | 200 |
| 2 | -15 | +45 | 60 |
| 3 | 65 | 205 | 140 |

---

## Performance Targets

| Metric | Target | Typical |
|--------|--------|---------|
| RMS Position Error | < 0.5° | 0.3-0.5° |
| Max Position Error | < 1.5° | 0.8-1.2° |
| Stabilization Time | < 30s | 10-20s |
| Feedback Contribution | 10-30% | 15-25% |
| Peak Command Torque | < 100 Nm | 80-95 Nm |

---

## Log CSV Columns (25+)

```
t,
q_des_1/2/3, qd_des_1/2/3, qdd_des_1/2/3,  ← Desired trajectory
q_act_1/2/3, qd_act_1/2/3,                  ← Actual states
tau_model_1/2/3,                            ← Feedforward (inverse dyn)
tau_fb_1/2/3,                               ← Feedback correction
tau_total_1/2/3,                            ← Commanded torque
e_pos_1/2/3, e_vel_1/2/3                    ← Tracking errors
```

---

## Typical Workflow

```
1. Setup
   └─ cd ~/Desktop/FYP-Puma_560/arm_bot
   └─ source install/setup.bash

2. Run Simulation
   └─ Terminal 1: ros2 launch arm_bot gazebo.launch.py
   └─ Terminal 2: ros2 run arm_bot torque_publisher_ctc.py --csv-path ...

3. Execution Time
   └─ Stabilization: ~10-20s (if needed)
   └─ Trajectory: ~10-60s (depends on trajectory)
   └─ Output: ctc_log_*.csv

4. Analysis
   └─ python3 analyze_ctc_performance.py ctc_log_*.csv --summary
   └─ python3 analyze_ctc_performance.py ctc_log_*.csv --output-dir results/

5. Interpretation
   └─ Check tracking error vs targets
   └─ Compare model vs feedback contribution
   └─ Review generated plots
   └─ Adjust Kp/Kd if needed
```

---

## Environment Setup (One-Time)

```bash
# Add to .bashrc for convenience
alias ctc_dir='cd ~/Desktop/FYP-Puma_560/arm_bot && source install/setup.bash'

# Install analysis dependencies (if not already)
pip3 install numpy scipy pandas matplotlib

# Verify ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash
```

---

## Save Results

```bash
# Create results directory
mkdir -p results

# Copy log file
cp ctc_log_*.csv results/

# Generate all plots in results directory
python3 arm_bot/src/scripts/analyze_ctc_performance.py \
  results/ctc_log_*.csv \
  --output-dir results/

# Archive results
tar -czf results_$(date +%Y%m%d_%H%M%S).tar.gz results/
```

---

## Testing Checklist

Before running main experiments:
- [ ] Gazebo simulation launches without errors
- [ ] Joint state feedback works (`/joint_states` topic)
- [ ] CSV file exists and is readable
- [ ] Control node starts without crashes
- [ ] Stabilization phase works
- [ ] Trajectory executes smoothly
- [ ] Log file is created
- [ ] Analysis tool runs on log file

---

## Tips & Tricks

**To find latest log file:**
```bash
ls -lt ctc_log_*.csv | head -1
```

**To compare multiple logs:**
```bash
for f in ctc_log_*.csv; do
  echo "=== $f ===" 
  python3 arm_bot/src/scripts/analyze_ctc_performance.py "$f" --summary
done
```

**To extract specific column from CSV:**
```bash
cut -d, -f1,11,12,13 ctc_log_*.csv | head -20  # time + tau_total columns
```

**To plot in real-time (Linux only):**
```bash
while true; do
  python3 analyze_ctc_performance.py ctc_log_*.csv --plot-errors err.png
  display err.png  # requires ImageMagick
done
```

---

## References

| Document | Purpose | Lines |
|----------|---------|-------|
| `README_CTC.md` | Short control guide | ~50 |
| `QUICK_REFERENCE.md` | Commands and troubleshooting | ~300 |
| `torque_publisher_ctc.py` | Main control code | ~1100 |
| `inverse_dynamics_model.py` | Inverse dynamics implementation | ~250 |
| `analyze_ctc_performance.py` | Analysis tool | ~400 |

**Total Documentation**: 350+ lines
**Total Code**: 1750+ lines

---

## Support

For detailed help:
1. Read `README_CTC.md` (short overview)
2. Check this quick reference for commands and troubleshooting
3. Inspect source code inline comments

---

**Quick Reference Card v1.0**
**Last Updated**: April 26, 2026
