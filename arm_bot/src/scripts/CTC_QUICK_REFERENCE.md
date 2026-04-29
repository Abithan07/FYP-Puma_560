# Computed Torque Control - Quick Reference Card

---

## Quick Start
### For the first time run the following in each terminal
```bash
cd ~/Desktop/FYP-Puma_560/arm_bot
colcon build --symlink-install
source install/setup.bash
```

```bash
# Terminal 1 — Gazebo
ros2 launch arm_bot gazebo.launch.py

# Terminal 2 — CTC (pre-generated trajectory)
ros2 run arm_bot torque_publisher_ctc.py \
  --csv-path /home/priyankan/Desktop/FYP-Puma_560/Joint_states_601_1120/path_610_joint_states.csv

# Terminal 2 — CTC (on-the-fly trajectory from end position)
ros2 run arm_bot torque_publisher_ctc.py \
  --q-end 30 20 100          # joint end positions in degrees
                             # q_start defaults to [0, 45, 135] deg; T selected from [12,17,22]s

# Terminal 3 — Analyse after run
cd ~/Desktop/FYP-Puma_560
python3 src/scripts/analyze_ctc_performance.py \
src/scripts/logs/path_870_joint_states_ctc_log_1.csv --plots  
# use --summary to get text summary in the terminal

```

---

## All CLI Options

```
--csv-path PATH                  Pre-generated trajectory CSV (mutually exclusive with --q-end)
--q-end Q1 Q2 Q3                 End joint positions in degrees; generates trajectory on the fly
--q-start Q1 Q2 Q3               Start joint positions in degrees (default: 0 45 135)
--kp KP1 KP2 KP3                 Proportional gains (default: 30 100 50)
--kd KD1 KD2 KD3                 Derivative gains   (default:  4  10  5)
--ki KI1 KI2 KI3                 Integral gains     (default: 0.2 1.0 0.8)
--torque-limits T1 T2 T3         Saturation limits in Nm (default: 100 100 60)
--vel-filter-alpha ALPHA         Velocity low-pass alpha 0-1 (default: 0.25)
--dynamics-frame {actual|desired} State used for D,C,G (default: actual)
--skip-stabilization-threshold-deg DEG  Skip Phase 1 if error < DEG (default: 1.0)
--no-feedback                    Disable PD feedback term (model feedforward only)
--no-model                       Disable inverse-dynamics term (feedback only)
--log-path PATH                  Override auto-generated log path
```

---

## File & Directory Map

```
FYP-Puma_560/arm_bot/src/scripts/
│
├── torque_publisher_ctc.py          ← Main CTC controller
├── inverse_dynamics_model.py        ← D(q), C(q,q̇), G(q) implementation
├── continuous_logger_triggered.py   ← Joint-state logger (launched as subprocess)
├── analyze_ctc_performance.py       ← Post-run analysis & plotting tool
│
├── script_resources/                ← INPUT: pre-generated trajectory CSVs
│   ├── path_001_trajectory.csv
│   ├── path_002_trajectory.csv
│   └── trajectory_complete.csv
│
└── logs/                            ← OUTPUT: everything written at runtime
    ├── gen_traj_qend_<Q1>_<Q2>_<Q3>.csv   ← trajectory generated from --q-end
    └── <traj_name>_ctc_log_<run>.csv       ← CTC control log (per run)
```

### Input trajectories (`script_resources/`)

Pre-generated trajectory CSV files must be placed here.
The format (one row per signal, columns = time-steps):

```
t,   0.000, 0.010, 0.020, ...
dp1, ...    ← desired joint 1 position (rad)
dp2, ...    ← desired joint 2 position (rad)
dp3, ...    ← desired joint 3 position (rad)
dv1, ...    ← desired joint 1 velocity (rad/s)
dv2, ...
dv3, ...
da1, ...    ← desired joint 1 acceleration (rad/s²)   [optional: computed if absent]
da2, ...
da3, ...
```

### Output files (`logs/`)

| File | When created | Contents |
|------|-------------|----------|
| `gen_traj_qend_<Q1>_<Q2>_<Q3>.csv` | `--q-end` mode only | Generated min-jerk trajectory |
| `<traj_name>_ctc_log_<run>.csv` | Every CTC run | Full per-timestep control data (see Log Columns below) |

The run counter (`_1`, `_2`, …) auto-increments — previous logs are never overwritten.

> **To change the output directory** → edit `LOGS_DIR` at the top of the `TorquePublisher`
> class in `torque_publisher_ctc.py` (line ~172):
> ```python
> LOGS_DIR = '/home/priyankan/Desktop/FYP-Puma_560/arm_bot/src/scripts/logs'
> ```

---

## Where to Change Test Variables

### Option A — CLI flags (preferred, no code edits needed)

```bash
ros2 run arm_bot torque_publisher_ctc.py \
  --csv-path ...           \
  --kp  30 100 50          \   # Proportional gains  [J1, J2, J3]
  --kd   4  10  5          \   # Derivative gains
  --ki 0.2 1.0 0.8         \   # Integral gains
  --torque-limits 100 100 60 \ # Torque saturation limits (Nm)
  --vel-filter-alpha 0.25  \   # Velocity low-pass  (0=frozen, 1=raw)
  --dynamics-frame actual  \   # 'actual' or 'desired' for D,C,G evaluation
  --skip-stabilization-threshold-deg 1.0
```

### Option B — Hard-coded defaults in `torque_publisher_ctc.py`

Edit these when you want the change to persist across all runs without typing flags:

| Variable | Location (approx. line) | Default |
|----------|------------------------|---------|
| **Trajectory Kp** | `__init__` → `self.kp` (~line 60) | `[30, 100, 50]` |
| **Trajectory Kd** | `__init__` → `self.kd` (~line 65) | `[4, 10, 5]` |
| **Trajectory Ki** | `__init__` → `self.ki` (~line 71) | `[0.2, 1.0, 0.8]` |
| **Torque limits** | `__init__` → `self.torque_limits` (~line 76) | `[100, 100, 60]` Nm |
| **Velocity filter α** | `__init__` → `vel_filter_alpha` (~line 49) | `0.25` |
| **Integral anti-windup cap** | `compute_control_torques()` → `max_i` (~line 562) | `[0.4, 0.8, 0.8]` |
| **Stabilisation Kp/Ki/Kd** | `stabilization_callback()` (~line 631) | `[50,200,150]` / `[5,25,20]` / `[12,35,10]` |
| **Stabilisation torque limits** | `stabilization_callback()` → `max_torques` (~line 655) | `[100, 100, 50]` Nm |
| **Output directory** | class var `LOGS_DIR` (~line 172) | `src/scripts/logs` |

### For `--q-end` trajectory generation

| Variable | Location | Default |
|----------|----------|---------|
| **Possible durations** | `generate_and_save_trajectory()` → `possible_T` | `np.arange(12,25,5)` → [12, 17, 22] s |
| **Max velocity** | same method → `v_max` | `2.0` rad/s |
| **Max acceleration** | same method → `a_max` | `7.0` rad/s² |
| **Default q_start** | `--q-start` CLI arg default | `[0, 45, 135]` deg |

---

## The 3 Control Modes

| Mode | Flag | Torque law |
|------|------|------------|
| **Full CTC** (default) | *(none)* | τ = D·q̈ + C + G + Kp·e + Kd·ė + Ki·∫e |
| **Model only** | `--no-feedback` | τ = D·q̈ + C + G |
| **Feedback only** | `--no-model` | τ = Kp·e + Kd·ė + Ki·∫e |

---

## Common Tuning Examples

```bash
# Tighten tracking — raise Kp
ros2 run arm_bot torque_publisher_ctc.py --csv-path ... --kp 40 120 60

# Damp oscillations — raise Kd
ros2 run arm_bot torque_publisher_ctc.py --csv-path ... --kd 6 15 8

# Both together
ros2 run arm_bot torque_publisher_ctc.py --csv-path ... --kp 40 120 60 --kd 6 15 8

# Add integral to remove steady-state bias
ros2 run arm_bot torque_publisher_ctc.py --csv-path ... --ki 0.1 0.5 0.5

# Smoother velocity feedback (less noise sensitivity)
ros2 run arm_bot torque_publisher_ctc.py --csv-path ... --vel-filter-alpha 0.1

# Reduce torque saturation for safety testing
ros2 run arm_bot torque_publisher_ctc.py --csv-path ... --torque-limits 60 60 40

# Skip Phase 1 if robot is already near start
ros2 run arm_bot torque_publisher_ctc.py --csv-path ... --skip-stabilization-threshold-deg 2.0
```

---

## Typical Workflow

```
1. Setup
   └─ cd ~/Desktop/FYP-Puma_560/arm_bot && source install/setup.bash

2. Choose trajectory mode
   ├─ Pre-generated:  --csv-path src/scripts/script_resources/path_XXX_trajectory.csv
   └─ From endpoint:  --q-end <q1_deg> <q2_deg> <q3_deg>
                         → trajectory saved to  logs/gen_traj_qend_*.csv

3. Run
   └─ Phase 1 (stabilisation): robot moves to trajectory start  (~10-30 s)
   └─ Phase 2 (trajectory):    CTC executes at 100 Hz
   └─ Output log:              logs/<traj_name>_ctc_log_<N>.csv

4. Analyse
   └─ python3 analyze_ctc_performance.py logs/<log>.csv --summary
   └─ python3 analyze_ctc_performance.py logs/<log>.csv --plots --output-dir plots/

5. Iterate
   └─ Adjust --kp / --kd via CLI flags (no rebuild needed)
   └─ Or edit defaults in torque_publisher_ctc.py (see table above)
```

---

## Joint Limits & Default Start

| Joint | Min (°) | Max (°) | Default start (°) |
|-------|---------|---------|-------------------|
| 1 | -160 | +160 | 0 |
| 2 | -225  | +45  | 45 |
| 3 | -45  | +225 | 135 |

---

## Control Law

```
τ_total = τ_model  +  τ_fb

τ_model = D(q)·q̈_des  +  C(q,q̇)  +  G(q)
τ_fb    = Kp·(q_des − q_act)  +  Kd·(q̇_des − q̇_act)  +  Ki·∫(q_des − q_act) dt

v       = q̈_des + τ_fb           ← virtual acceleration
τ_total = D(q)·v + C(q,q̇) + G(q) ← full CTC form
```

---

## Log CSV Columns

```
t
q_des_1/2/3    qd_des_1/2/3    qdd_des_1/2/3   ← desired trajectory
q_act_1/2/3    qd_act_1/2/3                    ← actual states
tau_model_1/2/3                                ← feedforward (inverse dynamics)
tau_fb_1/2/3                                   ← feedback correction
tau_total_1/2/3                                ← commanded torque
e_pos_1/2/3    e_vel_1/2/3                     ← tracking errors
```

---

## Typical Parameter Values

| Parameter | Joint 1 | Joint 2 | Joint 3 | Unit |
|-----------|---------|---------|---------|------|
| Kp | 30 | 100 | 50 | — |
| Kd | 4 | 10 | 5 | — |
| Ki | 0.2 | 1.0 | 0.8 | — |
| τ_max | 100 | 100 | 60 | N·m |
| Link mass | 0.01 | 17.4 | 4.8 | kg |

---

## Performance Targets

| Metric | Target | Typical |
|--------|--------|---------|
| RMS position error | < 0.5° | 0.3–0.5° |
| Max position error | < 1.5° | 0.8–1.2° |
| Stabilisation time | < 30 s | 10–20 s |
| Feedback contribution | 10–30% | 15–25% |
| Peak torque | < 100 Nm | 80–95 Nm |

---

## Analysis Commands

```bash
# Summary statistics
python3 arm_bot/src/scripts/analyze_ctc_performance.py \
  arm_bot/src/scripts/logs/<log>.csv --summary

# Generate all plots
python3 arm_bot/src/scripts/analyze_ctc_performance.py \
  arm_bot/src/scripts/logs/<log>.csv --plots --output-dir arm_bot/src/scripts/plots/

# Specific plots
python3 arm_bot/src/scripts/analyze_ctc_performance.py <log>.csv \
  --plot-trajectory traj.png \
  --plot-errors     err.png  \
  --plot-torques    tau.png  \
  --plot-velocity   vel.png
```

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Joint stuck during stabilisation | Increase `--skip-stabilization-threshold-deg` |
| Large steady-state error | Increase `--kp` |
| Oscillations | Decrease `--kp` or increase `--kd` |
| Slow decay / drift | Add `--ki 0.1 0.5 0.5` |
| Noisy velocity feedback | Decrease `--vel-filter-alpha` (e.g. 0.1) |
| Torque saturation warnings | Reduce `--torque-limits` or lower `--kp` |
| CSV not found | Use absolute path; verify with `ls` |
| Import error for `inverse_dynamics_model` | Ensure you are in `arm_bot/src/scripts/` |
| Log not created | Check `logs/` directory exists; check write permissions |
| `--q-end` ignored | Remove `--csv-path` (the two flags are mutually exclusive) |

---

## Tips

```bash
# Find the latest log
ls -lt ~/Desktop/FYP-Puma_560/arm_bot/src/scripts/logs/*.csv | head -5

# Quick error check on last log
python3 analyze_ctc_performance.py \
  $(ls -t ~/Desktop/FYP-Puma_560/arm_bot/src/scripts/logs/*_ctc_log_*.csv | head -1) --summary

# Compare multiple runs
for f in logs/*_ctc_log_*.csv; do
  echo "=== $f ===" && python3 analyze_ctc_performance.py "$f" --summary
done
```

---

**Quick Reference Card v2.0 — Last Updated: April 26, 2026**
