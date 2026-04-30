# DNN Torque Controller — Quick Reference

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

# Terminal 2 — DNN controller
ros2 run arm_bot torque_publisher_dnn.py \
  --csv-path src/scripts/Joint_states/path_559_joint_states.csv

ros2 run arm_bot torque_publisher_dnn.py \
  --csv-path  /home/priyankan/Desktop/FYP-Puma_560/Joint_states_601_1120/path_670_joint_states.csv --mode 
  --mode pid-dnn

ros2 run arm_bot torque_publisher_dnn.py \
  --csv-path src/scripts/Joint_states/Joint_states_601_1120/path_643_joint_states.csv

# Terminal 3 — Analyse after run
python3 src/scripts/analyze_dnn_performance.py \
  src/scripts/logs/path_559_joint_states_pid_dnn_log_1.csv --plots

python3 src/scripts/analyze_dnn_performance.py \
  src/scripts/logs/path_461_joint_states_pid_dnn_log_1.csv --summary
```
---
### Control Mode Mapping

| Mode | Meaning |
|------|---------|
| `pid-only` | PID feedback only |
| `delan-only` | DeLaN feedforward only |
| `dnn` | DeLaN + GRU feedforward only |
| `pid-delan` | PID + DeLaN only |
| `pid-dnn` | PID + DeLaN + GRU |


## File & Directory Map

```
FYP-Puma_560/
│
├── DNN_test/                            ← DNN model files (do not move)
│   ├── fyp_jax_delan_50.jax            ← DeLaN physics model (JAX/Haiku)
│   ├── best_GRUResidual.pt             ← GRU residual model (PyTorch)
│   ├── feature_scaler.pkl              ← Training scaler (joblib)
│   └── inf6.py                         ← Batch inference reference (not used at runtime)
│
└── arm_bot/src/scripts/
    ├── torque_publisher_dnn.py          ← Main DNN ROS controller (online inference)
    ├── analyze_dnn_performance.py       ← Post-run analysis & plotting tool
    │
    ├── script_resources/                ← INPUT: trajectory CSVs (same as CTC)
    │   └── path_<id>_trajectory.csv
    │
    └── logs/                            ← OUTPUT: DNN control logs
      └── <traj_name>_<mode>_log_<N>.csv
    
    plots/                               ← OUTPUT: analysis plots
        └── <log_stem>_*.png
```

### Input trajectory CSV format (`script_resources/`)

Same row format as CTC:
```
t,   0.000, 0.010, ...
dp1, ...   ← desired joint 1 position (rad)
dp2, ...
dp3, ...
dv1, ...   ← desired joint 1 velocity (rad/s)
dv2, ...
dv3, ...
da1, ...   ← desired joint 1 acceleration (rad/s²)  [optional]
da2, ...
da3, ...
```

### Output log CSV columns (`logs/`)

```
t
q_des_1/2/3    qd_des_1/2/3    qdd_des_1/2/3   ← desired trajectory
q_act_1/2/3    qd_act_1/2/3                    ← actual states
tau_delan_1/2/3                                ← DeLaN physics baseline
tau_dnn_1/2/3                                  ← DeLaN + GRU (final feedforward)
tau_fb_1/2/3                                   ← PD+I correction
tau_total_1/2/3                                ← commanded torque
e_pos_1/2/3    e_vel_1/2/3                     ← tracking errors
gru_active                                     ← 0=warmup (DeLaN only), 1=GRU on
```

> **To change model paths** → edit `DNN_TEST_DIR` near the top of `torque_publisher_dnn.py`
> or use `--delan-model`, `--gru-model`, `--scaler`, `--mode` CLI flags.
>
> **To change output directory** → edit `LOGS_DIR` class variable in `torque_publisher_dnn.py`.
>
> RViz shows the desired trajectory first, then any saved logs that match the same trajectory, and finally the live current path.

---

## How Online Inference Works

```
Every 10 ms (100 Hz):

  q_des, qd_des, qdd_des  ← trajectory CSV[idx]
           ↓
      DeLaN (JAX JIT)      →  tau_delan   (physics, every step)
           ↓
  append [q, qd, qdd, tau_delan] to rolling 128-step buffer
           ↓
  steps 0–127  (warmup):   tau_dnn = tau_delan   ← GRU not yet active
  steps 128+:              tau_dnn = tau_delan + GRU(buffer)
           ↓
  tau_total = tau_dnn  +  Kp·e  +  Kd·ė  +  Ki·∫e
              ─────────   ─────────────────────────
              DNN feedfwd       small correction
           ↓
  publish to /joint_1/2/3_controller/commands
```

> The JAX JIT is warmed up with a dummy call at startup so the first real step is not slow.

---

## Quick Start

### For the first run the following first in each terminal
```bash
cd ~/Desktop/FYP-Puma_560/arm_bot && colcon build --packages-select arm_bot && source install/setup.bash
```
---

## All CLI Options (`torque_publisher_dnn.py`)

```
--csv-path PATH                  Trajectory CSV  [required]
--delan-model PATH               DeLaN .jax file (default: DNN_TEST_DIR/fyp_jax_delan_50.jax)
--gru-model PATH                 GRU .pt file    (default: DNN_TEST_DIR/best_GRUResidual.pt)
--scaler PATH                    Scaler .pkl     (default: DNN_TEST_DIR/feature_scaler.pkl)
--mode MODE                      Control mode: pid-only, delan-only, dnn, pid-delan, pid-dnn
--kp KP1 KP2 KP3                 Feedback Kp     (default: 5 20 10)
--kd KD1 KD2 KD3                 Feedback Kd     (default: 1  3  2)
--ki KI1 KI2 KI3                 Feedback Ki     (default: 0.05 0.2 0.1)
--torque-limits T1 T2 T3         Saturation Nm   (default: 100 100 60)
--vel-filter-alpha ALPHA         Velocity LP α   (default: 0.25)
--skip-stabilization-threshold-deg DEG  (default: 1.0)
--no-feedback                    DNN feedforward only, no PD+I correction
--no-model                       PID feedback only, no DNN model
--log-path PATH                  Override auto log path
```

> `--mode` is the cleanest way to test the five main cases. The older `--no-feedback` and `--no-model` switches still work, but `--mode` should be preferred for experiments.

---

## Where to Change Test Variables

### Option A — CLI flags (no code edits)

```bash
ros2 run arm_bot torque_publisher_dnn.py \
  --csv-path ...         \
  --kp 8 30 15           \   # raise if tracking is sluggish
  --kd 2 5 3             \   # raise if there are oscillations
  --ki 0.02 0.1 0.05     \   # small integral to remove slow drift
  --torque-limits 80 80 50   # lower for safety testing
```

### Option B — Hard-coded defaults in `torque_publisher_dnn.py`

| Variable | Location (approx. line) | Default |
|----------|------------------------|---------|
| **Feedback Kp** | `__init__` → `self.kp` (~line 43) | `[5, 20, 10]` |
| **Feedback Kd** | `__init__` → `self.kd` (~line 44) | `[1, 3, 2]` |
| **Feedback Ki** | `__init__` → `self.ki` (~line 45) | `[0.05, 0.2, 0.1]` |
| **Torque limits** | `__init__` → `self.torque_limits` (~line 47) | `[100, 100, 60]` Nm |
| **Integral anti-windup** | `compute_torques()` → `np.clip(...)` | `[0.3, 0.5, 0.5]` |
| **Stabilisation gains** | `stabilization_callback()` (~line 215) | `Kp=[50,200,150]` |
| **Output directory** | class var `LOGS_DIR` | `src/scripts/logs` |
| **Model directory** | module var `DNN_TEST_DIR` (~line 33) | `/…/DNN_test` |

### GRU / DeLaN model constants (match training exactly)

| Constant | Location | Value |
|----------|----------|-------|
| `SEQ_LEN` | `DNNInferenceEngine` class | 128 |
| `INPUT_DIM` | same | 12 |
| `HIDDEN_DIM` | same | 64 |
| `N_LAYERS` | same | 4 |

> Only change these if you retrain the GRU with different hyperparameters.

---

## Typical Workflow

```
1. Setup
   └─ cd ~/Desktop/FYP-Puma_560/arm_bot && source install/setup.bash

2. Run
   └─ Terminal 1: ros2 launch arm_bot gazebo.launch.py
   └─ Terminal 2: ros2 run arm_bot torque_publisher_dnn.py --csv-path ...
      ├─ Phase 1 (stabilisation): PID moves robot to trajectory start  (~10–30 s)
      └─ Phase 2 (trajectory)  : online DNN inference at 100 Hz
            steps   0–127 : DeLaN only  (GRU warming up)
            steps 128+    : DeLaN + GRU residual + PD+I feedback

3. Output
  └─ logs/<traj_name>_<mode>_log_<N>.csv   (auto-incremented, never overwritten)

4. Analyse
   └─ python3 analyze_dnn_performance.py logs/<log>.csv --summary
   └─ python3 analyze_dnn_performance.py logs/<log>.csv --plots
      → plots saved to  src/scripts/plots/<log_stem>_*.png

5. Compare with CTC
   └─ python3 analyze_dnn_performance.py logs/<dnn_log>.csv \
        --compare-ctc logs/<ctc_log>.csv --plots
```

---

## All `analyze_dnn_performance.py` Options

```
log_file                        DNN log CSV  [required]
--summary                       Print metrics to terminal
--plots                         Generate all plots → src/scripts/plots/
--output-dir DIR                Override plot output directory
--compare-ctc CTC_LOG           Overlay CTC tracking error for comparison
--plot-trajectory FILE          Desired vs actual position
--plot-errors     FILE          Position tracking errors
--plot-torques    FILE          DNN feedforward + feedback + total
--plot-delan-vs-dnn FILE        DeLaN baseline vs DeLaN+GRU (shows GRU effect)
--plot-gru-residual FILE        GRU correction signal per joint
--plot-velocity   FILE          Velocity tracking
--plot-warmup     FILE          Bar chart: warmup vs post-GRU RMS error
```

### Plot output filenames (auto when `--plots`)

| File | Shows |
|------|-------|
| `<stem>_trajectory.png` | Desired vs actual joint angles |
| `<stem>_errors.png` | Per-joint position errors |
| `<stem>_torques.png` | DNN feedfwd / feedback / total torque split |
| `<stem>_delan_vs_dnn.png` | DeLaN baseline vs DeLaN+GRU (GRU correction shaded) |
| `<stem>_gru_residual.png` | Raw GRU residual torque per joint |
| `<stem>_velocity.png` | Velocity tracking |
| `<stem>_warmup_effect.png` | Bar chart: RMS error before/after GRU activation |
| `<stem>_vs_ctc.png` | DNN vs CTC error overlay (requires `--compare-ctc`) |

---

## Common Analysis Examples

```bash
# Quick terminal summary
python3 analyze_dnn_performance.py logs/path_461_trajectory_pid_dnn_log_1.csv --summary

# All plots (auto-saved to src/scripts/plots/)
python3 analyze_dnn_performance.py logs/path_461_trajectory_pid_dnn_log_1.csv --plots

# All plots to custom folder
python3 analyze_dnn_performance.py logs/path_461_trajectory_pid_dnn_log_1.csv \
  --plots --output-dir ~/results/run1/

# Compare DNN vs CTC on same trajectory
python3 analyze_dnn_performance.py logs/path_461_trajectory_pid_dnn_log_1.csv \
  --compare-ctc logs/path_461_trajectory_ctc_log_1.csv --plots

# Latest log shortcut
python3 analyze_dnn_performance.py \
  $(ls -t logs/*_pid_dnn_log_*.csv | head -1) --summary
```

---

## Tuning Guide

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Large error during warmup (steps 0–127) | GRU not yet active — normal | Lower `--kp`/`--kd` if oscillations appear |
| Error drops after step 128 | GRU activating correctly | ✓ expected |
| Error stays high after step 128 | GRU not fitting this trajectory | Check trajectory is in training distribution |
| Oscillations at any phase | Feedback gains too high | Decrease `--kp` or `--kd` |
| Slow convergence / steady-state drift | Integral term needed | Add small `--ki 0.02 0.1 0.05` |
| High feedback fraction (> 40%) | DNN feedforward weak | Check model files; verify scaler matches training |
| Torque saturation warnings | Limits too tight or gains too high | Increase `--torque-limits` or reduce `--kp` |
| JAX import error | Missing dependencies | `pip install jax jaxlib dm-haiku dill` |
| PyTorch import error | Missing dependencies | `pip install torch joblib` |

---

## Joint Limits & Default Start

| Joint | Min (°) | Max (°) | Default start (°) |
|-------|---------|---------|-------------------|
| 1 | -100 | +100 | 0 |
| 2 | -15  | +45  | 45 |
| 3 | +65  | +205 | 135 |

---

## Dependencies

```bash
# ROS 2 side (already in environment)
# Python side — install if missing:
pip install torch joblib
pip install jax jaxlib
pip install dm-haiku dill
pip install numpy pandas matplotlib
```

---

**DNN Quick Reference v1.0 — Last Updated: April 26, 2026**
