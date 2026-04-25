# Inverse Dynamics Dataset Pipeline

Drives the PUMA 560 3-DOF arm in Gazebo using a position-tracking controller,
records the joint torques the physics engine computes, then post-processes them
into a clean `(q, dq, ddq) → τ` CSV dataset for neural-network training.

---

## Quick reference – all paths at a glance

| Item | Path |
|------|------|
| Pipeline directory | `~/Desktop/FYP-Puma_560/id_pipeline/` |
| Step 1 script | `~/Desktop/FYP-Puma_560/id_pipeline/scripts/1_trajectory_player.py` |
| Step 2 script | `~/Desktop/FYP-Puma_560/id_pipeline/scripts/2_build_dataset.py` |
| Input trajectories | `~/Desktop/FYP-Puma_560/Dataset/Trajectories/path_NNN_trajectory.csv` |
| Intermediate raw file | `/tmp/id_raw_NNN.csv` (auto, deleted between runs is fine) |
| Output dataset | `~/Desktop/FYP-Puma_560/Dataset/Trajectories/path_NNN_dataset.csv` |
| arm_bot workspace | `~/Desktop/FYP-Puma_560/arm_bot/` |
| Controller config | `~/Desktop/FYP-Puma_560/arm_bot/src/config/my_controllers.yaml` |

`NNN` = zero-padded file ID, e.g. `--id 1` → `001`, `--id 12` → `012`.

---

## How it works (physics summary)

Newton-Euler equation of motion:

```
τ_ID = M(q)·q̈  +  C(q,q̇)·q̇  +  G(q)
```

The controller sends `τ_cmd` to the physics engine as the applied joint torque.
By Newton-Euler, `τ_cmd = τ_ID` exactly — so recording `τ_cmd` during
near-perfect position tracking **is** recording the true inverse-dynamics torque.
No analytical model is used; Gazebo's physics engine computes everything.

---

## Prerequisites

```bash
# Python dependencies (run once)
pip install numpy scipy matplotlib
```

| Requirement | Notes |
|-------------|-------|
| ROS2 Humble | Standard install |
| arm_bot package | Must be built and sourced |
| numpy, scipy | Required |
| matplotlib | Optional (for `--plot`) |

---

## One-time setup – set controller to 1000 Hz

The trajectory CSVs have `dt = 0.001 s` (1000 Hz).
The controller must run at the same rate to process each CSV step exactly once.

**Edit** `~/Desktop/FYP-Puma_560/arm_bot/src/config/my_controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000   # change from 100 → 1000
```

**Rebuild:**
```bash
cd ~/Desktop/FYP-Puma_560/arm_bot
colcon build --packages-select arm_bot
```

> If you skip this, pass `--ctrl-rate 100` to step 1 and the pipeline will
> still work at 100 Hz by interpolating the 1000 Hz CSV down to 100 points/s.

---

## Run procedure

### Terminal 1 – launch Gazebo (keep open the entire time)

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/FYP-Puma_560/arm_bot/install/setup.bash

ros2 launch arm_bot robot_gui.launch.py
```

Wait until the terminal prints all four of these lines:
```
[spawner]: joint_state_broadcaster ... active
[spawner]: joint_1_controller ... active
[spawner]: joint_2_controller ... active
[spawner]: joint_3_controller ... active
```

---

### Terminal 2 – Step 1: play trajectory and record raw torques

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/FYP-Puma_560/arm_bot/install/setup.bash

python3 ~/Desktop/FYP-Puma_560/id_pipeline/scripts/1_trajectory_player.py --id 1
```

**What happens:**

| Phase | Description |
|-------|-------------|
| Stabilise | PID moves the arm to the first waypoint of the trajectory |
| Hold | Holds that pose for 0.5 s to damp out any oscillation |
| Execute | Tracks all waypoints in order; records data at every step |

**Input read:**
```
~/Desktop/FYP-Puma_560/Dataset/Trajectories/path_001_trajectory.csv
```

**Output written (intermediate – not the final dataset):**
```
/tmp/id_raw_001.csv
```
Columns in raw file:
`t | q1_ref q2_ref q3_ref | dq1_ref dq2_ref dq3_ref | q1_act q2_act q3_act | dq1_act dq2_act dq3_act | tau1_cmd tau2_cmd tau3_cmd | tau1_act tau2_act tau3_act | e1 e2 e3`

At the end the script prints a tracking quality report.
**If max error > 0.2°** the PD gains need tuning (see Troubleshooting).

---

### Terminal 2 – Step 2: post-process into final dataset (no Gazebo needed)

```bash
python3 ~/Desktop/FYP-Puma_560/id_pipeline/scripts/2_build_dataset.py --id 1 --plot
```

**Input read:**
```
/tmp/id_raw_001.csv                                                    ← from step 1
~/Desktop/FYP-Puma_560/Dataset/Trajectories/path_001_trajectory.csv   ← original trajectory
```

**Output written (final dataset):**
```
~/Desktop/FYP-Puma_560/Dataset/Trajectories/path_001_dataset.csv
```

---

## Output CSV format

All **original rows are preserved unchanged**.
Three `tau` rows are appended at the end.

```
t,     0.000, 0.001, 0.002, 0.003, …   (16001 columns for a 16 s trajectory)
dp1,   …                                ← joint 1 position  (rad)
dp2,   …                                ← joint 2 position  (rad)
dp3,   …                                ← joint 3 position  (rad)
dv1,   …                                ← joint 1 velocity  (rad/s)
dv2,   …                                ← joint 2 velocity  (rad/s)
dv3,   …                                ← joint 3 velocity  (rad/s)
da1,   …                                ← joint 1 acceleration (rad/s²)
da2,   …                                ← joint 2 acceleration (rad/s²)
da3,   …                                ← joint 3 acceleration (rad/s²)
tau1,  …   ← NEW: joint 1 torque (Nm) – true τ_ID from Gazebo physics
tau2,  …   ← NEW: joint 2 torque (Nm)
tau3,  …   ← NEW: joint 3 torque (Nm)
```

Total: **13 rows × 16001 columns** (for the existing 16 s trajectories).

---

## Batch – process all 5 trajectories

Run in **Terminal 2** with Gazebo already open in Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/FYP-Puma_560/arm_bot/install/setup.bash

for ID in 1 2 3 4 5; do
    echo ""
    echo "========================================"
    echo "  Processing trajectory $ID / 5"
    echo "========================================"

    # Step 1: execute in Gazebo and record raw torques
    python3 ~/Desktop/FYP-Puma_560/id_pipeline/scripts/1_trajectory_player.py --id $ID

    # Step 2: filter and assemble the final dataset CSV
    python3 ~/Desktop/FYP-Puma_560/id_pipeline/scripts/2_build_dataset.py --id $ID
done

echo "All done. Datasets saved to:"
ls ~/Desktop/FYP-Puma_560/Dataset/Trajectories/path_*_dataset.csv
```

> Between trajectories the arm resets automatically to the new trajectory's
> initial pose during the Stabilise phase — no manual reset needed.

---

## All command-line options

### `1_trajectory_player.py`

```
--id N            File ID → auto-resolves input and raw output paths
--input PATH      Override: explicit input trajectory CSV
--raw   PATH      Override: explicit raw output CSV path
--ctrl-rate HZ    Controller rate in Hz (default: 1000)
                  Must match update_rate in my_controllers.yaml
```

### `2_build_dataset.py`

```
--id N            File ID → auto-resolves raw, input, and output paths
--raw   PATH      Override: raw CSV from step 1
--input PATH      Override: original trajectory CSV
--out   PATH      Override: output dataset CSV
--cutoff HZ       Butterworth LPF cutoff frequency (default: 30 Hz)
--sg-window N     Savitzky-Golay window for q̈, must be odd (default: 21)
--sg-order  N     Savitzky-Golay polynomial order (default: 4)
--plot            Show diagnostic plots (tracking error, torques, q̈)
```

---

## Neural-network training

The output CSV rows map directly to the inverse-dynamics learning problem:

```
NN inputs  (9):  dp1, dp2, dp3, dv1, dv2, dv3, da1, da2, da3
NN outputs (3):  tau1, tau2, tau3
```

Preprocessing notes:
- Normalise each feature to zero mean / unit variance (fit scaler on training set only).
- Shuffle **between trajectories**, not within a single trajectory — shuffling within
  creates data leakage via time-correlated dynamics.
- Hold out at least one full trajectory for validation.

---

## Filtering reference

| Parameter | Default | Effect |
|-----------|---------|--------|
| `--cutoff` | 30 Hz | Butterworth LPF on τ — lower = smoother, higher = more detail |
| `--sg-window` | 21 | Savitzky-Golay window for q̈ — larger = smoother |
| `--sg-order` | 4 | Polynomial order — 4 works well for smooth trajectories |

For 1000 Hz data, 20–40 Hz cutoff is the useful range.
Physics-solver noise is typically above 50 Hz; robot dynamics are below 20 Hz.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| Script exits immediately: `Joint states not received` | Controllers not spawned yet | Wait for all 4 `active` lines in Gazebo terminal |
| `KeyError: dp1` or similar | Row label mismatch | Accepted labels: `t, dp1/q1, dp2/q2, dp3/q3, dv1/dq1, …` |
| Stabilise phase times out (> 30 s) | Initial pose very far from trajectory start | Robot will still proceed; check tracking error in report |
| Max tracking error > 0.5° | PD gains too low | Increase `_KP` in `1_trajectory_player.py` by 20 % |
| Robot oscillates | PD gains too high | Decrease `_KP` and `_KD` by 20 % |
| Torques are all exactly 0.0 | `update_rate` still 100, not 1000 | Rebuild arm_bot after editing `my_controllers.yaml` |
| Very noisy torques in output | `--cutoff` too high | Lower to 20 Hz |
| Output has fewer columns than input | Controller rate < CSV rate | Resampling is automatic; or set `--ctrl-rate` to match `update_rate` |
| `FileNotFoundError: /tmp/id_raw_001.csv` | Step 1 was not run first | Always run step 1 before step 2 for each ID |
