# Complete run guide

This guide describes the organized PUMA 560 codebase, from setup to controller selection, data generation, evaluation, and research references. It controls the first three joints of the modified PUMA arm in ROS 2/Gazebo simulation.

For an explanation of each source file, its main functions, inputs, outputs, and dependencies, read the separate [code reference](docs/CODE_REFERENCE.md).

**Working-directory rule:** every command below starts in the **repository root**, the directory containing this file, `arm_bot/`, `models/`, and `data/`. The research section explicitly changes directory. Repeat the terminal setup whenever you open a new terminal.

## Contents

1. [Quick start](#1-quick-start)
2. [Repository and entry points](#2-repository-and-entry-points)
3. [Install or repair the runtime](#3-install-or-repair-the-runtime)
4. [Build and prepare each terminal](#4-build-and-prepare-each-terminal)
5. [Use the presentation GUI](#5-use-the-presentation-gui)
6. [Choose and run a controller](#6-choose-and-run-a-controller)
7. [Generate trajectories](#7-generate-trajectories)
8. [Run automatic comparisons](#8-run-automatic-comparisons)
9. [Find and interpret outputs](#9-find-and-interpret-outputs)
10. [Plot and analyze results](#10-plot-and-analyze-results)
11. [Run standalone inference](#11-run-standalone-inference)
12. [Configure paths and controller options](#12-configure-paths-and-controller-options)
13. [Verify the code and data](#13-verify-the-code-and-data)
14. [Use the research references](#14-use-the-research-references)
15. [Troubleshoot and stop runs](#15-troubleshoot-and-stop-runs)

## 1. Quick start

### Run the final presentation

On the existing working checkout:

```bash
source /opt/ros/humble/setup.bash
bash arm_bot/run_demo.sh
```

This builds the main workspace and opens the GUI. After building once:

```bash
source /opt/ros/humble/setup.bash
bash arm_bot/run_demo.sh --no-build
```

The GUI uses **PID feedback + DeLaN + GRU** automatically. It currently has **no controller-selection dropdown**. To use PID, CTC, or another DNN mode, follow [controller selection](#6-choose-and-run-a-controller).

### Check or run all three comparison controllers

The original batch needs the original local datasets:

```bash
source /opt/ros/humble/setup.bash
bash arm_bot/all.sh --check
bash arm_bot/all.sh
```

`--check` checks inputs, default model files, commands, and the numerical runtime without starting simulation. The normal run executes **DNN → CTC → PID** for each selected path. It does not select just one controller.

On a fresh checkout, prepare the [runtime](#3-install-or-repair-the-runtime) first. If the original batch CSVs are unavailable, use the [self-contained batch example](#compare-a-newly-generated-trajectory).

## 2. Repository and entry points

| Location | What to use it for |
| --- | --- |
| `arm_bot/run_demo.sh` | Build/source the workspace and start the final GUI |
| `arm_bot/all.sh` | Original or selected controller-comparison batches |
| `arm_bot/setup_runtime.sh` | Create/repair the project Python runtime |
| `arm_bot/src/launch/gazebo.launch.py` | Start the simulator, robot, effort controllers, RViz, and path drawing |
| `arm_bot/src/scripts/demo/` | Final GUI, online demo controller, trajectory generator, and demo plotter |
| `arm_bot/src/scripts/controllers/` | Main comparison DNN, analytical CTC, and trajectory PID controllers |
| `arm_bot/src/scripts/shared/` | Dynamics model, path resolution, runtime checks, logger, and line drawing |
| `arm_bot/src/scripts/analysis/` | DNN/CTC tracking-error and torque analysis |
| `arm_bot/scripts/` | Shared environment and terminal/process helpers |
| `models/` | Default inference checkpoints/scaler and retained alternatives |
| `data/` | Trajectories, datasets, results, retained assets, and preservation manifest |
| `tools/` | Standalone inference and runtime/source/data checks |
| `research/` | DeLaN training references, original MATLAB files, and notebooks |
| `requirements/` | Runtime/inference requirements, compatibility overrides, and original training snapshot |
| `tests/` | Regression checks for the retained workflows |
| `docs/` | [Architecture](docs/ARCHITECTURE.md), [source map](docs/REPOSITORY_MAP.md), [work report](docs/CLEANUP_REPORT.md), and [publication notes](docs/PUBLICATION_NOTES.md) |

There is one retained ROS package, `arm_bot`, directly inside `arm_bot/src/`. Build that workspace. Intermediate simulator packages and run-script variants were removed; their source remains in Git history.

## 3. Install or repair the runtime

### System prerequisites

The working environment uses Ubuntu 22.04, ROS 2 Humble, and Python 3.10, with Gazebo Sim/Fortress integration and RViz. The main controller configuration targets **100 Hz**.

These ROS packages must already be installed or built in a compatible ROS installation:

```text
ros_gz_sim, ros_gz_bridge, gz_ros2_control
controller_manager, ros2_control, ros2_controllers
robot_state_publisher, xacro, rviz2, rclpy
```

Check the simulator integrations with:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix ros_gz_bridge
ros2 pkg prefix gz_ros2_control
```

Python setup does not install these simulator/control plugins. The GUI needs a desktop display and PyQt5; the demo plotter uses Tk. The comparison runner also needs GNOME Terminal. Python's `venv` and `pip` must be available.

### Existing working checkout

The compatible `.venv-ros/` environment has already been created here. To recreate or repair it:

```bash
source /opt/ros/humble/setup.bash
bash arm_bot/setup_runtime.sh
```

The script creates a Python environment with `--system-site-packages`, reuses the installed ROS/inference stack, and installs only the project overrides:

| Override | Reason |
| --- | --- |
| NumPy `1.26.4` | Compatible with the installed PyTorch 2.1 and Matplotlib 3.5 binaries |
| scikit-learn `1.6.1` | Matches the version recorded in the default saved feature scaler |

System/user-site NumPy and other global packages are unchanged. `.venv-ros/` is local and ignored by Git.

### Fresh Python setup

If the ROS/system prerequisites are present but the Python inference packages are missing, bootstrap the project environment before running the repair/check script:

```bash
source /opt/ros/humble/setup.bash
source arm_bot/scripts/desktop_environment.sh
python3 -m venv --system-site-packages .venv-ros
.venv-ros/bin/python3 -m pip install 'torch==2.1.0' \
  --index-url https://download.pytorch.org/whl/cpu
.venv-ros/bin/python3 -m pip install -r requirements/dnn.txt
bash arm_bot/setup_runtime.sh
```

This installs into the project environment, not into the system Python. The full requirement groups are starting constraints rather than a complete fresh-machine lockfile. The working host's two-package compatibility overlay has been tested; a fresh installation must pass the runtime check below.

### Runtime and Snap terminal checks

For manual commands, use this in each terminal:

```bash
source /opt/ros/humble/setup.bash
source arm_bot/scripts/desktop_environment.sh
source arm_bot/scripts/runtime_environment.sh
python3 tools/check_runtime.py
```

Expected: `Runtime OK`, a Python executable under `.venv-ros/bin/`, and successful NumPy/PyTorch/JAX/ROS/Matplotlib checks. This exercises actual tensor/array conversion and an offscreen render, not just package imports.

`runtime_environment.sh` selects the project Python and puts its `bin/` first on `PATH`. The shell launchers apply it automatically, and logger subprocesses use the controller's exact interpreter.

`desktop_environment.sh` removes inherited Snap GTK/Qt/library paths while preserving non-Snap ROS/system paths. This addresses the `GLIBC_PRIVATE`/`libpthread` startup error from Snap-installed VS Code terminals. The background is recorded in the [VS Code issue tracker](https://github.com/microsoft/vscode/issues/179274). The launchers apply this cleanup automatically too.

After changing the runtime, stop processes from the old attempt and relaunch them. Already-running Python processes keep their previously imported libraries.

## 4. Build and prepare each terminal

Build from the repository root:

```bash
source /opt/ros/humble/setup.bash
source arm_bot/scripts/desktop_environment.sh
source arm_bot/scripts/runtime_environment.sh
(cd arm_bot && colcon build --symlink-install --packages-select arm_bot)
```

For every terminal used for manual simulation/controller commands, run:

```bash
source /opt/ros/humble/setup.bash
source arm_bot/install/setup.bash
source arm_bot/scripts/desktop_environment.sh
source arm_bot/scripts/runtime_environment.sh
python3 tools/check_runtime.py --quiet
```

Use the same checkout and the same ROS installation in all terminals. Sourcing an install in one terminal does not configure another terminal. The GUI launcher handles build/source/runtime setup for its own children.

`--no-build` requires an existing `arm_bot/install/setup.bash`. Rebuild after changing package installation, robot descriptions, or launch/configuration files.

## 5. Use the presentation GUI

Launch with `bash arm_bot/run_demo.sh`, or `--no-build` after a successful build.

| GUI control | Meaning |
| --- | --- |
| Single S-Curve | Motion from the fixed start pose to the endpoint |
| Double S-Curve | Motion through the specified midpoint to the endpoint |
| End point | Desired final joint angles, in degrees |
| Mid point | Waypoint angles, used only for the double curve |
| Duration | Requested motion duration; GUI range 10–30 seconds, default 18 seconds |
| RUN TRAJECTORY | Start simulation, wait for controllers, generate/execute motion, and plot |
| STOP EXECUTION | Stop the run/simulation and reset the GUI for another run |
| CLEAR TAB | Clear the selected console display; it does not delete saved data |

The generator's start pose is `[0°, 45°, 135°]`. GUI input ranges are:

| Joint | Range |
| --- | --- |
| Q1 — base | −150° to 150° |
| Q2 — shoulder | −60° to 45° |
| Q3 — elbow | 0° to 225° |

For an example motion, choose endpoint `[120, -30, 80]`, optional midpoint `[60, 10, 110]`, and duration `20` seconds. The generator rounds endpoint/waypoint degrees to integers; integer inputs make this explicit. The GUI labels the profiles as S-curves; the generator implements minimum-jerk polynomial motion with its waypoint handling.

The run sequence is:

1. Gazebo and RViz start; output appears in **SIMULATION**.
2. The three `joint_1_controller`, `joint_2_controller`, and `joint_3_controller` activation messages are detected.
3. Generation, model loading, stabilization, tracking, and plotting output appears in **TRAJECTORY**.
4. The plotter opens after successful controller execution. Close its plot window to let the wrapper finish.
5. Press **STOP EXECUTION** before starting another run or switching to manual commands.

The three joint-controller indicators are low-level effort interfaces, not PID/CTC/DNN algorithm selectors. The GUI always starts the **demo online PID + DeLaN + GRU controller**. Use the next section to select another algorithm.

## 6. Choose and run a controller

### Prepare one example input

After runtime/terminal setup, generate a scratch example without starting ROS motion:

```bash
python3 arm_bot/src/scripts/demo/trajectory_generator.py \
  --path-id 1 --q-end '[120,-30,80]' --t-total 20 \
  --base-dir /tmp/puma-guide-example
```

The input used in the examples below is:

```text
/tmp/puma-guide-example/Trajectories/path_001_traj.csv
```

To use your existing GUI trajectory instead, replace it with a file such as `data/trajectories/demo/Trajectories/path_127_traj.csv`.

The `/tmp/` examples are scratch data and may be removed by the operating system. For a retained experiment, choose a new directory under `data/trajectories/` with `--base-dir` and update the input path accordingly.

### Terminal 1 — start the simulator

Run the [terminal setup](#4-build-and-prepare-each-terminal), then:

```bash
ros2 launch arm_bot gazebo.launch.py
```

Wait for all three effort controllers and the joint-state broadcaster to become active. You can inspect readiness from the other configured terminal:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

### Terminal 2 — select one controller

Run the same terminal setup, then set the input path:

```bash
CSV="/tmp/puma-guide-example/Trajectories/path_001_traj.csv"
```

Choose **one** of these commands:

```bash
# Standalone trajectory PID baseline
python3 arm_bot/src/scripts/controllers/pid_controller_trajectory.py \
  --trajectory-csv "$CSV"
```

```bash
# Analytical computed torque control
ros2 run arm_bot torque_publisher_ctc.py --csv-path "$CSV"
```

```bash
# Comparison DNN controller: PID feedback + DeLaN + GRU
ros2 run arm_bot torque_publisher_dnn.py \
  --csv-path "$CSV" --mode pid-dnn
```

Run one publisher at a time because they command the same three effort topics. The controllers handle their initial stabilization phases. For a manual comparison, stop and restart the simulator between algorithms so each starts from the same simulation conditions; wait for controller readiness again.

The standalone PID baseline includes an optional small reference-acceleration feedforward term by default. To disable it:

```bash
python3 arm_bot/src/scripts/controllers/pid_controller_trajectory.py \
  --trajectory-csv "$CSV" --no-feedforward
```

### DNN modes

Both DNN publishers expose these trajectory-phase modes:

| `--mode` | Learned feedforward | Tracking feedback |
| --- | --- | --- |
| `pid-dnn` | DeLaN + GRU | Enabled; default behavior when no mode/disable flags are supplied |
| `dnn` | DeLaN + GRU | Disabled |
| `pid-delan` | DeLaN only | Enabled |
| `delan-only` | DeLaN only | Disabled |
| `pid-only` | Disabled | Enabled inside the DNN publisher |

For example:

```bash
ros2 run arm_bot torque_publisher_dnn.py \
  --csv-path "$CSV" --mode delan-only
```

The initial stabilization phase still uses feedback. The table describes the tracking phase after stabilization. Prefer the explicit `--mode` option for these experiments; `--no-model` and `--no-feedback` are also available, but an explicit mode determines the selected combination.

The DNN publisher's `pid-only` mode is an ablation of that publisher, with its own gains/filtering/compensation behavior. It is **not equivalent** to the standalone trajectory PID baseline.

### Demo DNN versus comparison DNN

The two retained implementations differ:

| Implementation | How to run | Feedforward execution |
| --- | --- | --- |
| Demo | GUI, or the Python command below | Online inference |
| Comparison | `ros2 run arm_bot torque_publisher_dnn.py` | Precomputed desired-state rollout by default; also supports online operation |

To run the demo publisher manually, including its supported modes:

```bash
python3 arm_bot/src/scripts/demo/torque_publisher_dnn.py \
  --csv-path "$CSV" --mode pid-dnn
```

This command runs the controller only; it does not start Gazebo or automatically open the result plotter. Do not treat demo and comparison results as interchangeable without accounting for their different defaults and control features.

### Generate motion inside PID or CTC

A CSV is not required for these two alternative commands:

```bash
python3 arm_bot/src/scripts/controllers/pid_controller_trajectory.py \
  --target 120 -30 80 --start 0 45 135 --T 20
```

```bash
ros2 run arm_bot torque_publisher_ctc.py \
  --q-end 120 -30 80 --q-start 0 45 135
```

PID accepts `--T`; CTC computes its generated duration internally and saves the generated reference under the comparison log directory. `--q-end` and `--csv-path` are mutually exclusive for CTC. Use one shared CSV when the purpose is comparing algorithms on identical reference motion.

## 7. Generate trajectories

The standalone generator needs the Python runtime, but does not require a running simulator.

### Direct and waypoint motion

```bash
python3 arm_bot/src/scripts/demo/trajectory_generator.py \
  --path-id 1 --q-end '[120,-30,80]' --q-mid '[60,10,110]' \
  --t-total 20 --base-dir /tmp/puma-waypoint-example
```

Omit `--q-mid` for direct motion. Omit `--t-total` for the generator's computed duration. A requested duration shorter than its computed minimum is increased. Read the reported duration and CSV times rather than assuming the requested value was used unchanged.

Useful options:

| Option | Meaning |
| --- | --- |
| `--path-id N` | Output path ID; default filenames are zero-padded to three digits |
| `--q-end '[q1,q2,q3]'` | Endpoint in degrees |
| `--q-mid '[q1,q2,q3]'` | Optional waypoint in degrees |
| `--t-total SECONDS` | Requested total duration |
| `--base-dir DIR` | Output directory; default `data/trajectories/demo/` |
| `--dt 0.01` | Reference sampling interval; keep aligned with the current 100 Hz setup |
| `--v-max 2.0` | Velocity constraint in rad/s |
| `--a-max 7.0` | Acceleration constraint in rad/s² |
| `--num-paths N` | Repeat the condition under incrementing IDs; does not randomize endpoints |

Quote bracket triples so the shell passes each as one argument. Use a new output directory or unused path IDs when preserving earlier experiments: reusing the same directory/ID can replace generated files.

### Conditions CSV

Create a scratch conditions file:

```bash
cat > /tmp/puma-conditions.csv <<'CSV'
path_id,q1-end,q2-end,q3-end,q1-mid,q2-mid,q3-mid,t-total,num-paths
1,120,-30,80,,,,20,1
2,100,15,90,50,20,110,20,1
CSV
```

Generate both conditions:

```bash
python3 arm_bot/src/scripts/demo/trajectory_generator.py \
  --csv /tmp/puma-conditions.csv --base-dir /tmp/puma-batch-trajectories
```

`--csv` and `--path-id` are mutually exclusive. Blank midpoint cells skip the waypoint; the CSV loader also treats an all-zero midpoint triple as absent. Blank/zero duration selects automatic duration. Keep generated ID ranges distinct between conditions.

Each output directory contains:

```text
Angles/path_NNN_angles.csv
XYZ/path_NNN_xyz.csv
Trajectories/path_NNN_traj.csv
JointStatesAll/all_paths_0.csv
trajectory_tracking.csv
```

The first two CSVs are numeric sample tables: joint angles in radians and XYZ positions in meters in the generator's kinematic frame. `Trajectories/` contains the labeled row-wise references consumed by the controllers. The path record and tracking CSV keep generation metadata.

### Generate, execute, and plot without the GUI

With the simulator already running and the terminal configured:

```bash
bash arm_bot/src/scripts/demo/run_trajectory.sh \
  --q-end '[120,-30,80]' --q-mid '[60,10,110]' --t-total 20
```

This allocates the next path ID unless `--path-id` is supplied, generates the reference, runs the demo PID + DNN publisher, and opens the plotter. It uses 18 seconds by default when the wrapper's `--t-total` is omitted; the Python generator's omitted-duration behavior is automatic instead.

The wrapper's `--csv`/`--num-paths` options generate multiple references but **execute only the newest generated CSV**. Use the Python generator for generation alone, or the comparison runner for deliberate multi-path execution. The wrapper currently does not forward a `--mode` selection; use a controller command directly to choose its mode.

## 8. Run automatic comparisons

### Original local dataset

```bash
bash arm_bot/all.sh --check
bash arm_bot/all.sh
```

The default input directory is `data/trajectories/Joint_states/J/`; the default set contains the original 50 path IDs. These inputs are local-only and are not supplied by a fresh clone. Default model/scaler files remain in `models/`.

Run selected original IDs, or supply another input directory:

```bash
bash arm_bot/all.sh --check --csv-dir /absolute/path/to/joint_states 641 647
bash arm_bot/all.sh --csv-dir /absolute/path/to/joint_states 641 647
```

The required filename is **`path_NNN_joint_states.csv`**, even for a kinematics-only reference. The numeric row labels must still be correct. `all.sh` does not discover the demo generator's `_traj.csv` names automatically.

### Compare a newly generated trajectory

This example uses the scratch reference generated in section 6 and needs no original batch dataset:

```bash
BATCH_DIR="$(mktemp -d /tmp/puma-comparison.XXXXXXXX)"
cp /tmp/puma-guide-example/Trajectories/path_001_traj.csv \
  "$BATCH_DIR/path_001_joint_states.csv"
bash arm_bot/all.sh --check --csv-dir "$BATCH_DIR" 1
bash arm_bot/all.sh --csv-dir "$BATCH_DIR" 1
```

The copy changes the filename, not the CSV contents. For multiple paths, prepare matching filenames and pass their IDs, or use the original dataset.

The runner builds the workspace, opens GNOME Terminal windows, and waits for Enter after starting Gazebo. Confirm all three effort controllers are active before continuing. Each input runs through the main **PID + DNN**, **CTC**, and **standalone PID** controllers, in that order.

The existing simulation resets and marker-clearing calls remain best-effort; check their effect on the target Gazebo installation before reporting comparisons. `all.sh` has no single-controller option: use section 6 to run one algorithm.

Increase the per-controller deadline if a run legitimately needs longer than the default 900 seconds:

```bash
PUMA_CONTROLLER_TIMEOUT=1200 bash arm_bot/all.sh --csv-dir "$BATCH_DIR" 1
```

## 9. Find and interpret outputs

| Location | Contents |
| --- | --- |
| `data/trajectories/demo/` | GUI/default generator references and generation metadata |
| `data/results/demo/` | Detailed demo controller CSVs |
| `data/results/comparison/` | Main DNN/CTC CSVs and a copy of each standalone PID CSV |
| `data/results/pid/` | Standalone PID CSVs and tracking/torque/velocity figures |
| `data/results/raw/` | Triggered joint-state logger CSVs |
| `data/results/console/` | Batch terminal logs and `gazebo_current.log` |
| `data/results/analysis/dnn/` | Default DNN analysis output |
| `data/results/analysis/ctc/` | Default CTC analysis output |
| `data/results/inference/` | Standalone inference predictions and plots |

The controller prints the exact detailed CSV path when it opens/closes the file. For the first runs of `path_001_traj.csv`, typical names are:

```text
data/results/comparison/path_001_traj_pid_dnn_log_1.csv
data/results/comparison/path_001_traj_ctc_log_1.csv
data/results/pid/path_001_traj_pid_log_1.csv
data/results/raw/path_001_log_1.csv
```

Run counters are independent; repeated runs use later suffixes. Demo mode-specific CSVs use similar names under `data/results/demo/`. Batch console filenames are reused for a given controller/path, so archive those console logs separately if keeping several batches. Detailed controller CSVs normally allocate incremental filenames; an explicit DNN/CTC `--log-path` can overwrite that chosen file.

### Three different CSV contracts

| File type | Layout | Intended consumer |
| --- | --- | --- |
| Trajectory reference | One signal per row; first cell is `t`, `dp1..dp3`, `dv1..dv3`, or `da1..da3` | Motion controllers |
| Controller/raw log | One header row, one timestep per row | Plotting and analysis tools |
| Conditions CSV | One header row and one endpoint/waypoint condition per row | Trajectory generator `--csv` |

Reference example:

```csv
t,0.000,0.010,0.020
dp1,0.00000000,0.00000100,0.00000800
```

A complete reference also needs the other position, velocity, and acceleration rows. Kinematics use radians, rad/s, and rad/s²; time is seconds and effort is N·m. CLI endpoint/waypoint inputs use degrees.

Detailed DNN/CTC logs include desired/actual states, model/feedback/total torques, errors, and sensed effort. DNN logs also indicate GRU activity; the comparison DNN has timing fields. Raw logs record observed joint states and use different headers. Use detailed logs for the DNN/CTC analyzers, not the reference or raw logger CSV.

The GRU needs 128 feature samples. DeLaN alone is expected until its history is full; record how this startup interval is handled in evaluations. The two CSV layouts have different orientations and should not be transposed or relabeled casually.

### Standalone raw logging

The demo and comparison DNN/CTC publishers manage their own triggered logger. For an independent state-recording session, with simulation running and no other logger using these services:

```bash
CSV="/tmp/puma-guide-example/Trajectories/path_001_traj.csv"
ros2 run arm_bot continuous_logger_triggered.py --dataset-path "$CSV"
```

Here `--dataset-path` labels the output file; the logger does not execute the reference motion.

From another configured terminal:

```bash
ros2 service call /logger/start std_srvs/srv/Trigger '{}'
# Perform the intended observation, then stop the recording:
ros2 service call /logger/stop std_srvs/srv/Trigger '{}'
```

Stop the standalone logger process with Ctrl+C after recording. Do not start an extra copy while a publisher already owns the `/logger/start` and `/logger/stop` services.

## 10. Plot and analyze results

Run analysis in a configured terminal with the project runtime. It does not need a live simulator. Select the actual CSV paths printed by the controllers; the examples below assume the first manual runs from section 6.

```bash
DNN_LOG="$PWD/data/results/comparison/path_001_traj_pid_dnn_log_1.csv"
CTC_LOG="$PWD/data/results/comparison/path_001_traj_ctc_log_1.csv"
```

### Tracking and torque reports

Save the DNN summary and full plot set:

```bash
MPLBACKEND=Agg python3 arm_bot/src/scripts/analysis/analyze_dnn_performance.py \
  "$DNN_LOG" --summary --plots --output-dir "$PWD/data/results/analysis/manual_dnn"
```

Save the CTC summary and full plot set:

```bash
MPLBACKEND=Agg python3 arm_bot/src/scripts/analysis/analyze_ctc_performance.py \
  "$CTC_LOG" --summary --plots --output-dir "$PWD/data/results/analysis/manual_ctc"
```

Compare DNN and CTC logs from the same reference motion:

```bash
MPLBACKEND=Agg python3 arm_bot/src/scripts/analysis/analyze_dnn_performance.py \
  "$DNN_LOG" --compare-ctc "$CTC_LOG" --output-dir "$PWD/data/results/analysis/dnn_vs_ctc"
```

`MPLBACKEND=Agg` exports plots without a desktop display. DNN analysis includes tracking errors, torque split, DeLaN/GRU contributions, velocity, and warmup plots; sensed-versus-commanded effort plots are included when those columns are available.

Some analysis defaults have side effects: DNN `--summary` alone also saves a combined torque plot, and CTC's default output directory causes its plot set to be written even with `--summary`. To request only a CTC text summary, use `--summary --output-dir ''`. Read `--help` for individual plot options.

### Reopen the demo plot

```bash
python3 arm_bot/src/scripts/demo/plot_results.py \
  --log-csv-path "$PWD/data/results/demo/path_001_traj_pid_dnn_log_1.csv"
```

This opens the demo torque/feedback-energy figure using TkAgg by default. It is intended for a demo detailed log, not the raw joint-state CSV. The standalone PID controller saves its own figures automatically on shutdown.

A saved log or rendered figure does not prove a successful experiment. Check for inference/torque errors, premature stopping, incomplete sample counts, and tracking/timing problems before using results. The earlier zero-torque NumPy-failure logs remain failed runs; repairing the runtime does not repair their recorded values. State controller mode, gains, sampling/timing, reference, and evaluation interval when comparing results.

## 11. Run standalone inference

`tools/infer_torques.py` is the latest standalone inference implementation, formerly `inf7.py`. It predicts torques and saves plots without launching ROS motion:

```bash
python3 tools/infer_torques.py 461
```

The positional ID maps to this fixed input naming scheme:

```text
data/datasets/dnn/Trajectory/path_461_trajectories.csv
```

Default predictions are written under `data/results/inference/`, and figures under its `plots/` directory. The CLI accepts an ID, not an arbitrary `--csv-path`. Supply data in that naming/layout or use a ROS publisher for a different explicit reference.

Optional output and residual controls:

```bash
python3 tools/infer_torques.py 461 \
  --output "$PWD/data/results/inference/path_461_custom.csv" \
  --residual-ramp-sec 1.0 --residual-clip '0.08,0.15,0.05'
```

Use `MPLBACKEND=Agg` for an offline machine. The GRU window requires at least 128 samples. This standalone implementation retains its own residual clipping/ramp behavior; its outputs are not a substitute for measuring closed-loop tracking in simulation.

Default matching inference assets:

```text
models/fyp_jax_delan_50.jax
models/best_GRUResidual.pt
models/feature_scaler.pkl
```

Keep the trained checkpoint/scaler set together. Alternative retained checkpoints are not selected automatically.

## 12. Configure paths and controller options

### Environment variables

Set absolute overrides before launching. If using manual commands, set them before sourcing the runtime or importing Python modules.

| Variable | Effect |
| --- | --- |
| `PUMA_PYTHON` | Explicit compatible Python executable; otherwise prefer `.venv-ros/bin/python3` when present |
| `PUMA_BASE_PYTHON` | Base interpreter used by `setup_runtime.sh`; defaults to `/usr/bin/python3` |
| `PUMA_PROJECT_ROOT` | Checkout root; needed by copied ROS installs outside the checkout |
| `PUMA_DATA_DIR` | Data/output root; defaults to repository `data/` |
| `PUMA_MODEL_DIR` | Default inference checkpoint/scaler directory; defaults to `models/` |
| `PUMA_LOG_DIR` | Main comparison DNN/CTC detailed CSV directory and PID copy destination |
| `PUMA_CONTROLLER_TIMEOUT` | Batch per-controller deadline in seconds; default 900 |
| `MPLBACKEND` | Plotting backend, such as `Agg` for offline rendering |

For example:

```bash
PUMA_DATA_DIR=/absolute/path/to/experiment-data bash arm_bot/run_demo.sh
```

A new data root has no original batch inputs until you supply them; `--csv-dir` can select batch inputs independently. PID originals, demo logs, raw logs, and console logs remain in their corresponding locations below the selected data root. `PUMA_LOG_DIR` does not move every output type.

### Important controller flags

| Scope | Options and meaning |
| --- | --- |
| Both DNN publishers | `--mode`, `--kp`, `--kd`, `--ki`, `--torque-limits`, `--vel-filter-alpha`, `--skip-stabilization-threshold-deg` |
| DNN model files | `--delan-model`, `--gru-model`, `--scaler` select explicit checkpoint/scaler paths |
| Comparison DNN | `--model-frame desired` (default) supports precomputed rollout; `actual` uses sensed q/qd online |
| Comparison DNN timing | `--infer-stride`, `--marker-publish-every`, `--timing-warn-ms`, `--fb-lpf-alpha`, `--torque-rate-limit`; inference stride applies to online inference, not every cached rollout sample |
| CTC | `--dynamics-frame actual` (default) or `desired`; gains operate in acceleration space before inertia mapping |
| Standalone PID | `--dt`, `--kp`, `--kd`, `--ki`, `--torque-limits`, `--no-feedforward`, `--settle-tolerance-deg` |
| Explicit log path | DNN/CTC accept `--log-path`; use an absolute path with a directory component |

Inspect the controller's startup log for the actual gains/mode, and its constructor for numeric defaults; some retained numerical descriptions in CLI help are historical. CTC gains and torque-feedback PID gains do not represent the same feedback law.

Keep the current 0.01-second reference/controller setup aligned with `arm_bot/src/config/my_controllers.yaml` (`update_rate: 100`). Changing reference sampling alone does not configure a new experiment rate. Rebuild after editing the ROS configuration.

The launch file accepts marker frame/offset/line-width arguments and `use_sim_time`. List available arguments without starting motion:

```bash
ros2 launch arm_bot gazebo.launch.py --show-args
```

### Use the analytical model from Python

The shared inverse-dynamics helper can be used without starting ROS motion:

```bash
python3 - <<'PY'
import numpy as np
from arm_bot.src.scripts.shared.inverse_dynamics_model import compute_inverse_dynamics

q = np.deg2rad([0.0, 45.0, 135.0])
qd = np.zeros(3)
qdd = np.zeros(3)
tau, inertia, coriolis, gravity = compute_inverse_dynamics(q, qd, qdd)
print('Analytical torque:', tau)
PY
```

Inputs use radians and their time derivatives. This evaluates the analytical CTC model; it does not measure Gazebo effort or run the learned DeLaN/GRU models.

## 13. Verify the code and data

With the common terminal/runtime setup complete:

```bash
python3 tools/check_runtime.py
python3 tools/check_repository.py
python3 -m unittest discover -s tests -v
python3 tools/verify_data.py
```

| Check | What it establishes |
| --- | --- |
| `check_runtime.py` | Actual imports, tensor/NumPy conversion, JAX operation, and Matplotlib rendering |
| `check_repository.py` | Python/shell syntax and XML/Xacro/world parsing |
| `unittest` | Retained launcher behavior, environment selection, inference failure handling, and numerical/trajectory contracts |
| `verify_data.py` | Hashes of the originally tracked protected research artifacts |

Source ROS before running the full tests; the actual ROS callback tests are skipped if `rclpy` is unavailable. These checks do not start a simulated motion or establish closed-loop tracking performance.

To check original local-only artifacts as well:

```bash
python3 tools/verify_data.py --all
```

`data/MANIFEST.csv` records original/current paths, hashes, sizes, and tracked status for 4,106 protected artifacts. A fresh clone lacks the original local-only files, so `--all` is intended for the original working datasets. Deliberately modified training/reference records can differ from the baseline; inspect reported differences rather than restoring them blindly. New outputs that are absent from the manifest are not evaluated by this preservation check.

Originally tracked artifacts remain tracked after relocation. New data/results are ignored by Git unless deliberately added. See [data notes](data/README.md) and [publication notes](docs/PUBLICATION_NOTES.md) for reproducibility and release metadata.

## 14. Use the research references

These are supporting research sources, not alternative final launchers:

| Location | Contents |
| --- | --- |
| `research/delan/fyp_data_set.py` | Convert the original row-wise training CSVs into the packed pickle dataset |
| `research/delan/fyp_jax_delan.py` | Original structured Lagrangian model training/evaluation |
| `research/delan/fyp_jax_blackbox.py` | Original black-box baseline training/evaluation |
| `research/matlab/` | Original trajectory/dynamics MATLAB references |
| `research/notebooks/` | Original data-preparation/analysis notebooks, grouped by their former paths |

The dataset packer expects the original training layout: time, position triplet, velocity triplet, **torque triplet**, acceleration triplet, then optional dynamics groups. The ten-signal demo reference has no target torques and is not a drop-in training dataset. A controller log also has a different orientation/contract.

Training scripts use their current working directory's `data/` folder and top-level configuration such as `load_model`, model selection, dataset split, and hyperparameters. They can overwrite training checkpoints. Prepare a copy before experimenting; the following is a starting layout, not a verified complete retraining recipe:

```bash
PUMA_REPO="$PWD"
TRAINING_DIR="$(mktemp -d /tmp/puma-training.XXXXXXXX)"
cp -a research/delan/. "$TRAINING_DIR/"
python3 -m venv "$TRAINING_DIR/.venv"
source "$TRAINING_DIR/.venv/bin/activate"
python3 -m pip install -r "$PUMA_REPO/requirements/delan-original.txt"
cd "$TRAINING_DIR"
# Review data ordering, model loading, splits, and hyperparameters first.
python3 fyp_data_set.py
python3 fyp_jax_delan.py
# Alternative research baseline: python3 fyp_jax_blackbox.py
```

Use the training environment separately from `.venv-ros`; do not source the project runtime selector after activating it. The original training snapshot differs from the tested inference environment, and checkpoint loading may need version/configuration work. Keep the original `models/` inference set untouched while evaluating new training outputs.

Notebook/MATLAB internal paths remain historical and need review before execution. A complete GRU training recipe and deterministic training/evaluation provenance are not supplied by the retained code alone. See [research notes](research/README.md) for the scope of these references.

## 15. Troubleshoot and stop runs

| Symptom | Action |
| --- | --- |
| `Numpy is not available`, `_ARRAY_API`, or `numpy.core.multiarray failed to import` | Run `bash arm_bot/setup_runtime.sh`, stop the old processes, and relaunch. Check `python3 tools/check_runtime.py` in the selected project runtime. |
| Still selecting the old Python | Inspect `echo "$PUMA_PYTHON"`; unset an unintended override with `unset PUMA_PYTHON`, then source `arm_bot/scripts/runtime_environment.sh` again. |
| `GLIBC_PRIVATE` / Snap `libpthread` symbol lookup error | Use the launchers, or source `arm_bot/scripts/desktop_environment.sh` before manual GUI/ROS commands. |
| `No module named rclpy` | Source ROS Humble and use its Python 3.10-compatible runtime; the project overlay inherits system packages. |
| `arm_bot` not found, executable not found, or missing `install/setup.bash` | Rebuild `arm_bot`, then source that checkout's install in each terminal. Avoid `--no-build` on the first run. |
| GUI waits for controllers | Read **SIMULATION** output and check `ros2 control list_controllers`; confirm all three effort controllers and the broadcaster are active. |
| Missing original batch CSVs | Supply the original local datasets, or use the generated-reference batch example with `--csv-dir`. |
| Demo `_traj.csv` files are not found by `all.sh` | Prepare copies named `path_NNN_joint_states.csv`, as shown in section 8. |
| CSV/key/column errors | Check whether the command expects a trajectory, conditions file, detailed controller log, or raw state log. |
| `Torque computation failed` | The run has aborted. Inspect the first error, runtime check, and matching model/scaler paths. Keep the partial log for diagnosis; rerun before comparing performance. |
| GUI says the simulator is still running after a failure | Press **STOP EXECUTION** before a fresh attempt. |
| Plot window is open and GUI has not finished | Close the plot window so the wrapper can exit, then press **STOP EXECUTION** to reset. |
| No desktop display / Qt cannot connect to display | Run GUI/Gazebo/RViz in a desktop session; use `MPLBACKEND=Agg` for offline analysis. |

Stop according to the workflow:

- **GUI:** press **STOP EXECUTION**, then close the GUI if finished.
- **Comparison batch:** press Ctrl+C in the terminal running `all.sh`; its cleanup stops process groups it started.
- **Manual controllers:** stop the controller terminal with Ctrl+C, then stop the simulator terminal with Ctrl+C. Let log cleanup finish before inspecting or reusing files.
- **Standalone logger:** call `/logger/stop`, then Ctrl+C its process.

To inspect a command's options without starting motion:

```bash
bash arm_bot/run_demo.sh --help
bash arm_bot/all.sh --help
python3 arm_bot/src/scripts/demo/trajectory_generator.py --help
python3 arm_bot/src/scripts/demo/torque_publisher_dnn.py --help
ros2 run arm_bot torque_publisher_dnn.py --help
ros2 run arm_bot torque_publisher_ctc.py --help
python3 arm_bot/src/scripts/controllers/pid_controller_trajectory.py --help
python3 tools/infer_torques.py --help
```

For code review, continue with [architecture](docs/ARCHITECTURE.md), the [retained source map](docs/REPOSITORY_MAP.md), and the [cleanup report](docs/CLEANUP_REPORT.md). For publishing, review the existing [publication notes](docs/PUBLICATION_NOTES.md), including remaining license, attribution, and training-provenance metadata.
