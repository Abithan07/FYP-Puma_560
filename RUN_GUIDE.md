# Final run guide

This guide covers the retained final scripts. Run commands from the repository root unless stated otherwise. The main simulation uses ROS 2 Humble, Gazebo Sim with `gz_ros2_control`, RViz, and a 100 Hz controller configuration.

## 1. Environment

Use the project's working Linux desktop environment with these ROS packages available:

```text
ros_gz_sim, ros_gz_bridge, gz_ros2_control
controller_manager, ros2_control, ros2_controllers
robot_state_publisher, xacro, rviz2, rclpy
```

The GUI needs PyQt5; inference needs JAX/Haiku, PyTorch, joblib, dill, and scikit-learn. Scientific processing and plotting need NumPy, SciPy, pandas, SymPy, Matplotlib, and Tk for the demo plotter. The comparison runner opens GNOME Terminal windows.

Python dependency groups are in `requirements/runtime.txt` and `requirements/dnn.txt`. They are starting constraints, not a verified environment lock. Install them into an isolated Python 3.10 environment compatible with ROS; avoid replacing packages in the working ROS installation.

**Existing environment issue:** the current default Python has NumPy 2.2.6 with Matplotlib 3.5.1, and plotting imports fail. Use a compatible environment before running the GUI/controller plots. The focused requirements constrain NumPy below 2 for the original binary stack. No host dependencies were changed during cleanup.

## 2. Final presentation demo

```bash
source /opt/ros/humble/setup.bash
bash arm_bot/run_demo.sh
```

Use a new terminal after organization. The old generated installation was removed; run without `--no-build` once to create the new installation.

The launcher builds `arm_bot`, sources its install, and opens the final GUI. In the GUI:

1. Select the endpoint joint angles and an optional midpoint.
2. Choose the motion duration.
3. Press **RUN**. The GUI starts Gazebo/RViz, waits for all three effort controllers to activate, generates a trajectory, and executes the demo DNN controller.
4. Review the resulting torque/energy plots. Use **STOP** to end the running processes.

After a successful build, skip rebuilding with:

```bash
bash arm_bot/run_demo.sh --no-build
```

The final demo code is in `arm_bot/src/scripts/demo/`. The two shell scripts inside that directory are required GUI helpers; they are part of the final workflow.

Demo outputs:

```text
data/trajectories/demo/Angles/
data/trajectories/demo/XYZ/
data/trajectories/demo/Trajectories/
data/trajectories/demo/JointStatesAll/
data/trajectories/demo/trajectory_tracking.csv
data/results/demo/                     # detailed demo controller CSVs
data/results/raw/                      # triggered joint-state logs
```

## 3. DNN / CTC / PID comparison

Check the original batch before launching:

```bash
source /opt/ros/humble/setup.bash
bash arm_bot/all.sh --check
```

By default the runner uses the original 50 path IDs from `data/trajectories/Joint_states/J/`. Those files were retained locally, but they were not tracked in the original Git repository. A fresh clone needs the original batch data or an explicitly supplied directory.

Run the batch:

```bash
bash arm_bot/all.sh
```

After Gazebo starts, wait until the three joint controllers are active, then press Enter in the launching terminal. For each CSV the runner executes **DNN → CTC → PID**, with the existing simulation reset calls between runs.

Run selected IDs from another dataset directory:

```bash
bash arm_bot/all.sh --check --csv-dir /absolute/path/to/joint_states 641 647
bash arm_bot/all.sh --csv-dir /absolute/path/to/joint_states 641 647
```

Expected filenames are `path_NNN_joint_states.csv`. `PUMA_CONTROLLER_TIMEOUT` sets the per-controller timeout in seconds; the default is 900. Reset requests retain their original best-effort behavior; check their effect on the intended Gazebo installation before reporting comparison results.

Comparison outputs:

```text
data/results/console/       # terminal logs and gazebo_current.log
data/results/comparison/    # DNN/CTC detailed CSVs; a copy of the PID CSV
data/results/pid/           # PID CSVs and plots
data/results/raw/           # triggered state logs
```

## 4. Manual build and individual controller runs

Build the main workspace:

```bash
source /opt/ros/humble/setup.bash
cd arm_bot
colcon build --symlink-install --packages-select arm_bot
source install/setup.bash
```

Launch the single retained simulator entry point:

```bash
# From arm_bot/, after sourcing its install/setup.bash
ros2 launch arm_bot gazebo.launch.py
```

In a second terminal, from `arm_bot/`:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# Choose one controller; do not run them simultaneously.
ros2 run arm_bot torque_publisher_dnn.py --csv-path /absolute/path/to/trajectory.csv
ros2 run arm_bot torque_publisher_ctc.py --csv-path /absolute/path/to/trajectory.csv
python3 src/scripts/controllers/pid_controller_trajectory.py \
  --trajectory-csv /absolute/path/to/trajectory.csv
```

Each controller exposes further options with `--help`. The CTC command requires `--csv-path` or its existing `--q-end` trajectory-generation option. The separate GUI demo DNN implementation is kept because its online inference behavior differs from the comparison controller's default precomputed feedforward.

## 5. Generate a trajectory without simulation

```bash
python3 arm_bot/src/scripts/demo/trajectory_generator.py \
  --path-id 1 --q-end '[120,-30,80]' --q-mid '[60,10,110]' \
  --t-total 20 --base-dir /tmp/puma-trajectories
```

Omit `--q-mid` for a direct motion. CLI joint angles are degrees; generated position/velocity/acceleration signals are radians, radians/second, and radians/second². Trajectory CSVs store one signal per row, with labels `t, dp1..dp3, dv1..dv3, da1..da3`.

To use the demo without its GUI, first start the simulator as above, then run in a sourced terminal:

```bash
# From the repository root
bash arm_bot/src/scripts/demo/run_trajectory.sh \
  --q-end '[120,-30,80]' --t-total 20
```

The wrapper generates a trajectory, executes it, and opens the plotter. It allocates the next path ID if none is provided. Its original batch/multiple-path generation options execute only the newest generated CSV; use the Python generator for generating data alone.

## 6. Standalone learned-model inference

The latest standalone inference version, formerly `inf7.py`, is retained as:

```bash
python3 tools/infer_torques.py 461
```

It reads `data/datasets/dnn/Trajectory/path_461_trajectories.csv`, loads the matching default models, and writes predictions/plots under `data/results/inference/`. Its residual clipping/ramp behavior remains distinct from the ROS publishers. Use `--output` to select another CSV output path. This workflow requires a compatible scientific/inference environment.

Default inference assets:

```text
models/fyp_jax_delan_50.jax
models/best_GRUResidual.pt
models/feature_scaler.pkl
```

## 7. Paths and data verification

| Environment variable | Purpose |
| --- | --- |
| `PUMA_PROJECT_ROOT` | Checkout root; needed by copied ROS installs outside the checkout |
| `PUMA_DATA_DIR` | Data root; defaults to this repository's `data/` |
| `PUMA_MODEL_DIR` | Checkpoint/scaler root; defaults to `models/` |
| `PUMA_LOG_DIR` | Main comparison DNN/CTC CSV directory |

Prefer absolute override paths. Source and symlink installs locate the checkout automatically. Data relocation is recorded in `data/MANIFEST.csv`, including original paths, new paths, hashes, and whether each artifact was originally tracked.

```bash
python3 tools/check_repository.py
python3 -m unittest discover -s tests -v
python3 tools/verify_data.py         # original tracked artifacts
python3 tools/verify_data.py --all   # also requires original local-only data
```

Existing data files were preserved; newly generated outputs remain ignored by Git. Original tracked data remains tracked at its new location. The preservation check compares to the cleanup baseline, so later deliberate changes to a recorded data file will be reported.

The original training/MATLAB/notebook material is in `research/` and is separate from the final run path. See [research notes](research/README.md) and [publication notes](docs/PUBLICATION_NOTES.md).
