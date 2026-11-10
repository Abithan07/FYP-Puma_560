# Build and run workflows

Commands start in the repository root unless a block changes directory. Use a compatible ROS/Python environment; see [Publication notes](PUBLICATION_NOTES.md).

## Build only the main workspace

```bash
source /opt/ros/humble/setup.bash
cd arm_bot
colcon build --symlink-install --packages-select arm_bot
source install/setup.bash
```

`src/` directly contains the `arm_bot` ROS package. Do not build the entire repository as one workspace: `DNN_test/ROS2_Gazebo_sim/src/` also declares a package named `arm_bot`.

## Final demonstration

After building, from `arm_bot/`:

```bash
python3 src/scripts/Demo_controller/traj_gui.py
```

Select endpoint angles, an optional midpoint, and duration. RUN starts the simulator and trajectory execution; STOP ends it. Outputs are written under:

```text
arm_bot/src/scripts/Demo_controller/demo_trajectories/
  Angles/
  XYZ/
  Trajectories/
  JointStatesAll/
arm_bot/src/scripts/Demo_controller/demo_logs/
```

The generator also appends to `./test_traj.csv` relative to the invoking working directory. Existing tracking CSVs are retained; record the working directory when reproducing results.

To run without the GUI, launch the simulation in one terminal:

```bash
# From arm_bot/, after sourcing its install/setup.bash
ros2 launch arm_bot gazebo.launch.py
```

Then run the wrapper in a second sourced terminal:

```bash
# From arm_bot/
bash src/scripts/Demo_controller/automater.sh \
  --q-end '[120,-30,80]' --q-mid '[60,10,110]' --t-total 20
```

The wrapper generates, executes, and plots a trajectory. Quote bracket triples to preserve them as single shell arguments. Omit `--path-id` to allocate the next available trajectory ID. The wrapper's existing default duration is 18 seconds.

To generate CSVs alone without ROS:

```bash
python3 arm_bot/src/scripts/Demo_controller/puma_trajectory_generator.py \
  --path-id 1 --q-end '[120,-30,80]' --t-total 20 \
  --base-dir /tmp/puma-demo-trajectories
```

The wrapper executes only the newest CSV after generation, including when using `--csv` or `--num-paths`. Use the Python generator directly for dataset generation without simulation.

## DNN / CTC / PID comparison

From the repository root, with the ROS environment sourced:

```bash
bash arm_bot/all.sh --check
bash arm_bot/all.sh
```

The default is the original 50-path set in `arm_bot/src/scripts/Joint_states/J/`. That directory exists in the original working environment but its files are absent from the tracked repository. Supply the original data separately or use an explicit input directory:

```bash
bash arm_bot/all.sh --check --csv-dir /path/to/joint_states 641 647
bash arm_bot/all.sh --csv-dir /path/to/joint_states 641 647
```

The expected filename is `path_NNN_joint_states.csv`. Check mode validates input/model files and command availability without starting the simulator. Normal execution builds the workspace, opens GNOME Terminal windows, and waits for Enter after simulator startup. Confirm that all three effort controllers are active before continuing.

The runner keeps the original DNN → CTC → PID order and existing reset calls. Reset errors remain best-effort; verify reset behavior in the intended Gazebo installation when reproducing a comparison. `PUMA_CONTROLLER_TIMEOUT` changes the per-controller deadline, in seconds; its default is 900.

Console logs go to `arm_bot/run_logs/`. Detailed DNN/CTC CSVs go to `arm_bot/src/scripts/logs/`; PID CSVs go to `arm_bot/src/scripts/controller/pid_controller/logs/`. A separate triggered logger may produce additional raw-state logs. Existing logs remain tracked as research records; newly generated output is ignored.

To invoke an individual controller, first launch the simulator and then use a separate sourced terminal:

```bash
# From arm_bot/
ros2 run arm_bot torque_publisher_dnn.py --csv-path /absolute/path/to/input.csv
ros2 run arm_bot torque_publisher_ctc.py --csv-path /absolute/path/to/input.csv
python3 src/scripts/controller/pid_controller/pid_controller_trajectory.py \
  --trajectory-csv /absolute/path/to/input.csv
```

Run one controller at a time. Each command exposes additional options through `--help`. Pass `--csv-path` explicitly: some historical defaults reference data that is not supplied in a fresh clone.

## Asset location overrides

`project_paths.py` resolves the checkout from the source location, including ROS symlink installs. Optional environment variables are:

| Variable | Meaning |
| --- | --- |
| `PUMA_PROJECT_ROOT` | Checkout containing `arm_bot/src/package.xml`; required for the comparison nodes in a copied install outside the source tree |
| `PUMA_MODEL_DIR` | Directory for the matching DeLaN, GRU, and scaler files |
| `PUMA_LOG_DIR` | Main comparison DNN/CTC detailed CSV directory |
| `PUMA_CONTROLLER_TIMEOUT` | Comparison runner's per-controller deadline |

Demo outputs remain alongside the demo scripts. PID logs keep their existing separate location. Existing explicit CLI model/log arguments take precedence over their defaults. Source `install/setup.bash` before launching scripts, even when the script itself uses paths independent of the working directory.

## Historical data pipelines

`arm_bot/pipeline.sh N` and `src/scripts/dataset_generator.sh START [END [SUFFIX]]` retain the older feedforward dataset collection workflow. They use `src/scripts/Joint_states/` inputs. Zero-padding is now consistently three digits; nonzero controller exit codes stop the pipeline. See publication notes for limitations of the legacy postprocessor.

For the separate inverse-dynamics recording pipeline, read [id_pipeline/README.md](../id_pipeline/README.md). Its 1000 Hz reference settings do not match the main 100 Hz demo configuration. Match simulator, controller, CSV sampling, and post-processing rates deliberately before collecting new research data. `id_pipeline/config/pipeline.yaml` documents reference values and is not loaded by the scripts.

## Verification

```bash
python3 tools/check_repository.py
python3 -m unittest discover -s tests -v
```

For a build without modifying the existing workspace artifacts:

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/puma-check/log build \
  --base-paths arm_bot/src \
  --build-base /tmp/puma-check/build \
  --install-base /tmp/puma-check/install \
  --symlink-install --packages-select arm_bot \
  --cmake-args -DBUILD_TESTING=OFF
```

This checks packaging without running the historical lint suite or a physical simulation. Regression tests do not establish closed-loop tracking performance; reproduce the interactive demo and a controller comparison before claiming a validated release.
