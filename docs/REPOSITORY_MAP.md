# Retained source map

The latest final run workflows were selected from the `demo` history and their actual launch/subprocess dependencies. The former `Demo_controller` directory supplies the final presentation. The main DNN/CTC publishers and trajectory PID controller supply the final comparison. The numbered standalone inference series is represented only by its latest version, formerly `inf7.py`.

## User-facing entry points

| Script | Purpose |
| --- | --- |
| [`arm_bot/run_demo.sh`](../arm_bot/run_demo.sh) | Build/source the main workspace and start the final presentation GUI |
| [`arm_bot/setup_runtime.sh`](../arm_bot/setup_runtime.sh) | Create/check the compatible project Python overlay |
| [`arm_bot/all.sh`](../arm_bot/all.sh) | Check/run the original or selected DNN → CTC → PID comparison batch |
| [`tools/infer_torques.py`](../tools/infer_torques.py) | Latest standalone DeLaN + GRU inference |

Use the [final run guide](../RUN_GUIDE.md) for commands.

Use the [file-by-file code reference](CODE_REFERENCE.md) for each file's responsibilities, main functions, inputs/outputs, and relationship to the rest of the system, including research and test files.

The [clone and reproduction guide](REPRODUCIBILITY.md) explains the bundled dataset/model snapshot, environment requirements, and numerical reproduction check.

## Final demo and required helpers

| Source | Purpose |
| --- | --- |
| [`demo/traj_gui.py`](../arm_bot/src/scripts/demo/traj_gui.py) | GUI and simulation/controller orchestration |
| [`demo/run_trajectory.sh`](../arm_bot/src/scripts/demo/run_trajectory.sh) | Generate a trajectory, execute it, and plot the result |
| [`demo/launch_simulation.sh`](../arm_bot/src/scripts/demo/launch_simulation.sh) | Source the built workspace and launch the simulator |
| [`demo/trajectory_generator.py`](../arm_bot/src/scripts/demo/trajectory_generator.py) | Direct/waypoint trajectory generation with bounded velocity/acceleration |
| [`demo/torque_publisher_dnn.py`](../arm_bot/src/scripts/demo/torque_publisher_dnn.py) | Final demo online DeLaN/GRU plus feedback controller |
| [`demo/plot_results.py`](../arm_bot/src/scripts/demo/plot_results.py) | Demo torque and feedback-energy plots |

## Comparison, shared code, and analysis

| Source | Purpose |
| --- | --- |
| [`controllers/torque_publisher_dnn.py`](../arm_bot/src/scripts/controllers/torque_publisher_dnn.py) | Latest comparison DNN controller with precomputed feedforward support |
| [`controllers/torque_publisher_ctc.py`](../arm_bot/src/scripts/controllers/torque_publisher_ctc.py) | Analytical CTC controller |
| [`controllers/pid_controller_trajectory.py`](../arm_bot/src/scripts/controllers/pid_controller_trajectory.py) | Latest trajectory PID controller |
| [`shared/inverse_dynamics_model.py`](../arm_bot/src/scripts/shared/inverse_dynamics_model.py) | Symbolic/numerical analytical dynamics |
| [`shared/continuous_logger_triggered.py`](../arm_bot/src/scripts/shared/continuous_logger_triggered.py) | Single shared triggered logger for demo and comparisons |
| [`shared/project_paths.py`](../arm_bot/src/scripts/shared/project_paths.py) | Checkout, model, data, and output paths |
| [`shared/line_drawer.py`](../arm_bot/src/scripts/shared/line_drawer.py) | RViz trajectory drawing |
| [`analysis/analyze_dnn_performance.py`](../arm_bot/src/scripts/analysis/analyze_dnn_performance.py) | DNN controller log analysis |
| [`analysis/analyze_ctc_performance.py`](../arm_bot/src/scripts/analysis/analyze_ctc_performance.py) | CTC controller log analysis |
| [`scripts/terminal_job.sh`](../arm_bot/scripts/terminal_job.sh) | Internal terminal/process/status helper for `all.sh` |
| [`scripts/runtime_environment.sh`](../arm_bot/scripts/runtime_environment.sh) | Select one Python interpreter for GUI, controllers, logger, and plots |
| [`scripts/desktop_environment.sh`](../arm_bot/scripts/desktop_environment.sh) | Removes inherited Snap desktop-library paths before host GUI applications start |
| [`launch/gazebo.launch.py`](../arm_bot/src/launch/gazebo.launch.py) | Sole final simulation launch description |

## Research and checks

`research/delan/` retains the original dataset packing, DeLaN training, and black-box baseline training scripts. `research/matlab/` and `research/notebooks/` retain original research references byte-for-byte. They are not alternative final launchers.

`tools/check_runtime.py` exercises actual numerical/plotting imports and conversions before motion. `tools/check_repository.py` checks Python/shell/XML syntax. `tools/verify_data.py` checks preservation hashes. `tests/` covers the retained launchers, process reporting, asset discovery, trajectory CLI, and numerical properties.

Earlier torque publishers, one-/two-joint experiments, home/sweep utilities, numbered inference versions 1–6, the modular DNN experiment, old PID controller, old dataset pipelines, and the three extra simulator/controller packages were removed. The obsolete launch wrapper was replaced by the final full launch at `gazebo.launch.py`. [Removal list](REMOVED_FILES.txt) records retired tracked paths; commit `76212c2` retains their source in Git history.
