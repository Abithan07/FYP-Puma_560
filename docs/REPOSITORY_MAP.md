# Repository map and source inventory

## Directory roles

| Location | Purpose | Review status |
| --- | --- | --- |
| `arm_bot/` | Main simulation workspace, final demo, and controller comparisons | Primary entry point |
| `arm_bot/src/scripts/Demo_controller/` | GUI, trajectory generation, online DNN controller, result plotting | Final demo |
| `arm_bot/src/scripts/controller/` | PID, modular DNN, and home-position controllers | Comparison and earlier controller variants |
| `DNN_test/` | Trained checkpoints, scalers, inference experiments, and training/data tools | Research support |
| `DNN_test/ROS2_Gazebo_sim/` | Earlier workspace also named `arm_bot` | Build separately |
| `arm_bot_pos_control/` | Position-control simulation experiment | Separate workspace |
| `arm_bot_new/` | Idealized three-joint simulation and validation tools | Separate workspace |
| `arm_controller/` | Standalone ROS Python PID package and original lint tests | Separate package |
| `id_pipeline/` | Recorded inverse-dynamics dataset pipeline | Separate 1000 Hz reference experiment |
| Root Python/MATLAB scripts | Trajectory generation, dynamics, and format conversion | Offline/research tools |
| `Dataset/`, `Test_data/` | Included sample/generated trajectories | Preserve data contracts |
| `arm_bot/run_logs/`, `arm_bot/src/scripts/piri_log/` | Recorded experiment outputs | Existing research records retained |
| Root Markdown/HTML/PDF files | Original technical explanations and guides | Historical documentation |
| `docs/` | Current architecture, workflows, publication notes, cleanup report | Advisor reading guide |
| `tests/`, `tools/` | Offline regression and source validation | Added during cleanup |
| `requirements/` | Focused Python dependency groups | Starting constraints; not a validated lock |

Existing files were kept in place so relative imports, ROS package lookup, meshes, and data references retain their structure. Earlier variants have not been deleted or combined.

## Recommended code reading order

1. [Main robot launch](../arm_bot/src/launch/robot_gui.launch.py), [robot geometry](../arm_bot/src/description/robot_core.xacro), and [controller configuration](../arm_bot/src/config/my_controllers.yaml).
2. [Final demo GUI](../arm_bot/src/scripts/Demo_controller/traj_gui.py), [wrapper](../arm_bot/src/scripts/Demo_controller/automater.sh), and [trajectory generator](../arm_bot/src/scripts/Demo_controller/puma_trajectory_generator.py).
3. [Demo DNN publisher](../arm_bot/src/scripts/Demo_controller/torque_publisher_dnn.py) and [comparison DNN publisher](../arm_bot/src/scripts/torque_publisher_dnn.py), noting their separate inference behavior.
4. [CTC publisher](../arm_bot/src/scripts/torque_publisher_ctc.py), [symbolic dynamics](../arm_bot/src/scripts/inverse_dynamics_model.py), and [trajectory PID controller](../arm_bot/src/scripts/controller/pid_controller/pid_controller_trajectory.py).
5. [Comparison runner](../arm_bot/all.sh), [triggered state logger](../arm_bot/src/scripts/continuous_logger_triggered.py), and the analysis tools listed below.
6. Training/data utilities and earlier experiments as needed for the associated research result.

## Python source inventory

This lists all 94 Python files in the baseline Git tree plus the new asset-location helper. Class and function names describe entry points; a listed file is not a claim that its full workflow has been executed during cleanup. Known legacy limitations are in [Publication notes](PUBLICATION_NOTES.md).

### Main simulation and controllers

| Source | Purpose / entry points |
| --- | --- |
| [arm_bot/src/launch/gazebo.launch.py](../arm_bot/src/launch/gazebo.launch.py) | Entry points: `generate_launch_description`. |
| [arm_bot/src/launch/line_drawer.py](../arm_bot/src/launch/line_drawer.py) | Entry points: `MarkerPublisher`, `main`. |
| [arm_bot/src/launch/robot_gui.launch.py](../arm_bot/src/launch/robot_gui.launch.py) | Entry points: `generate_launch_description`. |
| [arm_bot/src/scripts/Demo_controller/continuous_logger_triggered.py](../arm_bot/src/scripts/Demo_controller/continuous_logger_triggered.py) | Continuous Logger - Triggered Mode Entry points: `TriggeredLogger`, `main`. |
| [arm_bot/src/scripts/Demo_controller/plot_results.py](../arm_bot/src/scripts/Demo_controller/plot_results.py) | Entry points: `read_csv`, `plot_results`. |
| [arm_bot/src/scripts/Demo_controller/puma_trajectory_generator.py](../arm_bot/src/scripts/Demo_controller/puma_trajectory_generator.py) | PUMA-560 PATH + TRAJECTORY DATASET GENERATOR Entry points: `TrajConfig`, `dh_transform`, `forward_kinematics`, `min_jerk_profile`, `min_jerk_profile_bounded`, `compute_junction_velocity`, `split_time_by_arc`, `generate_two_segment_trajectory`, `load_path_record`, `is_unique_path`, `save_trajectory_csv`, `load_conditions_csv`, `run_one_trajectory`, `generate_trajectories`, `parse_bracket_triple`, `main`. |
| [arm_bot/src/scripts/Demo_controller/torque_publisher_dnn.py](../arm_bot/src/scripts/Demo_controller/torque_publisher_dnn.py) | DNN Torque Publisher — Online Inference Entry points: `DNNInferenceEngine`, `DNNTorquePublisher`, `main`. |
| [arm_bot/src/scripts/Demo_controller/traj_gui.py](../arm_bot/src/scripts/Demo_controller/traj_gui.py) | PUMA-560 Trajectory GUI Launcher  —  Light Theme Entry points: `strip_ansi`, `in_valid_range`, `range_label`, `graceful_stop`, `JointSpinBox`, `JointRow`, `SimMonitorThread`, `ProcThread`, `TrajectoryGUI`, `main`. |
| [arm_bot/src/scripts/analyze_ctc_performance.py](../arm_bot/src/scripts/analyze_ctc_performance.py) | Computed Torque Control Analysis Tool Entry points: `CTCAnalyzer`, `main`. |
| [arm_bot/src/scripts/analyze_dnn_performance.py](../arm_bot/src/scripts/analyze_dnn_performance.py) | DNN Torque Controller Analysis Tool Entry points: `DNNAnalyzer`, `main`. |
| [arm_bot/src/scripts/continuous_logger.py](../arm_bot/src/scripts/continuous_logger.py) | Continuous Logger - Standalone Script Entry points: `ContinuousLogger`, `main`. |
| [arm_bot/src/scripts/continuous_logger_triggered.py](../arm_bot/src/scripts/continuous_logger_triggered.py) | Continuous Logger - Triggered Mode Entry points: `TriggeredLogger`, `main`. |
| [arm_bot/src/scripts/continuous_logger_triggered_1dof.py](../arm_bot/src/scripts/continuous_logger_triggered_1dof.py) | Continuous Logger - Triggered Mode - 1DOF Version Entry points: `TriggeredLogger1DOF`, `main`. |
| [arm_bot/src/scripts/continuous_logger_triggered_2dof.py](../arm_bot/src/scripts/continuous_logger_triggered_2dof.py) | Continuous Logger - Triggered Mode - 2DOF Version Entry points: `TriggeredLogger2DOF`, `main`. |
| [arm_bot/src/scripts/controller/dnn_controller/dnn_controller.py](../arm_bot/src/scripts/controller/dnn_controller/dnn_controller.py) | ROS2 DNN torque controller for point-to-point motion. Entry points: `DNNTorqueController`, `main`. |
| [arm_bot/src/scripts/controller/dnn_controller/dnn_predictor.py](../arm_bot/src/scripts/controller/dnn_controller/dnn_predictor.py) | DNN predictor wrapper (DeLaN + GRU) — extracted from torque_publisher_dnn Entry points: `DNNInferenceEngine`. |
| [arm_bot/src/scripts/controller/dnn_controller/path_planner.py](../arm_bot/src/scripts/controller/dnn_controller/path_planner.py) | Entry points: `generate_min_jerk_trajectory`, `dh`. |
| [arm_bot/src/scripts/controller/home_controller.py](../arm_bot/src/scripts/controller/home_controller.py) | PID home-reset controller for the 3-DOF arm. Entry points: `HomeController`, `main`. |
| [arm_bot/src/scripts/controller/pid_controller/pid_controller.py](../arm_bot/src/scripts/controller/pid_controller/pid_controller.py) | PID-only ROS 2 torque controller. Entry points: `PIDTorqueController`, `main`. |
| [arm_bot/src/scripts/controller/pid_controller/pid_controller_trajectory.py](../arm_bot/src/scripts/controller/pid_controller/pid_controller_trajectory.py) | PID Trajectory-Tracking ROS 2 Torque Controller. Entry points: `min_jerk_profile`, `generate_trajectory`, `load_trajectory_from_csv`, `PIDTrajectoryController`, `main`. |
| [arm_bot/src/scripts/inverse_dynamics_model.py](../arm_bot/src/scripts/inverse_dynamics_model.py) | Exact inverse dynamics model for the 3-DOF PUMA arm. Entry points: `T`, `compute_D_matrix`, `compute_gravity`, `compute_coriolis`, `compute_inverse_dynamics`, `compute_gravity_analytical`, `compute_christoffel_symbols`. |
| [arm_bot/src/scripts/joint_position_Listener.py](../arm_bot/src/scripts/joint_position_Listener.py) | Entry points: `PositionListener`, `main`. |
| [arm_bot/src/scripts/plot_comparison.py](../arm_bot/src/scripts/plot_comparison.py) | Plot Comparison Script Entry points: `remove_outliers`, `load_dataset_file`, `load_continuous_log`, `plot_all_in_one`, `plot_all_comparisons`, `plot_torque_comparison`, `plot_position_comparison`, `main`. |
| [arm_bot/src/scripts/plot_with_plotly.py](../arm_bot/src/scripts/plot_with_plotly.py) | Plot Comparison Script Entry points: `load_dataset_file`, `remove_outliers`, `load_continuous_log`, `plot_all_comparisons_plotly`, `plot_all_comparisons`, `plot_torque_comparison`, `plot_position_comparison`, `main`. |
| [arm_bot/src/scripts/position_sequence_controller.py](../arm_bot/src/scripts/position_sequence_controller.py) | Position Sequence Controller - PID Control with CSV Position Input Entry points: `PositionSequenceController`, `main`. |
| [arm_bot/src/scripts/process_dataset.py](../arm_bot/src/scripts/process_dataset.py) | Legacy log transformation; retains historical velocity-difference acceleration calculation. |
| [arm_bot/src/scripts/project_paths.py](../arm_bot/src/scripts/project_paths.py) | Discover checkout, model, and comparison log directories. Entry points: `find_project_root`. |
| [arm_bot/src/scripts/simulate_gain_test.py](../arm_bot/src/scripts/simulate_gain_test.py) | Recompute feedback torques for different gains using logged errors and the Entry points: `run`, `main`. |
| [arm_bot/src/scripts/sinusoidal_torque_publisher.py](../arm_bot/src/scripts/sinusoidal_torque_publisher.py) | Sinusoidal Torque Publisher - 100Hz Operation with Real-time Logging Entry points: `SinusoidalTorquePublisher`, `main`. |
| [arm_bot/src/scripts/stabilize_zero_position.py](../arm_bot/src/scripts/stabilize_zero_position.py) | Entry points: `ZeroPositionStabilizer`, `main`. |
| [arm_bot/src/scripts/sweep_joints_limits.py](../arm_bot/src/scripts/sweep_joints_limits.py) | sweep_joints_limits.py Entry points: `SweepJointsLimits`, `main`. |
| [arm_bot/src/scripts/torque_publisher.py](../arm_bot/src/scripts/torque_publisher.py) | Optimized Torque Publisher - Zero Delay, 100Hz Operation Entry points: `TorquePublisher`, `main`. |
| [arm_bot/src/scripts/torque_publisher_.py](../arm_bot/src/scripts/torque_publisher_.py) | Optimized Torque Publisher - 100Hz Operation Entry points: `TorquePublisher`, `main`. |
| [arm_bot/src/scripts/torque_publisher_1dof.py](../arm_bot/src/scripts/torque_publisher_1dof.py) | Optimized Torque Publisher - 1 DOF Version - Zero Delay, 100Hz Operation Entry points: `TorquePublisher1DOF`, `main`. |
| [arm_bot/src/scripts/torque_publisher_26_4.py](../arm_bot/src/scripts/torque_publisher_26_4.py) | Optimized Torque Publisher - Zero Delay, 100Hz Operation Entry points: `TorquePublisher`, `main`. |
| [arm_bot/src/scripts/torque_publisher_2dof.py](../arm_bot/src/scripts/torque_publisher_2dof.py) | Optimized Torque Publisher - 2 DOF Version - Zero Delay, 100Hz Operation Entry points: `TorquePublisher2DOF`, `main`. |
| [arm_bot/src/scripts/torque_publisher_ctc.py](../arm_bot/src/scripts/torque_publisher_ctc.py) | Computed Torque Control Publisher - Full Dynamic Control + Feedback Entry points: `TorquePublisher`, `main`. |
| [arm_bot/src/scripts/torque_publisher_dnn.py](../arm_bot/src/scripts/torque_publisher_dnn.py) | DNN Torque Publisher — Trajectory-First Inference Entry points: `DNNInferenceEngine`, `DNNTorquePublisher`, `main`. |
| [arm_bot/src/scripts/torque_subscriber.py](../arm_bot/src/scripts/torque_subscriber.py) | Joint Torque Subscriber Entry points: `TorqueSubscriber`, `main`. |
| [arm_bot/src/scripts/trajectory_generator.py](../arm_bot/src/scripts/trajectory_generator.py) | PUMA-560 PATH + TRAJECTORY DATASET GENERATOR Entry points: `TrajConfig`, `dh_transform`, `forward_kinematics`, `min_jerk_profile`, `min_jerk_profile_bounded`, `compute_junction_velocity`, `split_time_by_arc`, `generate_two_segment_trajectory`, `load_path_record`, `is_unique_path`, `save_trajectory_csv`, `load_conditions_csv`, `run_one_trajectory`, `generate_trajectories`, `main`. |
| [arm_bot/src/scripts/transpose_csv.py](../arm_bot/src/scripts/transpose_csv.py) | Transpose CSV files. Entry points: `transpose_rows`, `transpose_file`, `process_path`, `main`. |

### Learning and inference experiments

| Source | Purpose / entry points |
| --- | --- |
| [DNN_test/DeLaN/fyp_data_set.py](../DNN_test/DeLaN/fyp_data_set.py) | Pack row-signal CSV datasets into a training dictionary/pickle. Entry points: `csv_to_custom_dict`, `combine_csv_dicts`, `save_as_pickle`. |
| [DNN_test/DeLaN/fyp_jax_blackbox.py](../DNN_test/DeLaN/fyp_jax_blackbox.py) | Black-box neural inverse-dynamics training experiment. Entry points: `dynamics_network`, `dynamics_model`, `loss_fn`, `ReplayMemory`, `load_dataset`, `compute_error_dict`, `add_errors_to_csv`, `update_fn`, `get_limits`. |
| [DNN_test/DeLaN/fyp_jax_delan.py](../DNN_test/DeLaN/fyp_jax_delan.py) | Structured Lagrangian neural inverse-dynamics training experiment. Entry points: `load_dataset`, `ReplayMemory`, `mass_matrix_fn`, `kinetic_energy_fn`, `potential_energy_fn`, `structured_lagrangian_fn`, `blackbox_lagrangian_fn`, `dynamics_model`, `inverse_loss_fn`, `update_fn`, `get_limits`. |
| [DNN_test/ROS2_Gazebo_sim/src/launch/robot_gui.launch.py](../DNN_test/ROS2_Gazebo_sim/src/launch/robot_gui.launch.py) | Entry points: `generate_launch_description`. |
| [DNN_test/ROS2_Gazebo_sim/src/scripts/continuous_logger.py](../DNN_test/ROS2_Gazebo_sim/src/scripts/continuous_logger.py) | Continuous Logger - Standalone Script Entry points: `ContinuousLogger`, `main`. |
| [DNN_test/ROS2_Gazebo_sim/src/scripts/continuous_logger_triggered.py](../DNN_test/ROS2_Gazebo_sim/src/scripts/continuous_logger_triggered.py) | Continuous Logger - Triggered Mode Entry points: `TriggeredLogger`, `main`. |
| [DNN_test/ROS2_Gazebo_sim/src/scripts/joint_position_Listener.py](../DNN_test/ROS2_Gazebo_sim/src/scripts/joint_position_Listener.py) | Entry points: `PositionListener`, `main`. |
| [DNN_test/ROS2_Gazebo_sim/src/scripts/plot_comparison.py](../DNN_test/ROS2_Gazebo_sim/src/scripts/plot_comparison.py) | Plot Comparison Script Entry points: `load_dataset_file`, `remove_outliers`, `load_continuous_log`, `plot_all_comparisons`, `plot_torque_comparison`, `plot_position_comparison`, `main`. |
| [DNN_test/ROS2_Gazebo_sim/src/scripts/plot_with_plotly.py](../DNN_test/ROS2_Gazebo_sim/src/scripts/plot_with_plotly.py) | Plot Comparison Script Entry points: `load_dataset_file`, `remove_outliers`, `load_continuous_log`, `plot_all_comparisons_plotly`, `plot_all_comparisons`, `plot_torque_comparison`, `plot_position_comparison`, `main`. |
| [DNN_test/ROS2_Gazebo_sim/src/scripts/position_sequence_controller.py](../DNN_test/ROS2_Gazebo_sim/src/scripts/position_sequence_controller.py) | Position Sequence Controller - PID Control with CSV Position Input Entry points: `PositionSequenceController`, `main`. |
| [DNN_test/ROS2_Gazebo_sim/src/scripts/sinusoidal_torque_publisher.py](../DNN_test/ROS2_Gazebo_sim/src/scripts/sinusoidal_torque_publisher.py) | Sinusoidal Torque Publisher - 100Hz Operation with Real-time Logging Entry points: `SinusoidalTorquePublisher`, `main`. |
| [DNN_test/ROS2_Gazebo_sim/src/scripts/stabilize_zero_position.py](../DNN_test/ROS2_Gazebo_sim/src/scripts/stabilize_zero_position.py) | Entry points: `ZeroPositionStabilizer`, `main`. |
| [DNN_test/ROS2_Gazebo_sim/src/scripts/sweep_joints_limits.py](../DNN_test/ROS2_Gazebo_sim/src/scripts/sweep_joints_limits.py) | sweep_joints_limits.py Entry points: `SweepJointsLimits`, `main`. |
| [DNN_test/ROS2_Gazebo_sim/src/scripts/torque_publisher.py](../DNN_test/ROS2_Gazebo_sim/src/scripts/torque_publisher.py) | Optimized Torque Publisher - Zero Delay, 100Hz Operation Entry points: `TorquePublisher`, `main`. |
| [DNN_test/datagen.py](../DNN_test/datagen.py) | Generate kinematic datasets for standalone inference experiments. Entry points: `dh_numpy`. |
| [DNN_test/dep.py](../DNN_test/dep.py) | Check imports required by the standalone inference scripts. Entry points: `check_dependencies`. |
| [DNN_test/inf1.py](../DNN_test/inf1.py) | Standalone torque-inference experiment. Entry points: `mass_matrix_fn`, `kinetic_energy_fn`, `potential_energy_fn`, `structured_lagrangian_fn`, `dynamics_model`, `GRUResidual`. |
| [DNN_test/inf2.py](../DNN_test/inf2.py) | Standalone torque-inference experiment. Entry points: `mass_matrix_fn`, `kinetic_energy_fn`, `potential_energy_fn`, `structured_lagrangian_fn`, `dynamics_model`, `GRUResidual`. |
| [DNN_test/inf3.py](../DNN_test/inf3.py) | Standalone torque-inference experiment. Entry points: `mass_matrix_fn`, `kinetic_energy_fn`, `potential_energy_fn`, `structured_lagrangian_fn`, `dynamics_model`, `GRUResidual`. |
| [DNN_test/inf4.py](../DNN_test/inf4.py) | Standalone DeLaN + GRU inference experiment. Entry points: `mass_matrix_fn`, `kinetic_energy_fn`, `potential_energy_fn`, `structured_lagrangian_fn`, `dynamics_model`, `GRUResidual`. |
| [DNN_test/inf5.py](../DNN_test/inf5.py) | Standalone DeLaN + GRU inference experiment. Entry points: `mass_matrix_fn`, `kinetic_energy_fn`, `potential_energy_fn`, `structured_lagrangian_fn`, `dynamics_model`, `GRUResidual`. |
| [DNN_test/inf6.py](../DNN_test/inf6.py) | Standalone DeLaN + GRU inference experiment. Entry points: `parse_args`, `mass_matrix_fn`, `kinetic_energy_fn`, `potential_energy_fn`, `structured_lagrangian_fn`, `dynamics_model`, `GRUResidual`. |
| [DNN_test/inf7.py](../DNN_test/inf7.py) | Standalone DeLaN + GRU inference experiment. Entry points: `parse_args`, `parse_clip_values`, `mass_matrix_fn`, `kinetic_energy_fn`, `potential_energy_fn`, `structured_lagrangian_fn`, `dynamics_model`, `GRUResidual`. |
| [DNN_test/minmax.py](../DNN_test/minmax.py) | Inspect signal ranges in a configured CSV. |
| [DNN_test/plot.py](../DNN_test/plot.py) | Plot utility for row-wise joint-state CSV files. Entry points: `parse_args`, `resolve_csv_path`, `normalize_name`, `load_row_wise_csv`, `get_series`, `plot_group`, `main`. |

### Idealized simulator experiment

| Source | Purpose / entry points |
| --- | --- |
| [arm_bot_new/src/launch/sim.launch.py](../arm_bot_new/src/launch/sim.launch.py) | sim.launch.py — Launches the ideal PUMA 560 Gazebo simulation. Entry points: `generate_launch_description`. |
| [arm_bot_new/src/scripts/torque_publisher.py](../arm_bot_new/src/scripts/torque_publisher.py) | torque_publisher.py Entry points: `_rx`, `_rz`, `_mm`, `_mv`, `_cmp`, `fk_tip`, `TorquePublisher`, `main`. |
| [arm_bot_new/src/scripts/triggered_logger.py](../arm_bot_new/src/scripts/triggered_logger.py) | triggered_logger.py Entry points: `TriggeredLogger`, `main`. |
| [arm_bot_new/src/scripts/validate.py](../arm_bot_new/src/scripts/validate.py) | validate.py Entry points: `load_desired`, `load_actual`, `main`. |

### Position-control simulator experiment

| Source | Purpose / entry points |
| --- | --- |
| [arm_bot_pos_control/src/launch/line_drawer.py](../arm_bot_pos_control/src/launch/line_drawer.py) | Entry points: `MarkerPublisher`, `main`. |
| [arm_bot_pos_control/src/launch/robot_gui.launch.py](../arm_bot_pos_control/src/launch/robot_gui.launch.py) | Entry points: `generate_launch_description`. |
| [arm_bot_pos_control/src/scripts/continuous_logger_triggered.py](../arm_bot_pos_control/src/scripts/continuous_logger_triggered.py) | Continuous Logger - Triggered Mode Entry points: `TriggeredLogger`, `main`. |
| [arm_bot_pos_control/src/scripts/plot_comparison.py](../arm_bot_pos_control/src/scripts/plot_comparison.py) | Plot Comparison Script Entry points: `remove_outliers`, `load_dataset_file`, `load_continuous_log`, `plot_all_in_one`, `plot_all_comparisons`, `plot_torque_comparison`, `plot_position_comparison`, `main`. |
| [arm_bot_pos_control/src/scripts/position_publisher.py](../arm_bot_pos_control/src/scripts/position_publisher.py) | Optimized Torque Publisher - Zero Delay, 100Hz Operation Entry points: `TorquePublisher`, `main`. |
| [arm_bot_pos_control/src/scripts/torque_publisher.py](../arm_bot_pos_control/src/scripts/torque_publisher.py) | Optimized Torque Publisher - Zero Delay, 100Hz Operation Entry points: `TorquePublisher`, `main`. |

### Standalone ROS Python package

| Source | Purpose / entry points |
| --- | --- |
| [arm_controller/arm_controller/__init__.py](../arm_controller/arm_controller/__init__.py) | Package initialization. |
| [arm_controller/arm_controller/pid_node.py](../arm_controller/arm_controller/pid_node.py) | Standalone ROS Python PID node. Entry points: `PidControllerNode`, `main`. |
| [arm_controller/setup.py](../arm_controller/setup.py) | ROS Python package installation metadata. |
| [arm_controller/test/test_copyright.py](../arm_controller/test/test_copyright.py) | Entry points: `test_copyright`. |
| [arm_controller/test/test_flake8.py](../arm_controller/test/test_flake8.py) | Entry points: `test_flake8`. |
| [arm_controller/test/test_pep257.py](../arm_controller/test/test_pep257.py) | Entry points: `test_pep257`. |

### Inverse-dynamics recording pipeline

| Source | Purpose / entry points |
| --- | --- |
| [id_pipeline/scripts/1_trajectory_player.py](../id_pipeline/scripts/1_trajectory_player.py) | Inverse Dynamics Pipeline – Step 1: Trajectory Player Entry points: `load_input_csv`, `save_raw_csv`, `IDTrajectoryPlayer`, `_resolve_paths`, `main`. |
| [id_pipeline/scripts/2_build_dataset.py](../id_pipeline/scripts/2_build_dataset.py) | Inverse Dynamics Pipeline – Step 2: Build Dataset Entry points: `load_col_csv`, `load_row_csv`, `write_row_csv`, `savgol_ddq`, `butterworth_lpf`, `print_quality_report`, `optional_plot`, `_resolve_paths`, `main`. |
| [id_pipeline/utils/generate_test_input.py](../id_pipeline/utils/generate_test_input.py) | Generate a synthetic test input CSV for the ID pipeline. Entry points: `main`. |

### Root offline tools

| Source | Purpose / entry points |
| --- | --- |
| [batch_processor.py](../batch_processor.py) | Batch processor for PUMA-560 inverse dynamics calculations Entry points: `main`. |
| [endpoint_generator.py](../endpoint_generator.py) | Sample endpoint conditions and write CSV configuration. Entry points: `get_random_j1`, `get_random_j2`, `get_random_j3`, `get_random_t`. |
| [gen_traj.py](../gen_traj.py) | Earlier minimum-jerk trajectory generation utility. Entry points: `min_jerk`, `generate_trajectories`. |
| [inv_dyn.py](../inv_dyn.py) | Symbolic inverse-dynamics dataset generation. Entry points: `T`, `process_file`, `main`. |
| [inv_dyn1.py](../inv_dyn1.py) | Alternative symbolic inverse-dynamics dataset generation. Entry points: `T`, `process_file`, `main`. |
| [inverse_dynamics_fast.py](../inverse_dynamics_fast.py) | PUMA-560 INVERSE DYNAMICS CALCULATOR (Fast) Entry points: `InverseDynamicsCalculator`, `main`. |
| [trajectory_generator copy.py](../trajectory_generator%20copy.py) | PUMA-560 PATH + TRAJECTORY DATASET GENERATOR Entry points: `TrajConfig`, `dh_transform`, `min_jerk_profile`, `forward_kinematics`, `load_path_record`, `is_unique_path`, `generate_trajectories`, `main`. |
| [trajectory_generator.py](../trajectory_generator.py) | PUMA-560 PATH + TRAJECTORY DATASET GENERATOR Entry points: `TrajConfig`, `dh_transform`, `min_jerk_profile`, `forward_kinematics`, `load_path_record`, `is_unique_path`, `generate_trajectories`, `main`. |
| [trajectory_generator_ab.py](../trajectory_generator_ab.py) | PUMA-560 trajectory dataset generator. Entry points: `TrajConfig`, `sample_integer_from_intervals`, `min_jerk_profile`, `load_path_record`, `is_unique_path`, `generate_trajectories`, `main`. |
| [transpose_trajectory_csvs.py](../transpose_trajectory_csvs.py) | Rotate trajectory CSV files into labeled signal rows. Entry points: `normalize_csv_file`, `main`. |

## Shell entry points

| Script | Purpose |
| --- | --- |
| `arm_bot/all.sh` | Original comparison batch plus explicit IDs/input directory and preflight mode |
| `arm_bot/scripts/terminal_job.sh` | Internal process-group and exit-status helper for comparison windows |
| `arm_bot/pipeline.sh` | Earlier data collection and postprocessing workflow |
| `arm_bot/src/scripts/dataset_generator.sh` | Legacy feedforward runs and comparison plotting |
| `arm_bot/src/scripts/dataset_generator_1dof.sh` | Earlier one-joint dataset experiment |
| `arm_bot/src/scripts/dataset_generator_2dof.sh` | Earlier two-joint dataset experiment |
| `arm_bot/src/scripts/Demo_controller/automater.sh` | Generate, execute the newest trajectory, and plot |
| `arm_bot/src/scripts/Demo_controller/gazebo_launch.sh` | Source the main workspace and launch the simulator |
| `arm_bot_pos_control/src/scripts/dataset_generator.sh` | Separate position-workspace experiment |
| `DNN_test/install_deps.sh` | Original dependency package selections, with self-overwrite removed |

## Supporting assets

The main ROS package installs `config/`, `description/`, `launch/`, `worlds/`, `meshes/`, and `scripts/`. Mesh and world paths are unchanged. The DNN model variants and scaler pickles stay in `DNN_test/`. Notebooks, MATLAB scripts, plots, and CSV logs remain next to their original experiments. New generated output is ignored without untracking existing research records.

The original root documents include `INVERSE_DYNAMICS_README.md`, `S_CURVE_TRAJECTORY_EXPLANATION.md`, `simulation Structure.md`, `project_overview.html`, and `DNN_CONTROLLER_DOCUMENTATION.html`. Treat rate and path examples in those documents as historical unless they match the current workflow guide.
