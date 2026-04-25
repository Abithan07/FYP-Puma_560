# Simulation Explanation: CSV-Driven Open-Loop Gazebo Torque Replay

## 1. Purpose

This simulation setup replays precomputed joint torques from a dataset CSV into Gazebo through ROS 2 effort controllers. The goal is to evaluate how closely the robot follows the expected motion when torques are applied in open loop.

Open loop here means: during trajectory replay, torque commands are taken directly from the CSV and published at a fixed rate, without real-time correction from tracking error.

---

## 2. High-Level Architecture

The full pipeline is:

1. Launch Gazebo + robot model + ros2_control effort controllers.
2. Load one trajectory dataset CSV.
3. Publish torque samples (tau1, tau2, tau3) at 100 Hz.
4. Simultaneously log sensed joint position, velocity, and effort from /joint_states.
5. Compare expected (dataset) vs sensed (logged) values in plots.

Key launch and control wiring is defined in:

- [src/launch/robot_gui.launch.py](src/launch/robot_gui.launch.py)
- [src/config/my_controllers.yaml](src/config/my_controllers.yaml)
- [src/description/ros2_control.xacro](src/description/ros2_control.xacro)

Main runtime scripts:

- [src/scripts/torque_publisher.py](src/scripts/torque_publisher.py)
- [src/scripts/continuous_logger_triggered.py](src/scripts/continuous_logger_triggered.py)
- [src/scripts/dataset_generator.sh](src/scripts/dataset_generator.sh)
- [src/scripts/plot_comparison.py](src/scripts/plot_comparison.py)

---

## 3. Gazebo and Controller Wiring

### 3.1 Gazebo launch and robot spawn

The launch file starts Gazebo Sim and spawns the robot:

- Gazebo launch via ros_gz_sim: [src/launch/robot_gui.launch.py](src/launch/robot_gui.launch.py#L58)
- Robot entity creation: [src/launch/robot_gui.launch.py](src/launch/robot_gui.launch.py#L74)
- Clock bridge (simulation time): [src/launch/robot_gui.launch.py](src/launch/robot_gui.launch.py#L88)

### 3.2 ros2_control effort path

Controllers are spawned after robot spawn and joint state broadcaster readiness:

- Joint state broadcaster spawner: [src/launch/robot_gui.launch.py](src/launch/robot_gui.launch.py#L93)
- Joint effort controllers spawners: [src/launch/robot_gui.launch.py](src/launch/robot_gui.launch.py#L100)

Controller definitions:

- Update rate = 100 Hz: [src/config/my_controllers.yaml](src/config/my_controllers.yaml#L3)
- Forward command controllers: [src/config/my_controllers.yaml](src/config/my_controllers.yaml#L6)
- Effort command interface: [src/config/my_controllers.yaml](src/config/my_controllers.yaml#L20)

Robot-side ros2_control plugin and interfaces:

- GazeboSimSystem plugin: [src/description/ros2_control.xacro](src/description/ros2_control.xacro#L8)
- Effort command interfaces per joint: [src/description/ros2_control.xacro](src/description/ros2_control.xacro#L11)
- Gazebo ros2_control plugin library: [src/description/ros2_control.xacro](src/description/ros2_control.xacro#L48)

So command flow is:

Torque Publisher -> /joint_X_controller/commands -> ForwardCommandController (effort) -> gz_ros2_control -> Gazebo physics.

---

## 4. Dataset CSV Format and Parsing

The trajectory CSV uses a row-key layout (signal-per-row), not timestamp-per-row.

Example file:

- [src/scripts/Joint_states/path_461_joint_states.csv](src/scripts/Joint_states/path_461_joint_states.csv)

Key rows:

- t: [src/scripts/Joint_states/path_461_joint_states.csv](src/scripts/Joint_states/path_461_joint_states.csv#L1)
- dp1, dp2, dp3 (desired positions): [src/scripts/Joint_states/path_461_joint_states.csv](src/scripts/Joint_states/path_461_joint_states.csv#L2)
- dv1, dv2, dv3 (desired velocities): [src/scripts/Joint_states/path_461_joint_states.csv](src/scripts/Joint_states/path_461_joint_states.csv#L5)
- tau1, tau2, tau3 (torque commands): [src/scripts/Joint_states/path_461_joint_states.csv](src/scripts/Joint_states/path_461_joint_states.csv#L14)

CSV ingestion in torque publisher:

- Loader function: [src/scripts/torque_publisher.py](src/scripts/torque_publisher.py#L183)
- csv.reader loop: [src/scripts/torque_publisher.py](src/scripts/torque_publisher.py#L187)
- t assignment: [src/scripts/torque_publisher.py](src/scripts/torque_publisher.py#L195)
- tau assignment: [src/scripts/torque_publisher.py](src/scripts/torque_publisher.py#L199)

The script preloads all arrays into memory before replay to avoid runtime disk I/O delays.

---

## 5. Runtime Sequence in Torque Publisher

Main entry sequence is in:

- [src/scripts/torque_publisher.py](src/scripts/torque_publisher.py#L457)

### 5.1 Phase 1: stabilization

Before replay, the node waits for joint states and runs a stabilization stage to bring the arm near the initial pose.

This stage uses feedback (PID + compensation) and is not open-loop replay yet.

### 5.2 Logger startup

The node starts the triggered logger process and service clients:

- Logger launch: [src/scripts/torque_publisher.py](src/scripts/torque_publisher.py#L230)
- Service start request: [src/scripts/torque_publisher.py](src/scripts/torque_publisher.py#L272)

### 5.3 Phase 2: open-loop replay

Open-loop trajectory phase starts here:

- Phase label: [src/scripts/torque_publisher.py](src/scripts/torque_publisher.py#L514)

Timer-driven replay callback:

- Callback: [src/scripts/torque_publisher.py](src/scripts/torque_publisher.py#L417)
- tau1 publish sample line: [src/scripts/torque_publisher.py](src/scripts/torque_publisher.py#L429)
- Timer at 0.01 s (100 Hz): [src/scripts/torque_publisher.py](src/scripts/torque_publisher.py#L531)

At each tick k:

- command_joint1 = tau1[k]
- command_joint2 = tau2[k]
- command_joint3 = tau3[k]
- k = k + 1

No trajectory feedback correction is applied to these torques in this phase.

### 5.4 Logger stop and shutdown

After replay, logger stop service is called:

- Stop request: [src/scripts/torque_publisher.py](src/scripts/torque_publisher.py#L298)

Then torques are set to zero on shutdown.

---

## 6. Triggered Logging Mechanics

Triggered logger node:

- [src/scripts/continuous_logger_triggered.py](src/scripts/continuous_logger_triggered.py#L32)

Services:

- /logger/start: [src/scripts/continuous_logger_triggered.py](src/scripts/continuous_logger_triggered.py#L100)
- /logger/stop: [src/scripts/continuous_logger_triggered.py](src/scripts/continuous_logger_triggered.py#L106)

Logging loop timing:

- 100 Hz timer: [src/scripts/continuous_logger_triggered.py](src/scripts/continuous_logger_triggered.py#L89)

Data source:

- /joint_states callback: [src/scripts/continuous_logger_triggered.py](src/scripts/continuous_logger_triggered.py#L199)
- effort capture: [src/scripts/continuous_logger_triggered.py](src/scripts/continuous_logger_triggered.py#L224)

CSV output schema written at start:

- Header row: [src/scripts/continuous_logger_triggered.py](src/scripts/continuous_logger_triggered.py#L145)

Schema is:

- time_elapsed
- pos1, pos2, pos3
- vel1, vel2, vel3
- torque1, torque2, torque3

---

## 7. Batch Execution and Plot Generation

Dataset launcher script:

- [src/scripts/dataset_generator.sh](src/scripts/dataset_generator.sh)

It runs torque replay on selected path files:

- Single or range replay calls: [src/scripts/dataset_generator.sh](src/scripts/dataset_generator.sh#L27)

Then it generates comparison plots:

- Plot invocation: [src/scripts/dataset_generator.sh](src/scripts/dataset_generator.sh#L167)

Plot parsing logic:

- Dataset parser: [src/scripts/plot_comparison.py](src/scripts/plot_comparison.py#L117)
- Logger parser: [src/scripts/plot_comparison.py](src/scripts/plot_comparison.py#L148)

---

## 8. Why Expected and Sensed Curves Can Diverge

Large mismatch in open-loop replay is normal, especially over longer trajectories.

Main reasons:

1. No online correction in replay phase.
2. Model mismatch between dynamics used to precompute tau and simulator dynamics.
3. Sensitivity to initial condition mismatch at replay start.
4. Timing and discretization effects (even with matched nominal 100 Hz).
5. Non-ideal effects: friction/contact/numerical integration differences.

As a result, smooth injected torques can still produce sensed position and velocity drift, oscillation, or phase mismatch.

---

## 9. Open-Loop vs Closed-Loop Clarification

This setup is hybrid in overall experiment flow:

- Closed-loop in Phase 1 (stabilization).
- Open-loop in Phase 2 (trajectory torque replay).

If you need strict end-to-end open-loop from initial condition as well, remove or bypass the stabilization phase and start replay immediately from current state.

---

## 10. Practical Validation Checklist

To verify that a run is truly open-loop replay:

1. Confirm torque samples are sourced from tau arrays only in replay callback.
2. Confirm replay timer is fixed at 0.01 s.
3. Confirm controller interface is effort.
4. Confirm no error term modifies tau during Phase 2.
5. Confirm logger starts before replay and stops after replay.

All five conditions are satisfied by the current implementation.

---

## 11. Optional Next Improvements

If tighter tracking is required, consider:

1. Add feedback term during replay: tau_cmd = tau_ff + Kp*e + Kd*edot.
2. Add timestamp-based interpolation instead of pure index stepping.
3. Sync replay with simulation clock and detect overruns.
4. Add initial-state alignment checks before replay starts.
5. Perform parameter identification to reduce model mismatch.

These changes move behavior from pure open-loop replay toward feedforward + feedback tracking.

---

## 12. Concrete Run Case Study: path_461

This section adds a measured analysis for the dataset [src/scripts/Joint_states/path_461_joint_states.csv](src/scripts/Joint_states/path_461_joint_states.csv) against:

- [src/scripts/logs/path_461_log_1.csv](src/scripts/logs/path_461_log_1.csv)
- [src/scripts/logs/path_461_log_2.csv](src/scripts/logs/path_461_log_2.csv)
- [src/scripts/logs/path_461_log_3.csv](src/scripts/logs/path_461_log_3.csv)

Method used:

1. Align dataset time to start at 0 s using `t - t[0]`.
2. Interpolate expected signals at logger timestamps.
3. Compute per-joint RMSE and max absolute error for position.
4. Detect first timestamp where absolute position error exceeds 5 degrees and 10 degrees.

All three runs had nearly identical results over ~10.22 s (~1022 samples), indicating repeatable simulator behavior for this trajectory.

### 12.1 Position Error Summary

| Log file | Joint 1 RMSE / Max (deg) | Joint 2 RMSE / Max (deg) | Joint 3 RMSE / Max (deg) |
|---|---:|---:|---:|
| path_461_log_1.csv | 2.03 / 2.72 | 1.99 / 6.62 | 2.52 / 6.04 |
| path_461_log_2.csv | 2.05 / 2.73 | 1.99 / 6.62 | 2.52 / 6.04 |
| path_461_log_3.csv | 2.02 / 2.74 | 1.98 / 6.62 | 2.52 / 6.03 |

### 12.2 Divergence Onset (Position)

First time where absolute position error exceeded threshold:

- Joint 1:
	- >5 degrees: not reached
	- >10 degrees: not reached
- Joint 2:
	- >5 degrees: ~9.56 s to ~9.57 s
	- >10 degrees: not reached
- Joint 3:
	- >5 degrees: ~8.92 s
	- >10 degrees: not reached

### 12.3 Velocity and Torque Agreement (RMSE)

Representative values (all three logs are very close):

- Joint 1: velocity RMSE ~0.41 to 0.42 deg/s, torque RMSE ~0.03 Nm
- Joint 2: velocity RMSE ~1.24 deg/s, torque RMSE ~0.05 Nm
- Joint 3: velocity RMSE ~1.02 deg/s, torque RMSE ~0.00 Nm

Interpretation:

1. Torque replay fidelity is high (very low torque RMSE), so command publication and controller routing are working as intended.
2. Position error remains moderate for most of the run and grows mainly near the end for joints 2 and 3.
3. Since torque tracking is good but position drift still appears, the dominant source is open-loop accumulation/model mismatch rather than command transport failure.

### 12.4 Practical Conclusion for path_461

For path_461, the system is stable and repeatable in open loop over ~10 s, with position errors mostly in the low single-digit degree range and late-run excursions above 5 degrees on joints 2 and 3 only.

This run supports the conclusion that your pipeline is functioning correctly as an open-loop torque replay experiment; the remaining mismatch is expected from feedforward-only execution.
