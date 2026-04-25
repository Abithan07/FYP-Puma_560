# arm_bot_new — Ideal PUMA 560 Gazebo Simulation

A clean ROS2 + Ignition Gazebo Fortress simulation of the **3-DOF Modified PUMA 560**,
built to validate DNN-predicted torques against the analytical Euler-Lagrange model.

---

## What this package does

```
DNN inference (inf7.py)
       │
       │  path_XXX_joint_states.csv
       ▼
torque_publisher.py  ──(torque commands)──▶  Gazebo (ideal physics)
                                                    │
                                            /joint_states
                                                    │
                                       triggered_logger.py
                                                    │
                                          path_XXX_log_N.csv
                                                    │
                                            validate.py
                                                    │
                                       tracking_comparison.png
                                         tracking_error.png
                                          phase_portrait.png
```

### Key design decisions

| Parameter | Value | Source |
|---|---|---|
| Gravity | 9.81 m/s² (−Z) | Matches MATLAB `g = 9.81` |
| Physics step | 1 ms (ODE sub-steps 10× per control cycle) | Stability |
| Control rate | 100 Hz | Matches MATLAB trajectory sampling |
| Joint damping | 0 | Ideal frictionless model |
| Joint friction | 0 | Ideal frictionless model |
| Self-collision | disabled | Clean ideal simulation |
| d₃ offset | −0.0934 m | Matches MATLAB `d_dh(3) = -0.0934` |

### DH parameters (identical to MATLAB model)

| Joint | α (rad) | a (m) | d (m) |
|---|---|---|---|
| 1 | 0 | 0 | 0 |
| 2 | −π/2 | 0 | 0.2435 |
| 3 | 0 | 0.4318 | −0.0934 |

---

## Prerequisites

- ROS2 Humble
- Ignition Gazebo Fortress (`ros-humble-ros-gz*`)
- `gz_ros2_control`, `ros2_control`, `ros2_controllers`
- Python: `rclpy`, `numpy`, `matplotlib`

---

## Build

```bash
# Navigate to this workspace
cd /home/priyankan/Desktop/FYP-Puma_560/arm_bot_new

# Build (source ROS2 first if not already)
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Source the workspace overlay
source install/setup.bash
```

---

## Run the Simulation

### Step 1 — Launch Gazebo

In **Terminal 1**:
```bash
cd /home/priyankan/Desktop/FYP-Puma_560/arm_bot_new
source install/setup.bash
ros2 launch arm_bot_new sim.launch.py
```

Wait until you see all three controller spawners report success:
```
[spawner-*] Configured and activated joint_1_controller
[spawner-*] Configured and activated joint_2_controller
[spawner-*] Configured and activated joint_3_controller
```

### Step 2 — Run DNN inference (if you haven't already)

```bash
cd /home/priyankan/Desktop/FYP-Puma_560/DNN_test
python3 inf7.py 461
# Output: Data/path_461_joint_states.csv
```

### Step 3 — Publish torques & log response

In **Terminal 2** (source the workspace first):
```bash
source /home/priyankan/Desktop/FYP-Puma_560/arm_bot_new/install/setup.bash

# Simplest form — path_id only, CSV resolved automatically
python3 /home/priyankan/Desktop/FYP-Puma_560/arm_bot_new/src/scripts/torque_publisher.py 461

# Optional overrides
python3 /home/priyankan/Desktop/FYP-Puma_560/arm_bot_new/src/scripts/torque_publisher.py 461 \
    --log-dir ~/puma560_logs
```

The script runs two phases automatically:
1. **Stabilisation** — PID drives joints to trajectory start [0°, 45°, 135°]
2. **Torque replay** — Open-loop torque commands at 100 Hz

Log is saved to: `~/puma560_logs/path_461_log_N.csv`

### Step 4 — Validate results

```bash
python3 /home/priyankan/Desktop/FYP-Puma_560/arm_bot_new/src/scripts/validate.py \
    --desired /home/priyankan/Desktop/FYP-Puma_560/DNN_test/Data/path_461_joint_states.csv \
    --actual  ~/puma560_logs/path_461_log_1.csv \
    --out     ~/puma560_logs/results_461
```

Outputs:
- `tracking_comparison.png` — Desired vs actual position per joint
- `tracking_error.png`      — Error over time per joint
- `phase_portrait.png`      — Phase plane (position vs velocity)
- Console RMSE report

---

## Full end-to-end example (one trajectory)

```bash
TRAJ=603

# 1. Generate trajectory
python3 /home/priyankan/Desktop/FYP-Puma_560/DNN_test/datagen.py $TRAJ

# 2. DNN torque prediction
cd /home/priyankan/Desktop/FYP-Puma_560/DNN_test
python3 inf7.py $TRAJ

# 3. Simulate (Gazebo must already be running from Step 1)
source /home/priyankan/Desktop/FYP-Puma_560/arm_bot_new/install/setup.bash
python3 /home/priyankan/Desktop/FYP-Puma_560/arm_bot_new/src/scripts/torque_publisher.py $TRAJ

# 4. Validate
python3 /home/priyankan/Desktop/FYP-Puma_560/arm_bot_new/src/scripts/validate.py \
    --desired /home/priyankan/Desktop/FYP-Puma_560/DNN_test/Data/path_${TRAJ}_joint_states.csv \
    --actual  ~/puma560_logs/path_${TRAJ}_log_1.csv \
    --out     ~/puma560_logs/results_${TRAJ}
```

---

## Topics and services at runtime

| Topic / Service | Type | Description |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Actual joint positions, velocities, efforts |
| `/joint_1_controller/commands` | `std_msgs/Float64MultiArray` | Torque command for joint 1 |
| `/joint_2_controller/commands` | `std_msgs/Float64MultiArray` | Torque command for joint 2 |
| `/joint_3_controller/commands` | `std_msgs/Float64MultiArray` | Torque command for joint 3 |
| `/logger/start` | `std_srvs/Trigger` | Begin logging |
| `/logger/stop`  | `std_srvs/Trigger` | Stop logging and flush to disk |
| `/logger/recording_active` | `std_msgs/Bool` | Latched recording state |

---

## Package structure

```
arm_bot_new/
├── README.md
└── src/
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   ├── my_controllers.yaml          # Controller config (100 Hz, effort)
    │   └── gaz_ros2_ctl_use_sim.yaml    # use_sim_time: true
    ├── description/
    │   ├── robot.urdf.xacro             # Top-level XACRO
    │   ├── robot_core.xacro             # Links, joints, DH geometry
    │   ├── inertial_macros.xacro        # Mass/inertia matching MATLAB
    │   └── ros2_control.xacro           # Hardware interface + plugin
    ├── launch/
    │   └── sim.launch.py                # Single launch file
    ├── meshes/
    │   └── puma_link{1-4}.stl           # Visual/collision meshes
    ├── scripts/
    │   ├── torque_publisher.py          # Reads CSV → sends torques
    │   ├── triggered_logger.py          # Records /joint_states to CSV
    │   └── validate.py                  # RMSE + comparison plots
    └── worlds/
        └── puma_world.sdf               # Gravity=9.81, 1ms physics step
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Robot falls / oscillates at startup | Stabilisation PID not converged | Wait; if timeout, check that gravity is correct in world SDF |
| `csv not found` error in torque_publisher | Wrong `--csv-path` | Run `inf7.py` first, check Data/ directory |
| `Logger service not available` | triggered_logger.py already running | Kill leftover python3 processes |
| Joint effort in log is all zeros | `<state_interface name="effort"/>` missing | Check ros2_control.xacro — already included |
| Tracking error > 5° | DNN model mismatch or ODE instability | Try reducing real_time_factor to 0.5 in puma_world.sdf |
