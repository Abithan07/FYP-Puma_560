# For all run this in Terminal 1 at start
```bash
colcon build --symlink-install && source install/setup.bash && ros2 launch arm_bot gazebo.launch.py
```
---

# In terminal 2 run this at start
```bash
source install/setup.bash
```

# DNN Controller
```bash
ros2 run arm_bot torque_publisher_dnn.py --csv-path \
    /home/priyankan/Desktop/FYP-Puma_560/Test_data/D/path_195_joint_states.csv
```

### This has options as,
```bash
--mode
```
After the mode type the needed option from below list
| Mode | Meaning |
|------|---------|
| `pid-only` | PID feedback only |
| `delan-only` | DeLaN feedforward only |
| `dnn` | DeLaN + GRU feedforward only |
| `pid-delan` | PID + DeLaN only |
| `pid-dnn` | PID + DeLaN + GRU |

### Analyze the output as
```bash
python3 src/scripts/analyze_dnn_performance.py \
  /home/priyankan/Desktop/FYP-Puma_560/arm_bot/src/scripts/logs/path_004_traj_pid_dnn_log_2.csv --plots
```
---

# CTC Controller
```bash
ros2 run arm_bot torque_publisher_ctc.py --csv-path \
    /home/priyankan/Desktop/FYP-Puma_560/Test_data/path_003_traj.csv
```

### This has options as,
```bash
--no-feedback                    Disable PD feedback term (model feedforward only)
--no-model                       Disable inverse-dynamics term (feedback only)
```

### Analyze the output as
```bash
python3 src/scripts/analyze_ctc_performance.py \
    /home/priyankan/Desktop/FYP-Puma_560/arm_bot/src/scripts/logs/path_004_traj_pid_dnn_log_2.csv --plots
```

---

# PID Controller
```bash
python3 src/scripts/controller/pid_controller/pid_controller_trajectory.py --trajectory-csv \
    /home/priyankan/Desktop/FYP-Puma_560/Test_data/path_003_traj.csv
```