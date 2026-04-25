# 1000 Hz Setup Guide

This guide lists the values that must be changed when running the arm simulation and data pipeline at 1000 Hz.

## What must run at 1000 Hz

The system has three timing layers:

1. Gazebo physics step
2. ros2_control update rate
3. Data publisher and logger timers

All three should be aligned to 1 ms for a true 1000 Hz setup.

## Files to change

### 1. Gazebo physics step

File: [src/worlds/empty_fortress.sdf](src/worlds/empty_fortress.sdf)

Current values:

```xml
<physics name="10ms" type="ode">
  <max_step_size>0.01</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

Change to:

```xml
<physics name="1ms" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

### 2. Controller manager rate

File: [src/config/my_controllers.yaml](src/config/my_controllers.yaml)

Current value:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000
```

This is already set correctly for 1000 Hz.

### 3. Torque publisher timing

File: [src/scripts/torque_publisher.py](src/scripts/torque_publisher.py)

Current 100 Hz assumptions to change:

```python
self.get_logger().info(f'Control frequency: 100 Hz (dt={self.dt:.4f}s)')
```

```python
dt = 0.01  # 100Hz = 0.01s
```

```python
self.stabilization_timer = self.create_timer(0.01, self.stabilization_callback)
self.trajectory_timer = self.create_timer(0.01, self.trajectory_callback)
```

```python
hold_iterations = [0]
if hold_iterations[0] >= 100:
```

```python
if self.stabilization_iterations >= 12000:
```

```python
if self.stabilization_iterations % 50 == 0:
```

Change these values for 1000 Hz:

```python
self.get_logger().info(f'Control frequency: 1000 Hz (dt={self.dt:.4f}s)')
```

```python
dt = 0.001  # 1000Hz = 0.001s
```

```python
self.stabilization_timer = self.create_timer(0.001, self.stabilization_callback)
self.trajectory_timer = self.create_timer(0.001, self.trajectory_callback)
```

```python
if hold_iterations[0] >= 1000:
```

```python
if self.stabilization_iterations >= 120000:
```

```python
if self.stabilization_iterations % 500 == 0:
```

### 4. Triggered logger timing

File: [src/scripts/continuous_logger_triggered.py](src/scripts/continuous_logger_triggered.py)

Current value:

```python
self.log_timer = self.create_timer(0.01, self.timer_callback)  # 100Hz = 0.01s
```

Change to:

```python
self.log_timer = self.create_timer(0.001, self.timer_callback)  # 1000Hz = 0.001s
```

Also update the log messages if you want them to match the actual rate:

```python
'✓ LOGGING STARTED - Recording trajectory data at 1000Hz'
```

```python
"""Log data at consistent 1000Hz rate when logging is active"""
```

## Data requirements

If you are feeding CSV data into the publisher, the dataset itself must also be 1000 Hz:

- `t` should increase in steps of `0.001`
- `dp1`, `dp2`, `dp3` must have one sample per 1 ms step
- `tau1`, `tau2`, `tau3` must match the same sample count

If the dataset is still 100 Hz, upsample it first using interpolation before running the simulation.

## Pipeline entrypoint

File: [src/scripts/dataset_generator.sh](src/scripts/dataset_generator.sh)

This script launches:

```bash
python3 src/scripts/torque_publisher.py --csv-path ...
```

So the torque publisher must be updated before the pipeline will behave like 1000 Hz.

## Run sequence

```bash
cd /home/priyankan/Desktop/FYP-Puma_560/arm_bot
colcon build
source install/setup.bash
./pipeline.sh <path_id>
```

## Quick verification

Use these checks after the changes:

```bash
ros2 topic hz /joint_states
ros2 topic hz /joint_1_controller/commands
```

If the values are not close to 1000 Hz, the bottleneck is usually Gazebo physics, Python timer jitter, or the input dataset rate.

## Practical note

Python timers can jitter at 1 ms. If you need strict deterministic 1000 Hz control, move the publisher into a C++ ROS 2 controller or a ros2_control plugin.