# Computed Torque Control Guide

This folder keeps only two practical markdown files:

- [README_CTC.md](README_CTC.md) for the short overview
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md) for commands and troubleshooting

## What the controller does

The CTC node computes torques online at 100 Hz using the current measured joint state and the current trajectory sample. It publishes the command, logs the run, and can be analyzed after execution.

Default log naming is trajectory-based with an incrementing run index:

`path_001_trajectory.csv -> path_001_trajectory_ctc_log_1.csv`

If that file already exists, the next run becomes:

`path_001_trajectory_ctc_log_2.csv`

The analyzer uses the log filename stem, so the plot names stay grouped with the same run:

`path_001_trajectory_ctc_log_1_trajectory.png`
`path_001_trajectory_ctc_log_1_errors.png`
`path_001_trajectory_ctc_log_1_torques.png`
`path_001_trajectory_ctc_log_1_velocity.png`

## Typical workflow

```bash
# 1. Launch simulation
cd ~/Desktop/FYP-Puma_560/arm_bot && source install/setup.bash
ros2 launch arm_bot gazebo.launch.py

# 2. Run controller
ros2 run arm_bot torque_publisher_ctc.py \
  --csv-path src/scripts/script_resources/path_001_trajectory.csv

# 3. Analyze the run
python3 arm_bot/src/scripts/analyze_ctc_performance.py \
  arm_bot/src/scripts/script_resources/path_001_trajectory_ctc_log_1.csv \
  --plots --output-dir arm_bot/src/scripts/plots
```

## Notes

- Use `--log-path` only if you want to override the default incremental naming.
- If you run the same trajectory again, the run number increases automatically.
- For commands, troubleshooting, and parameter values, use [QUICK_REFERENCE.md](QUICK_REFERENCE.md).
