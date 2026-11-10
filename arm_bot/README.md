# Final PUMA 560 workspace

This is the single retained ROS workspace. From the repository root:

```bash
source /opt/ros/humble/setup.bash
bash arm_bot/run_demo.sh        # Final presentation GUI
bash arm_bot/all.sh --check    # Check the comparison batch
bash arm_bot/all.sh            # Run DNN → CTC → PID comparisons
```

Read the [final run guide](../RUN_GUIDE.md) for environment setup, data requirements, manual commands, and output locations.

Source is grouped under `src/scripts/demo/`, `controllers/`, `shared/`, and `analysis/`. The only simulator entry point is `gazebo.launch.py`. Datasets and recorded results live outside the ROS package in `../data/`; checkpoints and scalers live in `../models/`.
