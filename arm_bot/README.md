# Final PUMA 560 workspace

This is the single retained ROS workspace. From the repository root:

```bash
source /opt/ros/humble/setup.bash
bash arm_bot/run_demo.sh        # Final presentation GUI
bash arm_bot/all.sh --check    # Check the comparison batch
bash arm_bot/all.sh            # Run DNN → CTC → PID comparisons
```

Read the [complete run guide](../RUN_GUIDE.md) for environment setup, data requirements, manual commands, output locations, analysis, and troubleshooting.

The GUI uses PID + DNN automatically; its joint-controller indicators are not algorithm selectors. Use [controller selection](../RUN_GUIDE.md#6-choose-and-run-a-controller) for PID, CTC, and DNN modes, or [automatic comparisons](../RUN_GUIDE.md#8-run-automatic-comparisons) to run all three on one input. The guide includes a generated-input example that does not require the original local batch dataset.

Source is grouped under `src/scripts/demo/`, `controllers/`, `shared/`, and `analysis/`. The only simulator entry point is `gazebo.launch.py`. Datasets and recorded results live outside the ROS package in `../data/`; checkpoints and scalers live in `../models/`.
