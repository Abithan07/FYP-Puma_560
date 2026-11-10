# Main PUMA 560 simulation workspace

This is the workspace used by the final demo and the DNN/CTC/PID comparisons.

```bash
source /opt/ros/humble/setup.bash
cd arm_bot  # from the repository root
colcon build --symlink-install --packages-select arm_bot
source install/setup.bash
python3 src/scripts/Demo_controller/traj_gui.py
```

To launch Gazebo and RViz separately:

```bash
ros2 launch arm_bot gazebo.launch.py
```

The active ROS package is directly inside `src/`; its `package.xml` and `CMakeLists.txt` install the robot description, meshes, launch files, and controller executables.

- [Run workflows](../docs/WORKFLOWS.md)
- [Controller and data architecture](../docs/ARCHITECTURE.md)
- [Source inventory](../docs/REPOSITORY_MAP.md)
- [Dependencies and release notes](../docs/PUBLICATION_NOTES.md)

`all.sh` compares controllers using trajectory CSVs. `pipeline.sh` is the earlier data-generation workflow. Both resolve the workspace from their own location, so they can be invoked from another working directory. Source the ROS environment first.
