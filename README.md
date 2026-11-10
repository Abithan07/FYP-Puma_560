# PUMA 560 — Final Year Project

ROS 2 simulation and trajectory tracking for the first three joints of a modified PUMA 560 arm. The project compares PID feedback, analytical computed torque control (CTC), and learned inverse dynamics using a Deep Lagrangian Network (DeLaN) with a GRU residual.

The final demonstration is in **`arm_bot/src/scripts/Demo_controller/`**. The controller comparison workflow starts at **`arm_bot/all.sh`**.

## Start here

| Purpose | Guide |
| --- | --- |
| Understand the robot, controllers, and data flow | [Architecture](docs/ARCHITECTURE.md) |
| Build, launch the demo, and compare controllers | [Workflows](docs/WORKFLOWS.md) |
| Find source files and distinguish experiments | [Repository map](docs/REPOSITORY_MAP.md) |
| Understand dependencies, datasets, and publication gaps | [Publication notes](docs/PUBLICATION_NOTES.md) |
| Review this cleanup and its validation | [Cleanup report](docs/CLEANUP_REPORT.md) |

## Build and run the final demo

Use the project's ROS 2 Humble desktop environment with Gazebo Sim, `gz_ros2_control`, RViz, and the Python dependencies described in [Publication notes](docs/PUBLICATION_NOTES.md). Commands below start in the repository root.

```bash
source /opt/ros/humble/setup.bash
cd arm_bot
colcon build --symlink-install --packages-select arm_bot
source install/setup.bash
python3 src/scripts/Demo_controller/traj_gui.py
```

In the GUI, choose the endpoint and optional waypoint, then press **RUN**. The GUI starts the simulation, waits for the three effort controllers, generates a trajectory, executes the demo DNN controller, and plots the results. **STOP** terminates the run.

Build from `arm_bot/`: the repository also contains an older workspace with the same ROS package name. Building every directory from the repository root can discover duplicate packages.

## Compare controllers

The original 50-trajectory batch requires local CSV files that are **not tracked in Git**. Check their availability before launching:

```bash
source /opt/ros/humble/setup.bash
bash arm_bot/all.sh --check
```

Run selected trajectories from an explicit dataset directory:

```bash
bash arm_bot/all.sh --csv-dir /absolute/path/to/joint_states 641 647
```

This workflow uses GNOME Terminal windows and preserves the original DNN → CTC → PID order, with simulation resets between controllers. See [Workflows](docs/WORKFLOWS.md) for individual commands and log locations.

## Check the code without launching the simulator

```bash
python3 tools/check_repository.py
python3 -m unittest discover -s tests -v
```

The syntax check uses the Python standard library. The regression checks also require NumPy and SymPy. They cover asset discovery, subprocess error reporting, trajectory boundaries, and inverse dynamics properties.

## Research artifacts

Existing models, CSVs, plots, notebooks, MATLAB scripts, and experiment logs retain their original paths. Earlier implementations are identified in the repository map. Existing HTML/PDF explanations remain available; the guides above describe the source reviewed during this cleanup.

The main configuration runs at **100 Hz**. The separate inverse dynamics data-collection pipeline documents a **1000 Hz** experiment and requires a deliberately matched simulator/controller configuration.

Before public release, resolve the outstanding license and attribution metadata and reproduce an interactive run in a compatible environment. No repository-wide license or unverified performance claim has been added.
