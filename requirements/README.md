# Python dependencies

- `runtime.txt`: scientific processing, analytical dynamics, plotting, and PyQt5.
- `dnn.txt`: runtime dependencies plus the inference stack for the default checkpoints.
- `ros-overlay.txt`: project-only NumPy/scikit-learn overrides for the existing ROS installation.
- `delan-original.txt`: untouched original DeLaN training dependency snapshot.

Use Python 3.10 with the ROS Humble environment. The focused runtime/inference constraints are based on the source and original environment records; they are not a validated fresh-install lockfile. Read [the run guide](../RUN_GUIDE.md) before installing.

The NumPy constraint below 2 is intended for the original Matplotlib/PyTorch binary generation. ROS packages, Gazebo plugins, Tk, and GNOME Terminal are separate system dependencies. The runtime repair installed NumPy 1.26.4 and scikit-learn 1.6.1 only inside `.venv-ros/`. Host packages remain unchanged. Run `bash arm_bot/setup_runtime.sh` to recreate and check this overlay.
