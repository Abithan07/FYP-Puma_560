# Python dependency groups

- `runtime.txt`: trajectory generation, analytical dynamics, plotting, and the PyQt5 GUI.
- `dnn.txt`: runtime dependencies plus the inference stack used by the default checkpoints.

Use Python 3.10 with the project's ROS Humble environment. These constraints are based on source imports and existing dependency records; they have not been validated as a complete fresh installation. Read [environment notes](../docs/PUBLICATION_NOTES.md) before installing.

The original `DNN_test/DeLaN/requirements-cpu.txt` and `DNN_test/install_deps.sh` are retained as historical dependency records. They are not interchangeable lockfiles. The new groups avoid unrelated packages from those snapshots, include the saved scaler's scikit-learn dependency, and constrain NumPy below 2 for compatibility with the older binary stack.

ROS packages, Gazebo plugins, Tk, and desktop applications are installed separately. No dependencies were installed or upgraded during cleanup.
