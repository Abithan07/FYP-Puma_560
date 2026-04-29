# Guide to use this repo.
#### Following all file structure will be in the folder FYP-puma_560

## Trajectory Generation
This repository contains multiple scripts to generate PUMA-560 trajectories and related dataset files. Below are the main scripts and how to use them.

### 1) gen_traj.py
Quick generator that creates minimum-jerk trajectories and appends metadata to `Dataset/trajectories.csv`.

Usage (from repository root):
```bash
python gen_traj.py <start_id> <num_paths>
```
Example:
```bash
python3 gen_traj.py 1 10
```
Output:
- Trajectory CSV files: `Dataset/Trajectories/path_XXX_trajectory.csv`
- Metadata/record: `Dataset/trajectories.csv` (columns: path_id,q1_rad,q2_rad,q3_rad,q1_deg,q2_deg,q3_deg,T_total)

Notes:
- Uses fixed joint limits and default sampling `dt = 0.01`.
- Good for quickly creating many simple trajectories.

### 2) trajectory_generator.py
A more feature-rich generator that creates joint-angle files, end-effector XYZ, and full trajectory files. It supports seeds, plotting, custom output dirs, and generating single trajectories to a specified endpoint.

Basic usage (from repository root):
```bash
python trajectory_generator.py <num_paths> [--start-id 601] [--base-dir /path/to/Dataset] [--seed 42] [--plot]
```
Common examples:
- Generate 10 random trajectories (IDs 601–610):
```bash
python trajectory_generator.py 10
```
- Generate single trajectory to specified joint angles (degrees):
```bash
python trajectory_generator.py 1 --q-end-deg 45.0 30.0 90.0 --t-total 15.0 --path-id 999
```
Key options:
- `--start-id`: starting path ID (default 601)
- `--base-dir`: base output directory (default: `/home/priyankan/Desktop/FYP-Puma_560/Dataset`)
- `--seed`: random seed for reproducibility
- `--plot`: show live 3D plots (requires matplotlib)
- `--q-end-deg`: supply three joint angles (deg) to generate a single specific trajectory

Outputs (under `<base-dir>`):
- `Angles/path_XXX_angles.csv` — joint angle time series (rad)
- `XYZ/path_XXX_xyz.csv` — end-effector positions (m)
- `Trajectories/path_XXX_traj.csv` — labeled trajectory CSV (row-wise with header)
- `JointStatesAll/all_paths_0.csv` — path records (used to avoid duplicates)

Notes:
- Use `--q-end-deg` with `--t-total` or let the script auto-compute a minimum duration respecting velocity/accel limits.
- `--plot` is optional; if matplotlib is missing, plotting is skipped with a warning.

### 3) DNN_test/datagen.py
Utility to generate a single trajectory and save a dataset subset under `DNN_test/Data`. Intended for preparing inputs for DNN experiments.

Usage (from repository root):
```bash
python DNN_test/datagen.py <path_id> [--seed <int>]
```
Example:
```bash
python3 DNN_test/datagen.py 601 --seed 123
```
Outputs (under `DNN_test/Data`):
- `Angles/path_XXX_angles.csv` — joint angles (rad)
- `XYZ/path_XXX_xyz.csv` — end-effector positions (m)
- `Trajectory/path_XXX_trajectories.csv` — trajectory rows (same format used elsewhere)
- `DNN_test/summary.csv` and `DNN_test/summary_deg.csv` — summary of endpoints and durations

Notes:
- `datagen.py` expects a `path_id` and optionally a `--seed` for deterministic generation.
- It uses its own summary file to ensure uniqueness of generated endpoints.

---

If you'd like, I can also add short examples showing how to load and visualize the generated CSVs, or add a small troubleshooting section describing common errors (missing dependencies, permission issues, etc.).