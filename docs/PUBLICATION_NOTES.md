# Publication and reproducibility notes

The organized working tree retains the final demonstration and comparison workflows. Follow [the final run guide](../RUN_GUIDE.md) for current commands. Earlier run instructions were removed to avoid conflicting entry points.

## What was preserved

The cleanup recorded and verified **4,106 protected artifacts**, including 719 originally tracked artifacts. Trajectories, datasets, trained weights, scalers, recorded CSVs/logs, plots, mesh assets, notebooks, and MATLAB references retain their pre-organization bytes. Locations and SHA-256 hashes are in `data/MANIFEST.csv`.

The branch now includes the previously local runtime inputs, all retained trajectory/dataset collections, DeLaN training data, models, and a CPU inference reference. [DATASET_MANIFEST.csv](../data/DATASET_MANIFEST.csv) inventories this **1,852-file, 838.74 MiB** published snapshot. `python3 tools/verify_data.py --datasets` verifies it in a fresh clone after the branch is pushed. Historical outputs/build logs remain local unless already tracked. The older preservation manifest and its `tracked` column describe the pre-cleanup state; `--all` still requires the original local archive.

Successful control/inference calculations and default gains/periods remain those of the retained final implementations. Inference exceptions now abort the trajectory, send zero torque, and return a failure code rather than silently continuing with zero-valued samples. Imports, subprocess paths, and output directories were updated for the new layout. The main ROS controller rate remains 100 Hz. The separate intermediate 1000 Hz pipeline was removed with the obsolete experiments.

## Environment and validation limits

The main package targets the original ROS 2 Humble/Python 3.10 desktop environment, using `ros_gz_sim`, `ros_gz_bridge`, and `gz_ros2_control`. `requirements/reproduce.txt` pins observed top-level CPU/runtime versions; the requirement groups are not a verified complete environment lock. `requirements/delan-original.txt` preserves the separate original training snapshot. The [clone and reproduction guide](REPRODUCIBILITY.md) records native package versions and a numerical reference-comparison procedure.

The default host Python has NumPy 2.2.6 and Matplotlib 3.5.1, which are incompatible. The project now selects `.venv-ros/`, an overlay with NumPy 1.26.4 and the saved scaler's scikit-learn 1.6.1. It reuses the existing ROS/inference packages without changing host dependencies or the earlier `.venv/`. `arm_bot/setup_runtime.sh` recreates the overlay; `tools/check_runtime.py` validates conversions and rendering before motion. The launchers and logger use one consistent interpreter.

Build, CLI/launch loading, offline regression checks, data preservation, and offscreen GUI construction were checked. A complete interactive motion/plotting run and a closed-loop comparison were not validated during organization. The repaired runtime also passed the full 1,802-sample demo-model inference sequence and a render of the reported log. These are offline checks, not closed-loop tracking validation. Record a successful interactive run, then export its actual Python, ROS, Gazebo, and control-plugin versions before reporting a reproduced release.

The tested scientific versions are NumPy 1.26.4, PyTorch 2.1.0+cpu, JAX/jaxlib 0.6.2, Haiku 0.0.16, SciPy 1.15.3, pandas 2.3.3, Matplotlib 3.5.1, and scikit-learn 1.6.1. This records the tested local stack, not a complete fresh-install lockfile.

The comparison runner retains the original best-effort reset calls. Validate simulator reset behavior on the target installation before interpreting controller comparisons.

## Research provenance

The original DeLaN training code/data, MATLAB references, and notebooks are retained under `research/`. Notebook/MATLAB internal paths remain historical and have not been revalidated after relocation. The standalone inference version was reduced to the latest `inf7.py` implementation, now `tools/infer_torques.py`, with asset/output paths updated.

A complete GRU training procedure, training seeds, dataset splits, checkpoint-selection criteria, and evaluation provenance need author input for training-level reproducibility. The analytical model remains the original CoG-based symbolic model; the retained `compute_christoffel_symbols` helper is not part of the main symbolic Coriolis-vector validation.

## Release metadata still needed

The authors should supply a repository-wide license, consistent package license metadata, real maintainer/contact details, author/university/project citation information, and attribution/redistribution permissions for borrowed code, meshes, models, and data. The main package's placeholder license/contact metadata has not been invented or replaced on the authors' behalf.

No remote push, public publication, or release was performed. Removed intermediate source remains recoverable from Git commit `76212c2` and the original `demo` branch.
