# Publication and reproducibility notes

The organized working tree retains the final demonstration and comparison workflows. Follow [the final run guide](../RUN_GUIDE.md) for current commands. Earlier run instructions were removed to avoid conflicting entry points.

## What was preserved

The cleanup recorded and verified **4,106 protected artifacts**, including 719 originally tracked artifacts. Trajectories, datasets, trained weights, scalers, recorded CSVs/logs, plots, mesh assets, notebooks, and MATLAB references retain their pre-organization bytes. Locations and SHA-256 hashes are in `data/MANIFEST.csv`.

Original tracked data remains tracked after moving. Large local-only datasets/results remain local-only, including the inputs for the original 50-path comparison. The manifest inventories those local files but does not make them part of a fresh clone. Use `python3 tools/verify_data.py --all` in the original working checkout to check them.

Control/inference calculations and default gains/periods remain those of the retained final implementations. Imports, subprocess paths, and output directories were updated for the new layout. The main ROS controller rate remains 100 Hz. The separate intermediate 1000 Hz pipeline was removed with the obsolete experiments.

## Environment and validation limits

The main package targets the original ROS 2 Humble/Python 3.10 desktop environment, using `ros_gz_sim`, `ros_gz_bridge`, and `gz_ros2_control`. The runtime/inference requirements are focused starting constraints, not a verified environment lock. `requirements/delan-original.txt` preserves the separate original training snapshot.

The default local Python environment has NumPy 2.2.6 and Matplotlib 3.5.1. Importing plotting fails because those binaries are incompatible. No host dependencies or existing virtual environments were modified. The focused requirements constrain NumPy below 2 for the original Matplotlib/PyTorch stack.

Build, CLI/launch loading, offline regression checks, data preservation, and offscreen GUI construction were checked. A complete interactive motion/plotting run and a closed-loop comparison were not validated during organization. Record a successful run in a compatible environment, then export its actual Python, ROS, Gazebo, and control-plugin versions before reporting a reproduced release.

The comparison runner retains the original best-effort reset calls. Validate simulator reset behavior on the target installation before interpreting controller comparisons.

## Research provenance

The original DeLaN training code/data, MATLAB references, and notebooks are retained under `research/`. Notebook/MATLAB internal paths remain historical and have not been revalidated after relocation. The standalone inference version was reduced to the latest `inf7.py` implementation, now `tools/infer_torques.py`, with asset/output paths updated.

A complete GRU training procedure, training seeds, dataset splits, checkpoint-selection criteria, and evaluation provenance need author input for training-level reproducibility. The analytical model remains the original CoG-based symbolic model; the retained `compute_christoffel_symbols` helper is not part of the main symbolic Coriolis-vector validation.

## Release metadata still needed

The authors should supply a repository-wide license, consistent package license metadata, real maintainer/contact details, author/university/project citation information, and attribution/redistribution permissions for borrowed code, meshes, models, and data. The main package's placeholder license/contact metadata has not been invented or replaced on the authors' behalf.

No remote push, public publication, or release was performed. Removed intermediate source remains recoverable from Git commit `76212c2` and the original `demo` branch.
