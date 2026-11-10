# Preserved project data

Existing data was relocated without changing its contents. `MANIFEST.csv` records 4,106 protected artifacts, their original/current paths, SHA-256 hashes, sizes, and whether they were originally tracked by Git. This includes local-only datasets/results as well as tracked research artifacts. Some entries live in `models/`, `research/`, or `docs/figures/` rather than under this directory.

| Location | Contents |
| --- | --- |
| `trajectories/Joint_states/` | Original local controller inputs, including the `J/` batch |
| `trajectories/Joint_states_601_1120/` | Original large local dataset |
| `trajectories/Test_data/` | Original test trajectories |
| `trajectories/demo/` | Preserved and newly generated demo trajectories |
| `datasets/Dataset/` | Original `Dataset/` contents |
| `datasets/dnn/` | Original standalone-inference input data |
| `results/demo/` | Demo controller CSVs |
| `results/comparison/` | Detailed comparison controller CSVs |
| `results/pid/` | PID CSVs and figures |
| `results/console/` | Existing/new batch terminal logs |
| `results/raw/` | New triggered logger outputs |
| `results/analysis/` | DNN/CTC analysis results |
| `results/inference/` | Standalone inference predictions and figures |
| `retained/` | Other original experiment data and generated records, preserving original path suffixes |
| `retained_assets/` | Mesh assets from removed experiments and unused robot variants |

Original notebooks were moved to `research/notebooks/`, MATLAB references to `research/matlab/`, and DeLaN training data to `research/delan/data/`.

The original local-only files remain local-only; a fresh clone does not include them. All originally tracked protected artifacts are retained in Git at their new locations. New outputs under `data/` and training data directories are ignored unless deliberately added.

Verify the original tracked artifacts with `python3 tools/verify_data.py`, or every original local artifact with `python3 tools/verify_data.py --all`.
