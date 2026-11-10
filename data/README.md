# Preserved project data

Existing data was relocated without changing its contents. `MANIFEST.csv` records 4,106 protected artifacts, their original/current paths, SHA-256 hashes, sizes, and whether they were originally tracked by Git. This includes local-only datasets/results as well as tracked research artifacts. Some entries live in `models/`, `research/`, or `docs/figures/` rather than under this directory.

The publication branch now deliberately includes the runtime input datasets, training data, model assets, and an offline numerical reference: **1,852 files, 838.74 MiB uncompressed**, indexed in [DATASET_MANIFEST.csv](DATASET_MANIFEST.csv). These files are ordinary Git objects and arrive with a normal clone of this branch after it is pushed. Follow the [clone and reproduction guide](../docs/REPRODUCIBILITY.md).

| Location | Contents |
| --- | --- |
| `trajectories/Joint_states/` | Original local controller inputs, including the `J/` batch |
| `trajectories/Joint_states_601_1120/` | Original large local dataset |
| `trajectories/Test_data/` | Original test trajectories |
| `trajectories/demo/` | Preserved and newly generated demo trajectories |
| `reproduction/` | Bundled CPU torque-prediction reference and interpretation |
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

All current files in `trajectories/` and `datasets/`, the retained DeLaN training-data tree, and all model checkpoints/scalers are included in the published snapshot. Historical result/build logs outside that selection remain local unless they were already tracked. New generated inputs/results are ignored unless deliberately added; changes to existing tracked inputs still appear in Git.

Verify everything in the published snapshot with:

```bash
python3 tools/verify_data.py --datasets
```

The publication manifest records the current paths, sizes, and hashes. It includes trajectory references generated since the earlier preservation inventory and the current demo path-history file. The old manifest remains unchanged as a historical record: its `tracked=False` values describe the pre-cleanup state, even for files now published. `python3 tools/verify_data.py` checks originally tracked artifacts; `--all` checks the entire older archive, including unpublished local logs, and is not expected to pass in a fresh clone.
