# PUMA 560 — Final Year Project

Final ROS 2 simulation and trajectory tracking for the first three joints of a modified PUMA 560 arm. The retained controllers compare PID, analytical computed torque control (CTC), and learned inverse dynamics using DeLaN with a GRU residual.

**[Follow the complete setup and usage guide](RUN_GUIDE.md).**

```bash
# From the repository root, in the project's compatible desktop environment:
source /opt/ros/humble/setup.bash
bash arm_bot/run_demo.sh
```

`run_demo.sh` builds the main workspace and starts the final presentation GUI. For the DNN → CTC → PID comparison batch:

```bash
source /opt/ros/humble/setup.bash
bash arm_bot/all.sh --check
bash arm_bot/all.sh
```

The batch CSVs are available in the original local checkout; a fresh clone needs those datasets separately. The launchers use the compatible `.venv-ros` runtime automatically. On a fresh checkout, run `bash arm_bot/setup_runtime.sh` first. Runtime setup and data requirements are covered in the run guide.

## Choose a workflow

| Goal | Guide section |
| --- | --- |
| Set up a new checkout or repair Python dependencies | [Runtime setup](RUN_GUIDE.md#3-install-or-repair-the-runtime) |
| Operate the final presentation GUI | [GUI walkthrough](RUN_GUIDE.md#5-use-the-presentation-gui) |
| Choose PID, CTC, DNN, or a DNN ablation mode | [Controller selection](RUN_GUIDE.md#6-choose-and-run-a-controller) |
| Generate direct/waypoint or conditions-CSV references | [Trajectory generation](RUN_GUIDE.md#7-generate-trajectories) |
| Compare all three algorithms on the same input | [Automatic comparisons](RUN_GUIDE.md#8-run-automatic-comparisons) |
| Find logs and export tracking/torque reports | [Outputs](RUN_GUIDE.md#9-find-and-interpret-outputs), [analysis](RUN_GUIDE.md#10-plot-and-analyze-results) |
| Explore training, MATLAB, or notebook references | [Research workflows](RUN_GUIDE.md#14-use-the-research-references) |
| Understand each code file and its role in the project | [File-by-file code reference](docs/CODE_REFERENCE.md) |

The GUI currently uses PID + DNN automatically. Other controllers are selected through the manual commands in the guide; `all.sh` runs all three main controllers.

## Repository layout

| Directory | Contents |
| --- | --- |
| `arm_bot/` | The single final ROS workspace and two user-facing launchers |
| `arm_bot/src/scripts/demo/` | Final GUI, trajectory generator, demo DNN publisher, and plotter |
| `arm_bot/src/scripts/controllers/` | Latest comparison DNN, CTC, and trajectory PID controllers |
| `arm_bot/src/scripts/shared/` | Common paths, triggered logger, inverse dynamics, and RViz drawing |
| `arm_bot/src/scripts/analysis/` | DNN and CTC result analysis |
| `models/` | Original learned-model checkpoints and feature scalers |
| `data/` | Trajectories, datasets, recorded outputs, retained assets, and preservation manifest |
| `research/` | DeLaN training code, original MATLAB references, and notebooks |
| `tools/` | Latest standalone inference and repository/data verification |
| `tests/` | Offline regression checks for the retained workflows |
| `requirements/` | Python dependency groups and the original training dependency snapshot |
| `docs/` | Architecture, source map, publication notes, and cleanup report |

Earlier simulator packages, numbered inference versions 1–6, experimental torque publishers, old dataset launchers, and obsolete run instructions have been removed. Their source is recoverable from Git history. Existing datasets, models, results, and notebooks were preserved and checked before and after relocation.

## Review and verification

- [Architecture](docs/ARCHITECTURE.md)
- [File-by-file code reference](docs/CODE_REFERENCE.md)
- [Retained source map](docs/REPOSITORY_MAP.md)
- [Cleanup work report](docs/CLEANUP_REPORT.md)
- [Publication notes](docs/PUBLICATION_NOTES.md)
- [Data locations and preservation](data/README.md)

```bash
source /opt/ros/humble/setup.bash
source arm_bot/scripts/desktop_environment.sh
source arm_bot/scripts/runtime_environment.sh
python3 tools/check_repository.py
python3 -m unittest discover -s tests -v
python3 tools/verify_data.py
```

The main simulation remains at 100 Hz. The cleanup changes organization, imports, and output paths; the retained control calculations and model weights are unchanged.
