# Final repository organization report

Date: 2026-11-09

Branch: `cleanup/publication-prep`

Reference commits: `f0d13bd` (original Final Demo), `76212c2` (first conservative cleanup)

## Final workflows selected

Git history and the actual GUI/subprocess/ROS dependency chain identify these retained run paths:

- **Presentation:** `arm_bot/run_demo.sh` → final GUI → simulation helper → trajectory wrapper → final demo generator/publisher/plotter.
- **Comparison:** `arm_bot/all.sh` → latest main DNN, CTC, and trajectory PID controllers, in the original order.
- **Standalone inference:** the latest `DNN_test/inf7.py` implementation, now `tools/infer_torques.py`.

The demo and comparison DNN implementations remain separate because their online/precomputed inference behavior differs. Required helper scripts remain with the final workflows.

## Removal and organization

- Retired 78 tracked Python and 6 tracked shell intermediate implementations, including an old launch wrapper replaced in place by the full final launch.
- Removed the earlier `arm_bot_new`, `arm_bot_pos_control`, `arm_controller`, and `DNN_test/ROS2_Gazebo_sim` runtime packages.
- Removed numbered inference versions 1–6, experimental torque publishers, one-/two-joint runs, home/sweep experiments, the earlier modular DNN and PID versions, obsolete dataset pipelines, and duplicated logger code.
- Removed obsolete run instructions, template files, unused final-package configuration/world files, stale generated installs/build products, and editor-specific files.
- Retained the actual final simulator at the sole launch entry point `gazebo.launch.py`.
- Grouped executable source into `demo/`, `controllers/`, `shared/`, and `analysis/` under the main package.
- Moved original models/scalers to `models/`, datasets/results to `data/`, and training/MATLAB/notebook references to `research/`.
- Kept only the four meshes referenced by the final URDF inside the ROS package; preserved other mesh assets with the retained data.
- Updated imports, ROS executable installation, GUI/wrapper paths, logger subprocess paths, data/model defaults, and ignore rules for the final layout.
- Added `run_demo.sh` as the final presentation launcher. Preserved `all.sh` as the final comparison launcher.
- Consolidated current run instructions into [RUN_GUIDE.md](../RUN_GUIDE.md), and rewrote the README, architecture, source map, and publication notes to match.

[Retired tracked paths](REMOVED_FILES.txt) documents the removal decisions. Earlier code remains recoverable from commit `76212c2` and the original `demo` history.

## Data preservation

Before removing or relocating files, recorded **4,106 protected artifacts** totaling approximately **1,488.7 MiB**. Each entry has its original path, current path, file size, tracked status, and SHA-256 hash in `data/MANIFEST.csv`.

All 4,106 passed hash verification after relocation. This includes **719 originally tracked artifacts**, plus the original local-only data. Preserved categories include trajectory/dataset CSVs, models, scalers, pickle datasets, recorded logs, plots, rendered result plots, mesh assets, notebooks, and MATLAB references.

Original tracked artifacts remain tracked at their new paths. Local-only datasets/results remain local-only, so the original batch still needs those inputs when using a fresh clone. Historical data that is not consumed by the final workflows is organized under `data/retained/` or `data/retained_assets/`.

No dataset filtering, weight conversion, retraining, or numerical data transformation was performed.

## Behavior preserved

The retained control and inference calculations, gains, control periods, stabilization behavior, and robot model remain unchanged. Changes to these implementations were limited to file-location handling: imports and subprocess paths, explicit CTC input validation in place of a missing personal default CSV, and output directories.

Direct and waypoint trajectory runs were compared against the original Final Demo generator. All five CSV contents per case matched byte-for-byte after mapping the tracking CSV to its new output location. The generator now places `trajectory_tracking.csv` inside the selected output directory, rather than writing `test_traj.csv` into whichever directory invoked it.

The main ROS package remains at 100 Hz. The removed intermediate 1000 Hz experiment is no longer presented as a final run option.

## Validation

| Check | Result |
| --- | --- |
| Clean ROS package build in new `/tmp` output directories | Passed |
| Offline regression tests | 21 passed |
| Repository syntax checks | 26 Python, 5 shell, 7 XML/Xacro/world files passed |
| Preserved artifact verification | 4,106 passed; zero missing/changed files |
| Original tracked artifact verification | 719 retained at new paths |
| Direct/waypoint original versus final CSV contents | Identical |
| Installed DNN, CTC, PID, and triggered logger CLI loading | Passed |
| Sole final simulator launch argument loading | Passed |
| Robot Xacro expansion and mesh references | Passed; six links and five joints |
| Original 50-path comparison preflight after data relocation | Passed |
| GUI construction with offscreen Qt and relocated helper paths | Passed; no simulation started |
| New presentation launcher build/source/skip-build behavior | Covered by regression tests |
| Documentation links and Git whitespace checks | Passed |

The earlier two legacy dataset-runner tests were removed together with that retired workflow. Five new tests cover the final presentation launcher and direct/waypoint CLI outputs.

## Remaining limitations

A full interactive motion/plotting run and a closed-loop controller comparison were not validated. The existing default Python environment has NumPy 2.2.6 with Matplotlib 3.5.1, causing plotting import failures. No host dependencies or existing virtual environments were changed. Use a compatible environment and the final run guide for the end-to-end validation.

Original model/data training provenance, license/maintainer metadata, and attribution still need author decisions before publication. Research notebook/MATLAB internal paths remain historical; their bytes were preserved, and they are not advertised as final executable workflows. See [publication notes](PUBLICATION_NOTES.md).

The work is local to the cleanup branch. No remote push or public release was made.
