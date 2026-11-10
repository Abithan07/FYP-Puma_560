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

## Startup fix — 2026-11-10

Investigated the reported `libpthread.so.0` / `__libc_pthread_init` / `GLIBC_PRIVATE` crash after a successful build. The VS Code Snap terminal exported GTK and GIO paths into its Snap runtime. The inherited Snap GTK module's RPATH explicitly includes `/snap/core20/current/lib/x86_64-linux-gnu`; loading it directly reproduced a `GLIBC_PRIVATE` libpthread failure.

Added `arm_bot/scripts/desktop_environment.sh` and sourced it from the demo/comparison launchers and their simulation/trajectory/terminal helpers. It strips Snap-specific desktop/plugin/library paths from launched processes while preserving ROS/system/custom non-Snap paths. No system libraries, Python packages, terminal settings, datasets, or controller code were changed.

Added regression coverage for inherited Snap paths, mixed ROS/Snap library paths, custom system paths, unset variables, repeated sourcing, and cleanup before both build and GUI launch. The suite now passes **26 tests**. Source checks pass for **27 Python files, six shell files, and seven XML/Xacro/world files**. The actual final GUI initializes with the cleaned environment in an offscreen Qt test. Desktop interaction and a full simulation remain outside that test.

The final run guide documents automatic cleanup and the equivalent environment setup for manual commands. The existing NumPy/Matplotlib issue is separate from this startup fix.

## NumPy/inference runtime fix — 2026-11-10

The reported run used host NumPy 2.2.6 with PyTorch 2.1.0+cpu and Matplotlib 3.5.1 binaries compiled for NumPy 1.x. The GRU tensor-to-NumPy conversion failed after the history window filled, and the plotter could not import its compiled extension. Successful checkpoint loading alone had not tested that conversion boundary.

Created a new project-only `.venv-ros` overlay with NumPy 1.26.4 and scikit-learn 1.6.1, matching the version recorded in the default saved scaler. It reuses installed ROS/inference dependencies through system-site packages. Host/user-site NumPy 2.2.6, other global packages, the earlier `.venv`, all weights, and all recorded data remain unchanged. `arm_bot/setup_runtime.sh` recreates this overlay from `requirements/ros-overlay.txt`.

Added runtime selection shared by the shell launchers and explicit interpreter inheritance for logger subprocesses. Added a preflight that checks real tensor/array conversion, JAX, required imports, and Matplotlib rendering before GUI/motion launch. `PUMA_PYTHON` supports an explicit compatible interpreter. The DNN engines independently validate the conversion before model initialization.

Corrected the controller error path: an inference exception now cancels the timer, sends zero torque, stops/flushes logging, and returns a failure exit code. It no longer advances through zero-torque samples and prints a completed trajectory. The generator wrapper therefore stops before plotting a failed execution. Successful inference/control equations and gains are unchanged. The demo plotter now honors `MPLBACKEND`, retaining TkAgg as its default while allowing headless verification.

Validation in the repaired runtime:

- The actual demo model produced finite predictions for all 1,802 samples of the reported `path_127` trajectory, including 1,675 GRU-active steps. The comparison model also passed beyond its 128-sample history window.
- The default scaler loaded under its original scikit-learn 1.6.1 without a version-mismatch warning.
- The actual plotter rendered the reported log to a temporary PNG successfully. The original failed log was preserved and remains an invalid tracking-performance run.
- The original incompatible `/usr/bin/python3` was rejected by preflight before GUI/motion startup.
- All 32 regression tests passed, including real callback abort behavior, interpreter selection, and rejection of a failed startup check.
- The ROS package rebuilt successfully with the new shared runtime helper installed.

The model checks ran offline with bounded CPU threads; no torques were sent to a simulator or physical arm during verification. They establish inference/conversion and plotting functionality, not closed-loop tracking performance. Stop any process from the earlier failed run and relaunch to use the new runtime.

## Complete usage guide — 2026-11-10

Expanded `RUN_GUIDE.md` into the main user manual with navigation and complete workflows: runtime prerequisites/bootstrap, per-terminal setup, GUI operation and limits, manual PID/CTC/DNN selection, all five DNN modes, direct/waypoint/conditions-CSV generation, filename conversion for automatic comparisons, raw logging, result locations/formats, analysis exports, standalone inference, configuration, verification, research references, stopping, and troubleshooting. The guide explicitly distinguishes the GUI's fixed PID+DNN workflow, the comparison controllers, the DNN publisher's PID ablation, and the standalone PID baseline.

Added workflow links to the root/workspace READMEs and research notes. Documented the original/local-only dataset distinction, temporary versus retained experiment inputs, failed-log interpretation, analysis-command output side effects, and research training limitations. Runtime/controller implementation and recorded data were not changed by this documentation update.

Checked current CLI help and documented Python flags against their argument parsers. All local links/section anchors and fenced shell syntax passed validation. Executed the direct/conditions-CSV generation examples, a generated-reference batch preflight, DNN/CTC report exports, paired analysis, the CTC text-summary command, and the shared analytical-model example using scratch files. Analysis generated 27 temporary PNGs. These checks did not start simulated motion or retrain models.

## File-by-file code reference — 2026-11-10

Added [CODE_REFERENCE.md](CODE_REFERENCE.md), explaining all **50 retained code files**: 31 Python files, eight shell scripts, three MATLAB scripts, and eight notebooks. It describes each file's purpose, important functions/classes, inputs, outputs, and role in the presentation, comparison, analysis, or research workflows. It also covers 18 build/configuration/dependency support files and the preservation manifest, with a presentation-flow diagram, ROS topic connections, units, and CSV orientation notes.

Documented the differences between the demo and comparison DNN publishers, cached versus online inference, CTC versus torque-domain PID feedback, the separate PID baseline, and actual plotting/logging behavior. Research entries explain historical path assumptions, the training packer's row-order contract, differing analytical models, and notebook processing calculations. Linked the reference from the root/workspace READMEs, run guide, source map, and research README.

Checked descriptions against source, verified that every tracked Python/shell/MATLAB/notebook file has an entry and source link, and validated local links/section anchors and whitespace. This update changes documentation only; executable code, datasets, checkpoints, and notebook contents were not modified. No simulation or training run was needed for this documentation change.

## Shareable dataset snapshot — 2026-11-10

At the user's request, changed the earlier local-only input policy: deliberately included all retained trajectory/dataset collections, DeLaN training data, model checkpoints/scalers, and a new numerical reference in the publication branch. The current snapshot contains **1,852 files totaling 838.74 MiB**, including 1,470 newly tracked files. Historical outputs/build logs outside this selection remain local unless already tracked. No dataset values or model weights were transformed. The current demo path-history file includes later runs and is published as it exists now; the earlier preservation manifest remains unchanged.

Added `data/DATASET_MANIFEST.csv` with groups, paths, sizes, and SHA-256. Extended `tools/verify_data.py` with `--datasets` for clone verification, size checks, and manifest-error reporting; the original/default and `--all` preservation selections remain available. Added four regression tests covering previously untracked input selection, missing/resized/corrupted files, historical selection, and unlisted outputs. All 36 tests pass; the 1,852 published assets and 719 originally tracked artifacts pass verification. Source syntax checks pass for 32 Python, eight shell, and seven XML files.

Added `.gitattributes` to preserve scientific artifact bytes across checkout settings and keep wide numerical CSV diffs manageable. The publication index uses LF text. New run outputs remain ignored by `.gitignore`. Dataset/model file sizes fit the ordinary Git file limit; no LFS dependency or external data host was introduced.

Added `docs/REPRODUCIBILITY.md` with branch-clone, dataset verification, environment setup, offline numerical comparison, simulation commands, and publication instructions. Added `requirements/reproduce.txt` with observed top-level CPU/runtime versions and recorded native ROS/simulator versions in the guide. Updated the README, run guide, source reference, data notes, and publication/architecture notes to describe the bundled inputs and distinguish the historical preservation inventory.

Calculated `data/reproduction/path_461_expected.csv` using the retained offline predictor and default models on the CPU: 1,148 timesteps, with four headless plots generated locally. This is a reproducible prediction reference rather than closed-loop tracking or independent torque-accuracy evidence. Full blank-machine dependency installation, interactive closed-loop comparisons, and GRU retraining remain unvalidated. No remote push or visibility change was made.
