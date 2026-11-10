# Cleanup work report

Date: 2026-11-09

Working branch: `cleanup/publication-prep`

Baseline: `demo` at `f0d13bd` (`Final Demo`)

## Purpose and scope

Prepare the working final-year project for advisor review and a future public release while preserving its control behavior and research artifacts. The baseline contained 915 tracked files and 94 Python sources across the final demo, controller comparison tools, training/inference experiments, multiple simulator workspaces, and offline dataset generators.

The final GUI demo and the main controller comparison flow were traced through launch, robot description, trajectory generation, inference, feedback control, logging, and analysis. A complete Python source inventory and a recommended reading order are provided in [Repository map](REPOSITORY_MAP.md).

## Work completed

### Review documentation and organization

- Added a project-specific root README and replaced the main workspace's template README.
- Added architecture, build/run workflows, a complete Python source inventory, dependency/publication notes, and this report under `docs/`.
- Distinguished the final demo, the DNN/CTC/PID comparison workflow, the separate 1000 Hz pipeline, and earlier experiments.
- Documented CSV layouts and units, default model/scaler relationships, ROS interfaces, log locations, and environment overrides.
- Flagged older quick-start/rate instructions and corrected the YAML configuration comment: the ID scripts do not load that file.
- Kept source and research assets in their existing directories to preserve imports, mesh lookup, and data paths.

### Portable paths and package support

- Added `arm_bot/src/scripts/project_paths.py` to locate the checkout from source or ROS symlink installs.
- Replaced personal absolute paths in the final GUI, demo launcher/wrapper, active DNN/CTC log defaults, modular DNN model defaults, analysis output defaults, root trajectory/batch/CSV utilities, and ID pipeline dataset defaults.
- Added optional `PUMA_PROJECT_ROOT`, `PUMA_MODEL_DIR`, and main DNN/CTC `PUMA_LOG_DIR` overrides. Existing explicit CLI model/log arguments remain available.
- Installed the support module with the ROS controller executables and added missing ROS/Python runtime declarations in the main package manifest.
- Replaced the main package's generic description with its actual purpose. Author/contact/license placeholders remain visible pending author decisions.

### Shell workflow cleanup

- Kept the original 50 path IDs, DNN → CTC → PID order, controller commands, and existing reset/marker calls in `all.sh`.
- Added selected path IDs, `--csv-dir`, `--help`, and `--check` so required data/model files can be checked before simulation starts.
- Replaced the fixed shared `/tmp` status file with per-run temporary state, atomic exit-status reporting, a configurable per-controller timeout, and literal argument passing through a small terminal helper.
- Scoped comparison and legacy-pipeline cleanup to process groups started by those scripts. Removed broad `pkill` calls targeting unrelated ROS/Gazebo/RViz processes.
- Removed obsolete commented implementations from `pipeline.sh` and `dataset_generator.sh`, anchored both to the workspace, and made failures propagate.
- Made legacy path IDs consistently three digits and isolated temporary publisher output per invocation.
- Added bounded legacy readiness/log-write polling, checking all three effort controllers and handling colored CLI output.
- Removed the dependency installer's self-overwrite/re-execution wrapper and tied pip calls to `python3`. Its original package selections were retained; the installer was not executed.

### Repository hygiene and validation support

- Expanded `.gitignore` for colcon output, virtual environments, Python caches, editor state, and generated run outputs.
- Replaced the global PNG ignore with output-specific rules so documentation figures can be versioned.
- Removed four tracked generated `.pyc` files. Historical missing `trajectory_logger` source is explicitly documented.
- Fixed the Python 3 syntax error `start_id = 001` in the earlier `trajectory_generator copy.py` variant.
- Added focused runtime/inference requirement groups, marked as starting constraints rather than a validated lockfile.
- Added a standard-library repository checker and 18 offline regression tests covering path resolution, terminal failures/arguments, comparison ordering/failure handling, legacy dataset IDs/failures, and numerical invariants.

## Preservation evidence

- Compared syntax trees against the baseline: **178 functions/methods across seven core controller, trajectory, and dynamics modules are unchanged**. This includes DNN inference implementations, the PID trajectory controller, CTC calculations, the final demo generator, and symbolic inverse dynamics. Module/class path defaults and imports were changed separately.
- Ran the original and cleaned final-demo trajectory generators in temporary directories for direct and waypoint cases. **All five CSV outputs per case were byte-identical**.
- Retained controller gains, control periods, model architectures, feedback/stabilization calculations, torque limits, and robot descriptions.
- Verified 715 original research artifacts against their Git blob hashes: model/scaler files, recorded CSV data, meshes, notebooks, MATLAB scripts, plots, and console logs remain byte-identical. No training or checkpoint conversion was performed.

## Validation performed

| Check | Result |
| --- | --- |
| Repository syntax/XML check | Passed: 101 Python, 10 shell, 32 XML/Xacro/world files |
| Offline regression suite | Passed: 18 tests |
| Main `arm_bot` package build in isolated `/tmp` output directories | Passed, with `BUILD_TESTING=OFF` |
| Installed DNN and CTC `--help` commands | Passed |
| DNN/CTC Python module `--help` invocation | Passed |
| Gazebo launch description argument loading | Passed; no simulator processes started |
| Main robot Xacro expansion | Passed: six links, five joints |
| Original 50-trajectory comparison preflight | Passed against local input/model files |
| GUI construction with offscreen Qt | Passed with system packages; no simulator processes started |
| Original/current direct and waypoint trajectory generation | Byte-identical CSV outputs |
| Git whitespace check | Passed |

Commands and limitations are documented in [Workflows](WORKFLOWS.md). Syntax checks do not validate every historical experiment. The package build did not run the original copyright/style lint suite.

## Remaining work before public release

A full interactive GUI run and a complete closed-loop controller comparison were **not validated** during cleanup. The default Python environment has an existing NumPy 2.2.6 / Matplotlib 3.5.1 binary incompatibility. Host dependencies were not changed. The project needs an isolated compatible environment, a recorded end-to-end run, and an exported environment snapshot.

The original batch data is present locally but is not tracked in Git. It must be supplied to reproduce the original comparisons. License/maintainer/attribution information and training/evaluation provenance also need author input. Numerical limitations in historical preprocessing and missing legacy logger source are recorded in [Publication notes](PUBLICATION_NOTES.md).

The work is prepared on the separate cleanup branch. No changes were pushed and no public release was created.
