# Dependencies, reproducibility, and publication notes

## Environment represented by this code

The main workspace uses ROS 2 Humble and Python 3.10. Its launch files refer to `ros_gz_sim`, `ros_gz_bridge`, and `gz_ros2_control`; these packages were found in the local ROS installation during cleanup. The source and older guides describe Gazebo Fortress, while plugin/library naming reflects the installed `gz` integration. Keep the original simulator environment for the first reproduction and record its actual versions.

Runtime ROS dependencies are declared in `arm_bot/src/package.xml`. Missing declarations for launch support, sensor messages, trigger services, clock messages, NumPy, and SymPy have been added. Model inference also needs JAX/Haiku, PyTorch, dill, joblib, and scikit-learn. The GUI uses PyQt5; the demo plotter selects TkAgg, so it also needs a working Tk installation and desktop display.

`requirements/runtime.txt` and `requirements/dnn.txt` provide focused starting constraints based on the imports and dependency records in this repository. They are **not a validated environment lock**. The original `DNN_test/DeLaN/requirements-cpu.txt` and the package selections in `DNN_test/install_deps.sh` describe different environments and remain available for provenance. The latter no longer overwrites and re-executes itself.

For an isolated environment, using the same Python minor version as ROS:

```bash
# From the repository root
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages /tmp/puma-review-env
source /tmp/puma-review-env/bin/activate
python3 -m pip install -r requirements/dnn.txt
```

This is a setup procedure for a new environment, not a command run during cleanup. System ROS packages, Gazebo plugins, Tk, and GNOME Terminal are separate system dependencies. Installed ROS executables may use `/usr/bin/python3` through their shebang; verify which interpreter they use before treating a virtual environment as the runtime for `ros2 run`. Direct `python3` execution uses the active environment.

### Observed compatibility issue

The default local Python environment reports NumPy 2.2.6 and Matplotlib 3.5.1. Importing `matplotlib.pyplot` fails with `_ARRAY_API` / `numpy.core.multiarray` errors. This was reproduced before changing any dependencies. The focused requirements constrain NumPy below 2 for the existing Matplotlib/PyTorch generation. Disabling user-site packages allows the system NumPy 1.21.5 / Matplotlib 3.5.1 combination to import, but removes other required user packages; it is not a complete DNN environment.

No host packages or existing virtual environments were modified. A fresh compatible environment and an interactive demo/comparison run remain necessary before release. After that run, export the actual dependency versions and record the simulator/plugin versions alongside the results.

## Data and models

The baseline Git tree contains 915 files, including 458 CSVs, 167 console logs, and the learned-model assets. Models and recorded research outputs were preserved byte-for-byte. New generated logs, build directories, virtual environments, and Python caches are ignored.

Important inputs are local but not tracked:

- `arm_bot/src/scripts/Joint_states/`, including the `J/` trajectories used by the original 50-path comparison;
- most of `Dataset/`, including trajectories expected by some root-level and `id_pipeline/` commands;
- the large `Joint_states_601_1120/` dataset, excluded by the original ignore rule.

A fresh clone can generate new demo trajectories. Reproducing the original batch requires providing the original CSVs. The single tracked `Dataset/Joint_states/path_662_joint_states.csv` does not replace the full batch.

The default model set is:

```text
DNN_test/fyp_jax_delan_50.jax
DNN_test/best_GRUResidual.pt
DNN_test/feature_scaler.pkl
```

Keep the scaler and network checkpoints together. The cleanup does not retrain, rescale, convert, or substitute them. Training seeds, full training-data provenance, and a complete GRU training recipe need to be supplied by the project authors for training-level reproducibility.

## Known research and legacy limitations

- The main simulation runs at 100 Hz. `id_pipeline/` and `1000Hz_setup_guide.md` describe a separate 1000 Hz experiment; their settings are not applied automatically.
- `id_pipeline/config/pipeline.yaml` is a reference configuration. Neither pipeline script implements a YAML argument.
- `arm_bot/src/scripts/process_dataset.py` retains its historical acceleration calculation: velocity differences without division by elapsed time. This cleanup changes its directories only. Review that calculation before using its output as physical acceleration for a publication.
- `inverse_dynamics_model.py` retains the original finite-difference `compute_christoffel_symbols` helper. The main CTC path uses the independently constructed symbolic Coriolis vector. The helper is outside the numerical validation performed here.
- Historical sweep/stabilization scripts import `trajectory_logger`, whose Python source is absent from the tracked tree. Removed `.pyc` caches do not replace that missing source. Restore it from the original project if those experiments are needed.
- Some earlier scripts and MATLAB files retain machine-specific paths and working-directory assumptions. The final demo, comparison controllers, root trajectory defaults, and ID pipeline defaults received relocation fixes; the source inventory identifies the remaining earlier experiments.
- Some package manifests still contain placeholder maintainer/license metadata. The older simulator copies also use different ROS integration names.
- Gazebo reset calls and RViz marker clearing in the comparison runner retain their original best-effort behavior. Their effect needs checking on the target simulator before comparing published runs.

These items are recorded instead of changing numerical behavior during a cleanup of a working project.

## Release information the authors must supply

Before making a public release, resolve:

1. A repository-wide license and consistent package license metadata. The main package has a placeholder; `arm_bot_new` separately declares MIT. This cleanup does not select a license on the authors' behalf.
2. Author/maintainer names, contact details, university/project title, and citation information.
3. Attribution and redistribution permission for borrowed template code, meshes, datasets, and model code.
4. The exact datasets/splits, checkpoint selection, training configuration, and evaluation procedure supporting each reported result.
5. A reproducible environment snapshot and a successful end-to-end demo/controller comparison.

No remote repository, release, or publication was created by this cleanup. The branch is prepared for code review with these remaining items visible.
