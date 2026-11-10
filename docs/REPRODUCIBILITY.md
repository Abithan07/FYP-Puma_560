# Clone, verify, and reproduce

The `cleanup/publication-prep` branch includes the input datasets and trained models needed by the retained demonstration, original comparison batch, and offline inference workflows. Once this branch is pushed to a **public repository**, readers can obtain these files with an ordinary Git clone. Git LFS and a separate dataset download are not required for this snapshot.

Data availability and identical experiment results are separate questions. The snapshot provides the same input bytes and model assets. A CPU inference example below compares newly calculated values with a bundled reference. Closed-loop Gazebo results still require a compatible ROS/simulator environment and validation of a successful motion run; complete model retraining also needs the missing GRU training procedure and experiment provenance.

## 1. Clone the published branch

After the branch is available on the remote:

```bash
git clone --branch cleanup/publication-prep --single-branch \
  https://github.com/Abithan07/FYP-Puma_560.git
cd FYP-Puma_560
git rev-parse HEAD
python3 tools/verify_data.py --datasets
```

Record the commit ID alongside any experiment. The verification command needs only Python's standard library and checks the published snapshot's file sizes and SHA-256 hashes. It should report **1,852 artifacts verified; 0 failures**. A normal clone checks out all bundled inputs; no Git LFS pull is needed. The published snapshot occupies about **839 MiB uncompressed**, in addition to source, historical tracked assets, Git history, and any installed environment.

| Bundled location | Contents |
| --- | --- |
| `data/trajectories/Joint_states/` | Original controller inputs, including all 520 `J/` references used to select the default 50-path comparison |
| `data/trajectories/Joint_states_601_1120/` | Retained 520-trajectory collection |
| `data/trajectories/Test_data/` | Original test inputs |
| `data/trajectories/demo/` | Saved presentation references, angles, XYZ, and path metadata |
| `data/datasets/` | Retained datasets, including offline inference inputs |
| `research/delan/data/` | Retained DeLaN training CSVs, packed dataset, and checkpoints |
| `models/` | Default DeLaN checkpoint, GRU checkpoint, scaler, and retained alternatives |
| `data/reproduction/` | Numerical CPU inference reference and its interpretation |

[`data/DATASET_MANIFEST.csv`](../data/DATASET_MANIFEST.csv) is the current published snapshot. [`data/MANIFEST.csv`](../data/MANIFEST.csv) is the older preservation inventory: its `tracked` column records whether a file was tracked **before cleanup**, rather than its current publication status. That historical manifest also lists local experiment logs/build records that are not needed as runtime inputs and are not all published. Therefore `--all` is an archival preservation check, not a fresh-clone acceptance check.

New experiment outputs remain ignored by Git. Modifying an existing tracked input, including the demo's path-history CSV, changes the published snapshot and can make the checksum check fail. Verify before running experiments, and keep the commit ID for the baseline.

The repository's `.gitattributes` preserves scientific artifact bytes across Git line-ending settings. Wide numerical CSVs are treated as binary diffs to keep code reviews manageable; their contents remain ordinary readable CSV files.

## 2. Install a compatible environment

The tested host is **Ubuntu 22.04.5, x86_64, Python 3.10.12, ROS 2 Humble, and Gazebo Fortress**. Install ROS Humble and the simulator/control dependencies listed in the [runtime prerequisites](../RUN_GUIDE.md#3-install-or-repair-the-runtime), including `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`, ros2_control/controllers, RViz, Xacro, Tk, and GNOME Terminal for the batch runner. Installing this Python environment does not install those OS/ROS packages.

After those prerequisites are present, from the clone root:

```bash
source /opt/ros/humble/setup.bash
source arm_bot/scripts/desktop_environment.sh
/usr/bin/python3 -m venv --system-site-packages .venv-ros
.venv-ros/bin/python3 -m pip install -r requirements/reproduce.txt
bash arm_bot/setup_runtime.sh
source arm_bot/scripts/runtime_environment.sh
python3 tools/check_runtime.py
```

[`requirements/reproduce.txt`](../requirements/reproduce.txt) pins the observed top-level CPU inference/scientific versions. ROS, native Qt, and transitive dependencies are not fully locked. This is a reproducibility starting point, not a claim that a blank-machine installation has been validated. The project overlay uses system-site packages for ROS; every new installation must pass the runtime check.

The local ROS/simulator package versions recorded during preparation are:

| Package | Recorded version |
| --- | --- |
| `ignition-fortress` | `1.0.3-2~jammy` |
| `libignition-gazebo6` | `6.18.0-1~jammy` |
| `ros-humble-ros-gz-sim` | `0.244.26-1jammy.20260907.235058` |
| `ros-humble-ros-gz-bridge` | `0.244.26-1jammy.20260915.210000` |
| `ros-humble-gz-ros2-control` | `0.7.21-1jammy.20261005.230213` |
| `ros-humble-ros2-control` | `2.54.3-1jammy.20261005.220306` |
| `ros-humble-ros2-controllers` | `2.54.1-1jammy.20261005.222410` |

These are observations of the tested installation. Their exact binary builds may not remain available from package repositories.

## 3. Reproduce a numerical output without simulated motion

After the terminal/runtime setup above, run the retained offline predictor on bundled trajectory 461:

```bash
export PUMA_REPRO_OUTPUT="$(mktemp /tmp/puma-reproduction.XXXXXXXX.csv)"
MPLBACKEND=Agg JAX_PLATFORMS=cpu CUDA_VISIBLE_DEVICES='' \
  OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  python3 tools/infer_torques.py 461 --output "$PUMA_REPRO_OUTPUT"
```

This calculates a labeled-row CSV at the temporary path and writes four PNGs under `data/results/inference/plots/`. The CPU and thread settings make this example consistent with the prepared reference; they do not change the production controller settings. Default residual clipping/ramping is retained.

Compare every signal against the bundled numerical reference:

```bash
python3 - <<'PY'
import os
import numpy as np
import pandas as pd

expected = pd.read_csv('data/reproduction/path_461_expected.csv', header=None, index_col=0)
actual = pd.read_csv(os.environ['PUMA_REPRO_OUTPUT'], header=None, index_col=0)
assert expected.index.tolist() == actual.index.tolist(), 'Signal labels differ'
assert expected.shape == actual.shape, 'Sample counts differ'
assert np.isfinite(expected.to_numpy(dtype=float)).all(), 'Reference contains nonfinite values'
assert np.isfinite(actual.to_numpy(dtype=float)).all(), 'Output contains nonfinite values'
np.testing.assert_allclose(actual.to_numpy(dtype=float), expected.to_numpy(dtype=float),
                           rtol=1e-5, atol=1e-6, equal_nan=False)
print('PASS: all reference states and predicted torques match within tolerance.')
PY
```

The reference was produced with the retained code and default trained assets. It is a regression/reproduction target, not an independent torque-accuracy measurement. Numeric values are compared with tolerance; PNG bytes, fonts, rendering, and device-dependent floating-point details are not an exact-match contract. A failure should be investigated against the recorded environment before changing the reference.

## 4. Run the simulator and controller comparisons

With the same environment configured:

```bash
python3 tools/verify_data.py --datasets
python3 tools/check_repository.py
python3 -m unittest discover -s tests -v
bash arm_bot/all.sh --check
bash arm_bot/run_demo.sh
```

The comparison preflight now finds the original default inputs in the clone. To run all three algorithms, close the demo/simulation processes, then use `bash arm_bot/all.sh`. Follow the [GUI instructions](../RUN_GUIDE.md#5-use-the-presentation-gui), [manual controller selection](../RUN_GUIDE.md#6-choose-and-run-a-controller), and [comparison instructions](../RUN_GUIDE.md#8-run-automatic-comparisons) for a controlled experiment.

The committed data and weights remove missing-input differences. Matching closed-loop plots and tracking statistics also requires matching controller settings, input paths, simulator initialization/reset behavior, timestep, and environment. Exact log bytes/timestamps and identical errors across machines are not guaranteed. A full interactive motion and closed-loop comparison have not been certified by the publication preparation checks. Do not present the offline inference comparison as that validation.

Full retraining is a further level of reproducibility. The DeLaN research sources/data are included, but the complete GRU training recipe, original splits/seeds, and checkpoint-selection provenance still require author input. See [publication notes](PUBLICATION_NOTES.md).

### Checks completed on a separate clone

The data release at commit `787c63b` was cloned into a separate temporary directory with independent Git objects, without supplying unpublished local datasets. The clone reused the author's installed Python/ROS dependencies through an explicit interpreter selection; it was not a blank OS installation.

| Check in the separate clone | Result |
| --- | --- |
| Published snapshot verification | 1,852 assets passed |
| Original tracked preservation check | 719 artifacts passed |
| Source syntax/XML checks | 32 Python, eight shell, seven XML files passed |
| Regression suite | 36 tests passed |
| Original comparison preflight | All 50 selected trajectories passed |
| ROS package build from the clone | Passed |
| CPU inference versus bundled reference | All 16 signals × 1,148 finite samples passed; maximum numerical difference was zero on this host |
| Working tree after checks | Clean; generated outputs remained ignored |

## 5. Make the prepared branch accessible

From the author's checkout, publish the prepared local branch when ready:

```bash
git push -u origin cleanup/publication-prep
```

Anonymous cloning requires the repository to be public. This preparation does not change remote visibility, push the branch, or issue a public release. The existing license/attribution metadata still needs the author decisions listed in the publication notes.

The snapshot uses regular Git files; its files fit within [GitHub's documented file-size limits](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github). If future datasets grow substantially, distribute versioned data separately with hashes or adopt Git LFS for those new assets.
