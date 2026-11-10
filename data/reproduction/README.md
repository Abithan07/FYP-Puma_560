# Offline inference reference

`path_461_expected.csv` contains the numerical output of `tools/infer_torques.py 461` with the default checkpoint/scaler set and default residual clipping/ramping. It was calculated on the recorded CPU runtime on 2026-11-10. The inference implementation was unchanged from source commit `50b040a`.

Each labeled row is one signal over time: `t`, desired joint positions/velocities/accelerations, `tau_delan_j1..3`, and final `tau1..3`. Values are predicted torques and reference states, rather than measured Gazebo tracking or experimental ground truth. All inputs and model assets are hashed in `../DATASET_MANIFEST.csv`.

Follow the [clone/reproduction guide](../../docs/REPRODUCIBILITY.md#3-reproduce-a-numerical-output-without-simulated-motion) to compute a fresh output and compare its numeric values with `rtol=1e-5`, `atol=1e-6`. Thread/device settings and the recorded environment are included there. Plot-image bytes are not part of the comparison contract.
