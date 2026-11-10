# Original learned models

The default matching inference set is:

- `fyp_jax_delan_50.jax`: DeLaN checkpoint.
- `best_GRUResidual.pt`: GRU residual checkpoint.
- `feature_scaler.pkl`: saved feature scaler.

All original `_o1`, `_o2`, `_o3`, and alternative scaler artifacts are retained. They are not substituted automatically. File contents match the pre-organization SHA-256 hashes in `data/MANIFEST.csv`.

The DeLaN/black-box training checkpoints remain beside their original training data in `research/delan/data/`. No model was retrained or converted.
