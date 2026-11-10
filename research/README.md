# Retained research source

The final presentation and comparison workflows are documented in [the run guide](../RUN_GUIDE.md). Its [research section](../RUN_GUIDE.md#14-use-the-research-references) explains the training data contract, a separate experiment environment/copy, and validation limits. This directory retains research material that supports the project but is not an alternative final launcher.

- `delan/`: original DeLaN/black-box training and CSV-to-pickle code, with its original `data/` layout preserved.
- `matlab/`: original MATLAB trajectory/dynamics references.
- `notebooks/`: original notebooks grouped by their former repository paths.

The training scripts use paths relative to the current working directory. Run them from `research/delan/` in an appropriate training environment. Their original dependency snapshot is `requirements/delan-original.txt`. Training may update the checkpoints in `delan/data/`; preserve a separate copy when running new experiments.

Historical paths inside MATLAB files and notebook cells were retained as provenance. These research notebooks and MATLAB references have not been revalidated as runnable workflows after relocation. Their file contents were preserved byte-for-byte. A complete GRU training recipe, seeds, and train/test provenance still need to be supplied for training-level reproducibility.

Intermediate Python run scripts and obsolete simulator packages were removed from the working tree. They remain recoverable from Git commit `76212c2` and the earlier `demo` history.
