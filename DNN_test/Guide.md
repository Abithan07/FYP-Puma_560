# DNN Test Guide

## 1. Generate Trajectory Data

Open a terminal in:

```bash
~/Desktop/FYP-Puma_560/DNN_test
```

Run:

```bash
python3 datagen.py <id>
```

- `<id>` is the trajectory/path number (example: `603`).

Optional (reproducible random trajectory):

```bash
python3 datagen.py <id> --seed 42
```

Generated files are saved under:

- `~/Desktop/FYP-Puma_560/DNN_test/Data/Angles/`
- `~/Desktop/FYP-Puma_560/DNN_test/Data/XYZ/`
- `~/Desktop/FYP-Puma_560/DNN_test/Data/Trajectory/`

The trajectory summary is updated in:

- `~/Desktop/FYP-Puma_560/DNN_test/summary.csv`

## 2. Predict Torques

Run in the same terminal using the same id:

```bash
python3 inf6.py <id>
```

- Example: `python3 inf6.py 603`

Optional custom output file:

```bash
python3 inf6.py <id> --output Data/path_<id>_joint_states.csv
```

Generated output files are saved in:

```bash
~/Desktop/FYP-Puma_560/DNN_test/Data/
```

The main output file will look like:

- `path_<id>_joint_states.csv`

## 3. Plot Joint States and Torques

Generate plots from the predicted joint states file:

```bash
python3 plot.py <id>
```

- Example: `python3 plot.py 603`

Generated plot file:

- `~/Desktop/FYP-Puma_560/DNN_test/Data/path_<id>_joint_states_data_plot.png`

## Quick Example

```bash
cd ~/Desktop/FYP-Puma_560/DNN_test
python3 datagen.py 603 --seed 42
python3 inf6.py 603
python3 plot.py 603
```