#!/usr/bin/env python3
"""Check the actual ROS/inference/plotting runtime before launching a motion."""
import argparse
import importlib
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'arm_bot/src/scripts/shared'))
from runtime_compatibility import require_torch_numpy


def check_runtime():
    import numpy as np
    if tuple(int(value) for value in np.__version__.split('.')[:2]) < (1, 26) or int(np.__version__.split('.')[0]) >= 2:
        raise RuntimeError(f'NumPy {np.__version__} is incompatible with this project runtime; use NumPy 1.26.4.')
    import torch
    require_torch_numpy(torch, np)
    for name in ('rclpy', 'pandas', 'scipy', 'sympy', 'haiku', 'joblib', 'sklearn', 'dill', 'PyQt5.QtWidgets', 'tkinter'):
        importlib.import_module(name)
    import jax
    jax.numpy.asarray(np.zeros(3)).block_until_ready()
    # Draw with the actual compiled plotting extension, without opening a display.
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    figure = Figure()
    figure.subplots().plot([0, 1], [0, 1])
    FigureCanvasAgg(figure).draw()
    return np.__version__, torch.__version__


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--quiet', action='store_true')
    args = parser.parse_args()
    try:
        numpy_version, torch_version = check_runtime()
    except Exception as error:
        print(f'Runtime check failed: {error}\nRun bash arm_bot/setup_runtime.sh, then restart the launcher.', file=sys.stderr)
        return 1
    if not args.quiet:
        print(f'Runtime OK: {sys.executable}\nNumPy {numpy_version}; PyTorch {torch_version}; JAX, ROS, and Matplotlib checks passed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
