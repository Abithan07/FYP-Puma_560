"""Locate repository assets for direct scripts and ROS symlink installs.

Set PUMA_PROJECT_ROOT when running from a copied ROS install outside the checkout.
PUMA_MODEL_DIR and PUMA_LOG_DIR optionally override model and comparison log paths.
"""

import os
from pathlib import Path


def find_project_root(start=None):
    """Find the checkout without depending on its name or current directory."""
    override = os.environ.get('PUMA_PROJECT_ROOT')
    if override:
        root = Path(override).expanduser().resolve()
        if not (root / 'arm_bot' / 'src' / 'package.xml').is_file():
            raise RuntimeError(f'PUMA_PROJECT_ROOT is not a project checkout: {root}')
        return root

    location = Path(start or __file__).resolve()
    for parent in (location, *location.parents):
        if (parent / 'arm_bot' / 'src' / 'package.xml').is_file():
            return parent
    raise RuntimeError('Cannot locate project assets. Set PUMA_PROJECT_ROOT to the checkout.')


PROJECT_ROOT = find_project_root()
SCRIPTS_DIR = PROJECT_ROOT / 'arm_bot' / 'src' / 'scripts'
MODEL_DIR = Path(os.environ.get('PUMA_MODEL_DIR', PROJECT_ROOT / 'DNN_test')).expanduser().resolve()
LOG_DIR = Path(os.environ.get('PUMA_LOG_DIR', SCRIPTS_DIR / 'logs')).expanduser().resolve()
