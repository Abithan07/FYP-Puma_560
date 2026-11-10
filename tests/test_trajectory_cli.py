"""Check the retained trajectory command and its output contract after relocation."""
import csv
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / 'arm_bot/src/scripts/demo/trajectory_generator.py'


class TrajectoryCliTests(unittest.TestCase):
    def check_trajectory(self, midpoint):
        with tempfile.TemporaryDirectory(prefix='puma trajectory ') as tmp:
            output = Path(tmp) / 'generated trajectories'
            command = [sys.executable, str(GENERATOR), '--path-id', '1', '--q-end', '[120,-30,80]',
                       '--t-total', '20', '--base-dir', str(output)]
            if midpoint:
                command += ['--q-mid', '[60,10,110]']
            result = subprocess.run(command, cwd=tmp, capture_output=True, text=True, timeout=20)
            self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
            with (output / 'Trajectories/path_001_traj.csv').open(newline='') as handle:
                signals = {row[0]: np.asarray(row[1:], dtype=float) for row in csv.reader(handle)}
            self.assertEqual(set(signals), {'t', 'dp1', 'dp2', 'dp3', 'dv1', 'dv2', 'dv3', 'da1', 'da2', 'da3'})
            np.testing.assert_allclose([signals[f'dp{j}'][0] for j in range(1, 4)], np.deg2rad([0, 45, 135]), atol=1e-7)
            np.testing.assert_allclose([signals[f'dp{j}'][-1] for j in range(1, 4)], np.deg2rad([120, -30, 80]), atol=1e-7)
            self.assertTrue(np.all(np.diff(signals['t']) > 0))
            self.assertTrue(all(np.all(np.isfinite(signal)) for signal in signals.values()))
            self.assertTrue((output / 'trajectory_tracking.csv').is_file())
            self.assertFalse((Path(tmp) / 'test_traj.csv').exists())

    def test_direct_trajectory(self):
        self.check_trajectory(midpoint=False)

    def test_waypoint_trajectory(self):
        self.check_trajectory(midpoint=True)
