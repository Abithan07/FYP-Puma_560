"""ROS-independent checks of trajectory boundaries and inverse dynamics."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def load(name, relative_path):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


trajectory = load('puma_trajectory', 'arm_bot/src/scripts/Demo_controller/puma_trajectory_generator.py')
dynamics = load('puma_dynamics', 'arm_bot/src/scripts/inverse_dynamics_model.py')


class NumericalTests(unittest.TestCase):
    def test_minimum_jerk_endpoints_are_at_rest(self):
        position, velocity, acceleration = trajectory.min_jerk_profile(np.array([0., 6., 12.]), 12.)
        np.testing.assert_allclose(position, [0., 0.5, 1.])
        np.testing.assert_allclose(velocity[[0, -1]], 0., atol=1e-12)
        np.testing.assert_allclose(acceleration[[0, -1]], 0., atol=1e-12)

    def test_static_torque_is_gravity(self):
        q = np.deg2rad([30., 20., 100.])
        torque, _, coriolis, gravity = dynamics.compute_inverse_dynamics(q, np.zeros(3), np.zeros(3))
        np.testing.assert_allclose(coriolis, 0., atol=1e-12)
        np.testing.assert_allclose(torque, gravity)

    def test_inertia_is_symmetric_and_positive(self):
        for angles in ([0., 45., 135.], [120., -30., 80.], [-60., 0., 160.]):
            matrix = dynamics.compute_D_matrix(np.deg2rad(angles))
            np.testing.assert_allclose(matrix, matrix.T, atol=1e-12)
            self.assertTrue(np.all(np.linalg.eigvalsh(matrix) > 0))

    def test_inverse_dynamics_decomposition(self):
        q, qd, qdd = np.array([0.2, 0.4, 1.2]), np.array([0.1, -0.2, 0.3]), np.array([0.2, 0.5, -0.1])
        torque, matrix, coriolis, gravity = dynamics.compute_inverse_dynamics(q, qd, qdd)
        np.testing.assert_allclose(torque, matrix @ qdd + coriolis + gravity)
