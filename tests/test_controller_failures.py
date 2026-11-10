"""A failed inference must stop the trajectory without claiming completion."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
HAS_ROS = importlib.util.find_spec('rclpy') is not None


@unittest.skipUnless(HAS_ROS, 'Source ROS Humble to test the actual ROS controller callbacks.')
class ControllerFailureTests(unittest.TestCase):
    def check_failure(self, group):
        path = ROOT / f'arm_bot/src/scripts/{group}/torque_publisher_dnn.py'
        spec = importlib.util.spec_from_file_location('failure_' + group, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        node = module.DNNTorquePublisher.__new__(module.DNNTorquePublisher)
        node.current_idx = 128
        node.n_points = 256
        node.trajectory_active = True
        node.trajectory_error = None
        node.trajectory_timer = Mock()
        node.get_logger = Mock(return_value=Mock())
        node.compute_torques = Mock(side_effect=RuntimeError('Numpy is not available'))
        node.last_ctrl_wall_t = None
        node.last_joint_state_wall_t = None
        node.dt = 0.01
        for index in range(1, 4):
            setattr(node, f'msg{index}', module.Float64MultiArray())
            setattr(node, f'pub{index}', Mock())
        node.trajectory_callback()
        self.assertFalse(node.trajectory_active)
        self.assertEqual(node.current_idx, 128)
        self.assertIn('Numpy is not available', node.trajectory_error)
        node.trajectory_timer.cancel.assert_called_once()
        for index in range(1, 4):
            publisher = getattr(node, f'pub{index}')
            publisher.publish.assert_called_once()
            self.assertEqual(list(publisher.publish.call_args.args[0].data), [0.0])
        messages = [args[0] for args, _ in node.get_logger.return_value.info.call_args_list]
        self.assertFalse(any('COMPLETED' in message for message in messages))

    def test_demo_inference_failure_stops_motion(self):
        self.check_failure('demo')

    def test_comparison_inference_failure_stops_motion(self):
        self.check_failure('controllers')
