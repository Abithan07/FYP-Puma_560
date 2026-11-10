#!/usr/bin/env python3
"""PID home-reset controller for the 3-DOF arm.

This controller drives the robot to the home pose used by the rest of the
pipeline: [0.0°, 45.0°, 135.0°]. It publishes torque commands to the same
per-joint topics as the other controllers so Gazebo actually responds.
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class HomeController(Node):
    def __init__(self):
        super().__init__('home_controller')

        self.dt = 0.01
        self.timeout = 20.0
        self.settle_tolerance_rad = math.radians(0.02)

        self.kp = np.array([50.0, 200.0, 150.0], dtype=np.float64)
        self.ki = np.array([5.0, 25.0, 20.0], dtype=np.float64)
        self.kd = np.array([12.0, 35.0, 10.0], dtype=np.float64)
        self.torque_limits = np.array([100.0, 100.0, 50.0], dtype=np.float64)

        self.home_position = np.deg2rad(np.array([0.0, 45.0, 135.0], dtype=np.float64))

        self.current_joint_pos = np.zeros(3, dtype=np.float64)
        self.current_joint_vel = np.zeros(3, dtype=np.float64)
        self.joint_states_received = False
        self.integral_error = np.zeros(3, dtype=np.float64)

        self.start_time = None
        self.converged_count = 0
        self.convergence_threshold = 10

        self.idx1 = 0
        self.idx2 = 1
        self.idx3 = 2

        self.pub1 = self.create_publisher(Float64MultiArray, '/joint_1_controller/commands', 10)
        self.pub2 = self.create_publisher(Float64MultiArray, '/joint_2_controller/commands', 10)
        self.pub3 = self.create_publisher(Float64MultiArray, '/joint_3_controller/commands', 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.msg1 = Float64MultiArray()
        self.msg2 = Float64MultiArray()
        self.msg3 = Float64MultiArray()

        self.get_logger().info('Home controller initialized. Moving to home position...')

    def joint_state_callback(self, msg: JointState):
        try:
            self.idx1 = msg.name.index('joint_1')
            self.idx2 = msg.name.index('joint_2')
            self.idx3 = msg.name.index('joint_3')
        except ValueError:
            if len(msg.position) < 3:
                return
            self.idx1, self.idx2, self.idx3 = 0, 1, 2

        if len(msg.position) <= max(self.idx1, self.idx2, self.idx3):
            return

        self.current_joint_pos = np.array(
            [msg.position[self.idx1], msg.position[self.idx2], msg.position[self.idx3]],
            dtype=np.float64,
        )

        if len(msg.velocity) > max(self.idx1, self.idx2, self.idx3):
            self.current_joint_vel = np.array(
                [msg.velocity[self.idx1], msg.velocity[self.idx2], msg.velocity[self.idx3]],
                dtype=np.float64,
            )

        self.joint_states_received = True

    def _publish_zero_torque(self):
        self.msg1.data = [0.0]
        self.msg2.data = [0.0]
        self.msg3.data = [0.0]
        self.pub1.publish(self.msg1)
        self.pub2.publish(self.msg2)
        self.pub3.publish(self.msg3)

    def _publish_torque(self, torque: np.ndarray):
        self.msg1.data = [float(torque[0])]
        self.msg2.data = [float(torque[1])]
        self.msg3.data = [float(torque[2])]
        self.pub1.publish(self.msg1)
        self.pub2.publish(self.msg2)
        self.pub3.publish(self.msg3)

    def control_loop(self):
        if not self.joint_states_received:
            return

        if self.start_time is None:
            self.start_time = time.time()

        elapsed_time = time.time() - self.start_time
        if elapsed_time > self.timeout:
            self.get_logger().warn(f'Timeout reached ({self.timeout}s). Stopping home reset.')
            self.shutdown()
            return

        error = self.home_position - self.current_joint_pos
        self.integral_error += error * self.dt
        self.integral_error = np.clip(self.integral_error, -0.5, 0.5)

        gravity_comp = np.array(
            [
                0.0,
                -44.0 * math.cos(self.current_joint_pos[1]),
                -12.0 * math.cos(self.current_joint_pos[1] + self.current_joint_pos[2]),
            ],
            dtype=np.float64,
        )

        torque = self.kp * error + self.ki * self.integral_error - self.kd * self.current_joint_vel + gravity_comp
        torque = np.clip(torque, -self.torque_limits, self.torque_limits)
        self._publish_torque(torque)

        max_error = float(np.max(np.abs(error)))
        if max_error < self.settle_tolerance_rad:
            self.converged_count += 1
            if self.converged_count >= self.convergence_threshold:
                self.get_logger().info(
                    f'Home position reached in {elapsed_time:.2f}s. '
                    f'Final position: {np.degrees(self.current_joint_pos).tolist()} deg'
                )
                self.shutdown()
                return
        else:
            self.converged_count = 0

        if int(elapsed_time / 0.5) != int((elapsed_time - self.dt) / 0.5):
            self.get_logger().info(
                f'Home reset t={elapsed_time:.2f}s | '
                f'err_deg={np.degrees(max_error):.4f} | '
                f'pos_deg={np.degrees(self.current_joint_pos).tolist()}'
            )

    def shutdown(self):
        self._publish_zero_torque()
        self._publish_zero_torque()
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        rclpy.shutdown()


def main():
    rclpy.init()
    node = HomeController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Home controller interrupted by user')
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
