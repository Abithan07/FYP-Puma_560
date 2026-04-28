#!/usr/bin/env python3
"""PID-only ROS 2 torque controller.

This controller waits for joint states, then drives the arm toward a single
target joint configuration using only PID control. No DNN, no CTC, no planner.
"""

import argparse
import math
import csv
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class PIDTorqueController(Node):
    LOGS_DIR = os.path.join(os.path.dirname(__file__), 'logs')

    def __init__(
        self,
        target_rad,
        kp=None,
        ki=None,
        kd=None,
        torque_limits=None,
        dt=0.01,
        vel_filter_alpha=0.25,
        settle_tolerance_deg=1.0,
    ):
        super().__init__('pid_torque_controller')

        self.target = np.array(target_rad, dtype=np.float64)
        self.dt = float(dt)
        self.vel_filter_alpha = float(np.clip(vel_filter_alpha, 0.0, 1.0))
        self.settle_tolerance_rad = math.radians(float(settle_tolerance_deg))


        self.kp = np.array(kp if kp is not None else [50.0, 200.0, 150.0], dtype=np.float64)
        self.ki = np.array(ki if ki is not None else [5.0, 25.0, 20.0], dtype=np.float64)
        self.kd = np.array(kd if kd is not None else [12.0, 35.0, 10.0], dtype=np.float64)
        self.torque_limits = np.array(
            torque_limits if torque_limits is not None else [100.0, 100.0, 50.0],
            dtype=np.float64,
        )

        self.pub1 = self.create_publisher(Float64MultiArray, '/joint_1_controller/commands', 10)
        self.pub2 = self.create_publisher(Float64MultiArray, '/joint_2_controller/commands', 10)
        self.pub3 = self.create_publisher(Float64MultiArray, '/joint_3_controller/commands', 10)

        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.msg1 = Float64MultiArray()
        self.msg2 = Float64MultiArray()
        self.msg3 = Float64MultiArray()

        self.current_joint_pos = np.zeros(3, dtype=np.float64)
        self.current_joint_vel = np.zeros(3, dtype=np.float64)
        self.joint_states_received = False

        self.integral_error = np.zeros(3, dtype=np.float64)
        self.current_idx = 0
        self.settled = False
        self.settled_counter = 0
        self.settle_required_cycles = max(1, int(0.5 / self.dt))
        self.timer = None

        os.makedirs(self.LOGS_DIR, exist_ok=True)
        self.log_t = []
        self.log_dp1 = []
        self.log_dp2 = []
        self.log_dp3 = []
        self.log_q1 = []
        self.log_q2 = []
        self.log_q3 = []
        self.log_qd1 = []
        self.log_qd2 = []
        self.log_qd3 = []
        self.log_tau1 = []
        self.log_tau2 = []
        self.log_tau3 = []

        self.get_logger().info('=' * 70)
        self.get_logger().info('PID TORQUE CONTROLLER')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Target (deg): {np.degrees(self.target).tolist()}')
        self.get_logger().info(f'Kp={self.kp.tolist()}  Ki={self.ki.tolist()}  Kd={self.kd.tolist()}')
        self.get_logger().info(f'Torque limits: {self.torque_limits.tolist()} Nm')
        self.get_logger().info(f'Control rate: {1.0 / self.dt:.1f} Hz')

    def joint_state_callback(self, msg: JointState):
        try:
            idx1 = msg.name.index('joint_1')
            idx2 = msg.name.index('joint_2')
            idx3 = msg.name.index('joint_3')

            self.current_joint_pos = np.array(
                [msg.position[idx1], msg.position[idx2], msg.position[idx3]],
                dtype=np.float64,
            )

            if len(msg.velocity) > max(idx1, idx2, idx3):
                raw_vel = np.array(
                    [msg.velocity[idx1], msg.velocity[idx2], msg.velocity[idx3]],
                    dtype=np.float64,
                )
                a = self.vel_filter_alpha
                self.current_joint_vel = (1.0 - a) * self.current_joint_vel + a * raw_vel

            self.joint_states_received = True
        except (ValueError, IndexError):
            pass

    def start(self):
        self.integral_error[:] = 0.0
        self.current_idx = 0
        self.settled = False
        self.settled_counter = 0
        self.timer = self.create_timer(self.dt, self.control_callback)

    def control_callback(self):
        error_pos = self.target - self.current_joint_pos
        error_vel = -self.current_joint_vel

        self.integral_error += error_pos * self.dt
        self.integral_error = np.clip(self.integral_error, -0.75, 0.75)

        tau = self.kp * error_pos + self.kd * error_vel + self.ki * self.integral_error
        tau = np.clip(tau, -self.torque_limits, self.torque_limits)

        self.msg1.data = [float(tau[0])]
        self.msg2.data = [float(tau[1])]
        self.msg3.data = [float(tau[2])]
        self.pub1.publish(self.msg1)
        self.pub2.publish(self.msg2)
        self.pub3.publish(self.msg3)

        max_err = float(np.max(np.abs(error_pos)))
        self.current_idx += 1

        if max_err <= self.settle_tolerance_rad:
            self.settled_counter += 1
            if not self.settled and self.settled_counter >= self.settle_required_cycles:
                self.settled = True
                self.get_logger().info(
                    f'Target reached and held: max error {math.degrees(max_err):.4f} deg'
                )
        else:
            self.settled_counter = 0

        self.log_t.append(f'{self.current_idx * self.dt:.3f}')
        self.log_dp1.append(f'{self.target[0]:.8f}')
        self.log_dp2.append(f'{self.target[1]:.8f}')
        self.log_dp3.append(f'{self.target[2]:.8f}')
        self.log_q1.append(f'{self.current_joint_pos[0]:.8f}')
        self.log_q2.append(f'{self.current_joint_pos[1]:.8f}')
        self.log_q3.append(f'{self.current_joint_pos[2]:.8f}')
        self.log_qd1.append(f'{self.current_joint_vel[0]:.8f}')
        self.log_qd2.append(f'{self.current_joint_vel[1]:.8f}')
        self.log_qd3.append(f'{self.current_joint_vel[2]:.8f}')
        self.log_tau1.append(f'{tau[0]:.8f}')
        self.log_tau2.append(f'{tau[1]:.8f}')
        self.log_tau3.append(f'{tau[2]:.8f}')

        if self.current_idx % 50 == 0:
            self.get_logger().info(
                f'idx={self.current_idx} | err_deg={math.degrees(max_err):.3f} | '
                f'tau=[{tau[0]:.2f}, {tau[1]:.2f}, {tau[2]:.2f}]'
            )

    def save_log(self):
        try:
            filename = f'pid_log_{int(time.time())}.csv'
            path = os.path.join(self.LOGS_DIR, filename)
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['t'] + self.log_t)
                writer.writerow(['dp1'] + self.log_dp1)
                writer.writerow(['dp2'] + self.log_dp2)
                writer.writerow(['dp3'] + self.log_dp3)
                writer.writerow(['q1'] + self.log_q1)
                writer.writerow(['q2'] + self.log_q2)
                writer.writerow(['q3'] + self.log_q3)
                writer.writerow(['qd1'] + self.log_qd1)
                writer.writerow(['qd2'] + self.log_qd2)
                writer.writerow(['qd3'] + self.log_qd3)
                writer.writerow(['tau1'] + self.log_tau1)
                writer.writerow(['tau2'] + self.log_tau2)
                writer.writerow(['tau3'] + self.log_tau3)
            self.get_logger().info(f'✓ Log saved: {path}')
        except Exception as exc:
            self.get_logger().error(f'Failed to save log: {exc}')

    def shutdown(self):
        try:
            zero = Float64MultiArray()
            zero.data = [0.0]
            self.pub1.publish(zero)
            self.pub2.publish(zero)
            self.pub3.publish(zero)
        except Exception:
            pass
        self.save_log()


def main():
    parser = argparse.ArgumentParser(description='PID-only torque controller')
    parser.add_argument('--target', type=float, nargs=3, required=True, help='Target joint angles (deg)')
    parser.add_argument('--kp', type=float, nargs=3, default=None, help='PID Kp gains [j1 j2 j3]')
    parser.add_argument('--ki', type=float, nargs=3, default=None, help='PID Ki gains [j1 j2 j3]')
    parser.add_argument('--kd', type=float, nargs=3, default=None, help='PID Kd gains [j1 j2 j3]')
    parser.add_argument('--torque-limits', type=float, nargs=3, default=None, help='Torque limits [j1 j2 j3]')
    parser.add_argument('--dt', type=float, default=0.01, help='Controller period in seconds')
    parser.add_argument('--vel-filter-alpha', type=float, default=0.25, help='Velocity filter alpha')
    parser.add_argument('--settle-tolerance-deg', type=float, default=1.0, help='Tolerance for settled state')
    args = parser.parse_args()

    rclpy.init()
    node = PIDTorqueController(
        target_rad=np.deg2rad(np.array(args.target, dtype=np.float64)),
        kp=args.kp,
        ki=args.ki,
        kd=args.kd,
        torque_limits=args.torque_limits,
        dt=args.dt,
        vel_filter_alpha=args.vel_filter_alpha,
        settle_tolerance_deg=args.settle_tolerance_deg,
    )

    try:
        node.get_logger().info('Waiting for joint states...')
        while not node.joint_states_received and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.joint_states_received:
            node.get_logger().error('No joint states received, exiting')
            return

        node.get_logger().info('Starting PID control loop')
        node.start()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
