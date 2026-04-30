#!/usr/bin/env python3
"""PID-only ROS 2 torque controller.

This controller waits for joint states, then drives the arm toward a single
target joint configuration using only PID control. No DNN, no CTC, no planner.

For trajectory tracking, use pid_controller_trajectory.py instead.
"""

import argparse
import math
import csv
import os
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


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
            torque_limits if torque_limits is not None else [2.0, 45.0, 10.0],
            dtype=np.float64,
        )

        self.pub1 = self.create_publisher(Float64MultiArray, '/joint_1_controller/commands', 10)
        self.pub2 = self.create_publisher(Float64MultiArray, '/joint_2_controller/commands', 10)
        self.pub3 = self.create_publisher(Float64MultiArray, '/joint_3_controller/commands', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.msg1 = Float64MultiArray()
        self.msg2 = Float64MultiArray()
        self.msg3 = Float64MultiArray()

        self.current_joint_pos = np.zeros(3, dtype=np.float64)
        self.current_joint_vel = np.zeros(3, dtype=np.float64)
        self.current_joint_eff = np.zeros(3, dtype=np.float64)
        self.joint_states_received = False
        self.effort_received = False

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
        self.log_dv1 = []
        self.log_dv2 = []
        self.log_dv3 = []
        self.log_tau1 = []
        self.log_tau2 = []
        self.log_tau3 = []
        self.log_tau_sensed1 = []
        self.log_tau_sensed2 = []
        self.log_tau_sensed3 = []

        self.get_logger().info('=' * 70)
        self.get_logger().info('PID TORQUE CONTROLLER (Point-to-Point)')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Target (deg): {np.degrees(self.target).tolist()}')
        self.get_logger().info(f'Kp={self.kp.tolist()}  Ki={self.ki.tolist()}  Kd={self.kd.tolist()}')
        self.get_logger().info(f'Torque limits: {self.torque_limits.tolist()} Nm')
        self.get_logger().info(f'Control rate: {1.0 / self.dt:.1f} Hz')

    @staticmethod
    def _mat_mul(a, b):
        return [
            [a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
             a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
             a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2]],
            [a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
             a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
             a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2]],
            [a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
             a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
             a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2]],
        ]

    @staticmethod
    def _mat_vec_mul(a, v):
        return [
            a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
            a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
            a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2],
        ]

    @staticmethod
    def _rot_x(angle):
        c = math.cos(angle)
        s = math.sin(angle)
        return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]

    @staticmethod
    def _rot_z(angle):
        c = math.cos(angle)
        s = math.sin(angle)
        return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]

    @staticmethod
    def _compose(r, p, r_local, p_local):
        rp = PIDTorqueController._mat_vec_mul(r, p_local)
        p_new = [p[0] + rp[0], p[1] + rp[1], p[2] + rp[2]]
        r_new = PIDTorqueController._mat_mul(r, r_local)
        return r_new, p_new

    def _fk_tip_world(self, q1, q2, q3):
        r = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        p = [0.0, 0.0, 0.0]
        r, p = self._compose(r, p, self._rot_z(0.0), [0.5, 0.5, 0.0])
        r, p = self._compose(r, p, self._rot_z(0.0), [0.0, 0.0, 0.1])

        r, p = self._compose(r, p, self._rot_z(0.0), [0.0, 0.0, 0.6718])
        r, p = self._compose(r, p, self._rot_z(q1), [0.0, 0.0, 0.0])

        r, p = self._compose(r, p, self._rot_x(-math.pi / 2.0), [0.0, 0.2435, 0.0])
        r, p = self._compose(r, p, self._rot_z(q2), [0.0, 0.0, 0.0])

        r, p = self._compose(r, p, self._rot_z(0.0), [0.4318, 0.0, -0.094])
        r, p = self._compose(r, p, self._rot_z(q3), [0.0, 0.0, 0.0])

        tip_local = [0.0, -0.233, 0.0]
        tip_world = self._mat_vec_mul(r, tip_local)
        return [p[0] + tip_world[0], p[1] + tip_world[1], p[2] + tip_world[2]]

    def publish_expected_path_marker(self):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'expected_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.012
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

        # For point-to-point: interpolate from current to target
        n = 50
        start_q = self.current_joint_pos.copy()
        end_q = self.target.copy()
        for i in range(n):
            alpha = i / (n - 1)
            q = start_q * (1 - alpha) + end_q * alpha
            xyz = self._fk_tip_world(q[0], q[1], q[2])
            pt = Point()
            pt.x = xyz[0]
            pt.y = xyz[1]
            pt.z = xyz[2]
            marker.points.append(pt)

        self.marker_pub.publish(marker)
        self.get_logger().info(f'Published expected end-effector path ({len(marker.points)} points)')

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

            if len(msg.effort) > max(idx1, idx2, idx3):
                self.current_joint_eff = np.array(
                    [msg.effort[idx1], msg.effort[idx2], msg.effort[idx3]],
                    dtype=np.float64,
                )
                self.effort_received = True

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
        self.log_dv1.append('0.00000000')
        self.log_dv2.append('0.00000000')
        self.log_dv3.append('0.00000000')
        self.log_tau1.append(f'{tau[0]:.8f}')
        self.log_tau2.append(f'{tau[1]:.8f}')
        self.log_tau3.append(f'{tau[2]:.8f}')
        self.log_tau_sensed1.append(f'{self.current_joint_eff[0]:.8f}')
        self.log_tau_sensed2.append(f'{self.current_joint_eff[1]:.8f}')
        self.log_tau_sensed3.append(f'{self.current_joint_eff[2]:.8f}')

        if self.current_idx % 50 == 0:
            self.get_logger().info(
                f'idx={self.current_idx} | err_deg={math.degrees(max_err):.3f} | '
                f'tau=[{tau[0]:.2f}, {tau[1]:.2f}, {tau[2]:.2f}]'
            )

    def save_log(self) -> Optional[str]:
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
                writer.writerow(['dv1'] + self.log_dv1)
                writer.writerow(['dv2'] + self.log_dv2)
                writer.writerow(['dv3'] + self.log_dv3)
                writer.writerow(['qd1'] + self.log_qd1)
                writer.writerow(['qd2'] + self.log_qd2)
                writer.writerow(['qd3'] + self.log_qd3)
                writer.writerow(['tau1'] + self.log_tau1)
                writer.writerow(['tau2'] + self.log_tau2)
                writer.writerow(['tau3'] + self.log_tau3)
                writer.writerow(['tau_sensed1'] + self.log_tau_sensed1)
                writer.writerow(['tau_sensed2'] + self.log_tau_sensed2)
                writer.writerow(['tau_sensed3'] + self.log_tau_sensed3)
            self.get_logger().info(f'✓ Log saved: {path}')
            return path
        except Exception as exc:
            self.get_logger().error(f'Failed to save log: {exc}')
            return None

    def save_tracking_plots(self, log_path: Optional[str] = None):
        if not self.log_t:
            return
        try:
            import matplotlib.pyplot as plt

            t = np.array([float(v) for v in self.log_t], dtype=np.float64)
            dp1 = np.array([float(v) for v in self.log_dp1], dtype=np.float64)
            dp2 = np.array([float(v) for v in self.log_dp2], dtype=np.float64)
            dp3 = np.array([float(v) for v in self.log_dp3], dtype=np.float64)
            q1 = np.array([float(v) for v in self.log_q1], dtype=np.float64)
            q2 = np.array([float(v) for v in self.log_q2], dtype=np.float64)
            q3 = np.array([float(v) for v in self.log_q3], dtype=np.float64)
            dv1 = np.array([float(v) for v in self.log_dv1], dtype=np.float64)
            dv2 = np.array([float(v) for v in self.log_dv2], dtype=np.float64)
            dv3 = np.array([float(v) for v in self.log_dv3], dtype=np.float64)
            qd1 = np.array([float(v) for v in self.log_qd1], dtype=np.float64)
            qd2 = np.array([float(v) for v in self.log_qd2], dtype=np.float64)
            qd3 = np.array([float(v) for v in self.log_qd3], dtype=np.float64)
            tc1 = np.array([float(v) for v in self.log_tau1], dtype=np.float64)
            tc2 = np.array([float(v) for v in self.log_tau2], dtype=np.float64)
            tc3 = np.array([float(v) for v in self.log_tau3], dtype=np.float64)
            ts1 = np.array([float(v) for v in self.log_tau_sensed1], dtype=np.float64)
            ts2 = np.array([float(v) for v in self.log_tau_sensed2], dtype=np.float64)
            ts3 = np.array([float(v) for v in self.log_tau_sensed3], dtype=np.float64)

            fig, axes = plt.subplots(2, 2, figsize=(14, 9), sharex=True)
            axes = axes.ravel()

            axes[0].plot(t, dp1, label='dp1_ref', linewidth=1.6)
            axes[0].plot(t, dp2, label='dp2_ref', linewidth=1.6)
            axes[0].plot(t, dp3, label='dp3_ref', linewidth=1.6)
            axes[0].plot(t, q1, '--', label='q1', linewidth=1.2)
            axes[0].plot(t, q2, '--', label='q2', linewidth=1.2)
            axes[0].plot(t, q3, '--', label='q3', linewidth=1.2)
            axes[0].set_title('Position Tracking')
            axes[0].set_ylabel('rad')
            axes[0].grid(True, alpha=0.3)
            axes[0].legend(ncol=2)

            axes[1].plot(t, dv1, label='dv1_ref', linewidth=1.6)
            axes[1].plot(t, dv2, label='dv2_ref', linewidth=1.6)
            axes[1].plot(t, dv3, label='dv3_ref', linewidth=1.6)
            axes[1].plot(t, qd1, '--', label='qd1', linewidth=1.2)
            axes[1].plot(t, qd2, '--', label='qd2', linewidth=1.2)
            axes[1].plot(t, qd3, '--', label='qd3', linewidth=1.2)
            axes[1].set_title('Velocity Tracking')
            axes[1].set_ylabel('rad/s')
            axes[1].grid(True, alpha=0.3)
            axes[1].legend(ncol=2)

            axes[2].plot(t, tc1, label='tau_cmd1', linewidth=1.6)
            axes[2].plot(t, tc2, label='tau_cmd2', linewidth=1.6)
            axes[2].plot(t, tc3, label='tau_cmd3', linewidth=1.6)
            axes[2].set_title('Commanded Torques')
            axes[2].set_xlabel('t (s)')
            axes[2].set_ylabel('Nm')
            axes[2].grid(True, alpha=0.3)
            axes[2].legend()

            axes[3].plot(t, ts1, label='tau_sensed1', linewidth=1.6)
            axes[3].plot(t, ts2, label='tau_sensed2', linewidth=1.6)
            axes[3].plot(t, ts3, label='tau_sensed3', linewidth=1.6)
            axes[3].set_title('Sensed Torques')
            axes[3].set_xlabel('t (s)')
            axes[3].set_ylabel('Nm')
            axes[3].grid(True, alpha=0.3)
            axes[3].legend()

            plt.tight_layout()
            if log_path:
                png_path = os.path.splitext(log_path)[0] + '_tracking.png'
            else:
                png_path = os.path.join(self.LOGS_DIR, f'pid_log_{int(time.time())}_tracking.png')
            plt.savefig(png_path, dpi=300, bbox_inches='tight')
            plt.close(fig)
            self.get_logger().info(f'✓ Tracking plots saved: {png_path}')
        except Exception as exc:
            self.get_logger().error(f'Failed to save tracking plots: {exc}')

    def shutdown(self):
        try:
            zero = Float64MultiArray()
            zero.data = [0.0]
            self.pub1.publish(zero)
            self.pub2.publish(zero)
            self.pub3.publish(zero)
        except Exception:
            pass
        log_path = self.save_log()
        self.save_tracking_plots(log_path)


def main():
    parser = argparse.ArgumentParser(description='PID-only torque controller (point-to-point)')
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

        node.get_logger().info('Publishing expected path and starting PID control loop')
        try:
            node.publish_expected_path_marker()
        except Exception:
            node.get_logger().warning('Failed to publish expected path marker')
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
