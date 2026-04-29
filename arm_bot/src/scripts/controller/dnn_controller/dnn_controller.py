#!/usr/bin/env python3
"""ROS2 DNN torque controller for point-to-point motion.

This controller mirrors the robust run flow used by the main DNN publisher:
1) Wait for joint states
2) Plan a smooth min-jerk trajectory from current pose to target
3) Stabilize at trajectory start with strong PID
4) Execute trajectory using online DeLaN+GRU feedforward + PD+I feedback

Usage:
  python3 /home/priyankan/Desktop/FYP-Puma_560/arm_bot/src/scripts/controller/dnn_controller/dnn_controller.py \
  --target 0 45 90

Or with a CSV trajectory:
    python3 /home/priyankan/Desktop/FYP-Puma_560/arm_bot/src/scripts/controller/dnn_controller/dnn_controller.py \
    --csv-path /path/to/trajectory.csv
"""

import argparse
import csv
import math
import os
import re
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker

# Allow direct script execution.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from .path_planner import generate_min_jerk_trajectory
    from .dnn_predictor import DNNInferenceEngine
except ImportError:
    from path_planner import generate_min_jerk_trajectory
    from dnn_predictor import DNNInferenceEngine


DNN_TEST_DIR = '/home/priyankan/Desktop/FYP-Puma_560/DNN_test'


class DNNTorqueController(Node):
    LOGS_DIR = os.path.join(os.path.dirname(__file__), 'logs')

    def __init__(
        self,
        target_rad,
        delan,
        gru,
        scaler,
        dt=0.01,
        kp=None,
        kd=None,
        ki=None,
        use_feedback=True,
        torque_limits=None,
        vel_filter_alpha=0.25,
        skip_stabilization_threshold_deg=1.0,
    ):
        super().__init__('dnn_torque_controller')
        self.target = np.array(target_rad, dtype=np.float64)
        self.dt = float(dt)

        # DNN tracking gains (same defaults as working torque_publisher_dnn.py)
        self.use_feedback = bool(use_feedback)
        self.kp = np.array(kp if kp is not None else [5.0, 20.0, 10.0], dtype=np.float64)
        self.kd = np.array(kd if kd is not None else [1.0, 3.0, 2.0], dtype=np.float64)
        self.ki = np.array(ki if ki is not None else [0.05, 0.2, 0.1], dtype=np.float64)

        # Stabilization gains (same style as PID controller)
        self.kp_stab = np.array([50.0, 200.0, 150.0], dtype=np.float64)
        self.kd_stab = np.array([12.0, 35.0, 10.0], dtype=np.float64)
        self.ki_stab = np.array([5.0, 25.0, 20.0], dtype=np.float64)

        # Use realistic limits for DNN torque mode (old joint-1 limit=2 Nm blocked motion)
        self.torque_limits = np.array(
            torque_limits if torque_limits is not None else [100.0, 100.0, 60.0],
            dtype=np.float64,
        )

        self.vel_filter_alpha = float(np.clip(vel_filter_alpha, 0.0, 1.0))
        self.skip_stabilization_threshold_rad = math.radians(
            float(skip_stabilization_threshold_deg)
        )

        # Publishers
        self.pub1 = self.create_publisher(Float64MultiArray, '/joint_1_controller/commands', 10)
        self.pub2 = self.create_publisher(Float64MultiArray, '/joint_2_controller/commands', 10)
        self.pub3 = self.create_publisher(Float64MultiArray, '/joint_3_controller/commands', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        # Subscriber
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Controller state
        self.current_joint_pos = np.zeros(3, dtype=np.float64)
        self.current_joint_vel = np.zeros(3, dtype=np.float64)
        self.current_joint_eff = np.zeros(3, dtype=np.float64)
        self.joint_states_received = False

        self.integral_error = np.zeros(3, dtype=np.float64)
        self.traj_integral_error = np.zeros(3, dtype=np.float64)
        self.stabilization_complete = False
        self.stabilization_iterations = 0

        self.timer = None
        self.stabilization_timer = None

        # Trajectory buffers
        self.t = None
        self.q = None
        self.qd = None
        self.qdd = None
        self.xyz = None
        self.n_points = 0
        self.current_idx = 0

        # DNN inference
        self.get_logger().info('Loading DNN models (DeLaN + GRU)...')
        self.dnn = DNNInferenceEngine(delan, gru, scaler)
        self.get_logger().info('DNN models loaded and ready.')

        # Log data in analyzer-compatible column format
        self.log_rows = []

        self.msg1 = Float64MultiArray()
        self.msg2 = Float64MultiArray()
        self.msg3 = Float64MultiArray()

        os.makedirs(self.LOGS_DIR, exist_ok=True)
        self.log_path = self._build_incremental_log_path()
        self.trajectory_mode = 'target'

        self.get_logger().info('=' * 80)
        self.get_logger().info('DNN TORQUE CONTROLLER (online DeLaN + GRU per timestep)')
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'Target (deg): {np.degrees(self.target).tolist()}')
        self.get_logger().info(
            f'Feedback={self.use_feedback}  Kp={self.kp.tolist()}  Kd={self.kd.tolist()}  Ki={self.ki.tolist()}'
        )
        self.get_logger().info(f'Torque limits: {self.torque_limits.tolist()} Nm')

    @staticmethod
    def _build_incremental_log_path() -> str:
        os.makedirs(DNNTorqueController.LOGS_DIR, exist_ok=True)
        prefix = 'dnn_controller_log_'
        pat = re.compile(r'^dnn_controller_log_(\d+)\.csv$')
        max_i = 0
        for fn in os.listdir(DNNTorqueController.LOGS_DIR):
            m = pat.match(fn)
            if m:
                max_i = max(max_i, int(m.group(1)))
        return os.path.join(DNNTorqueController.LOGS_DIR, f'{prefix}{max_i + 1}.csv')

    def joint_state_callback(self, msg: JointState):
        try:
            i1 = msg.name.index('joint_1')
            i2 = msg.name.index('joint_2')
            i3 = msg.name.index('joint_3')

            self.current_joint_pos = np.array(
                [msg.position[i1], msg.position[i2], msg.position[i3]], dtype=np.float64
            )

            if len(msg.velocity) > max(i1, i2, i3):
                raw_vel = np.array(
                    [msg.velocity[i1], msg.velocity[i2], msg.velocity[i3]],
                    dtype=np.float64,
                )
                a = self.vel_filter_alpha
                self.current_joint_vel = (1.0 - a) * self.current_joint_vel + a * raw_vel

            if len(msg.effort) > max(i1, i2, i3):
                self.current_joint_eff = np.array(
                    [msg.effort[i1], msg.effort[i2], msg.effort[i3]], dtype=np.float64
                )

            self.joint_states_received = True
        except (ValueError, IndexError):
            pass

    def publish_expected_path_marker(self):
        if self.xyz is None:
            return
        mk = Marker()
        mk.header.frame_id = 'world'
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns = 'expected_path'
        mk.id = 0
        mk.type = Marker.LINE_STRIP
        mk.action = Marker.ADD
        mk.scale.x = 0.012
        mk.color.r = 0.0
        mk.color.g = 1.0
        mk.color.b = 1.0
        mk.color.a = 0.95
        for p in self.xyz:
            pt = Point()
            pt.x = float(p[0])
            pt.y = float(p[1])
            pt.z = float(p[2])
            mk.points.append(pt)
        self.marker_pub.publish(mk)

    def plan_trajectory(self):
        dq = np.abs(self.target - self.current_joint_pos)
        v_max = 2.0
        a_max = 7.0
        t_vel = np.max(1.875 * dq / v_max)
        t_acc = np.max(np.sqrt(5.77 * dq / a_max))
        t_min = max(t_vel, t_acc, 0.5)
        t_total = max(t_min * 1.5, 3.0)

        self.get_logger().info(
            f'Planning trajectory: dq={np.degrees(dq).tolist()} deg, T_total={t_total:.2f}s'
        )
        self.t, self.q, self.qd, self.qdd, self.xyz = generate_min_jerk_trajectory(
            self.current_joint_pos, self.target, t_total, self.dt
        )
        self.n_points = len(self.t)
        self.current_idx = 0

    def load_trajectory_from_csv(self, csv_path: str):
        data = {}
        with open(csv_path, 'r') as f:
            for row in csv.reader(f):
                if row:
                    data[row[0].strip()] = [float(v) for v in row[1:]]

        required = ['t', 'dp1', 'dp2', 'dp3', 'dv1', 'dv2', 'dv3']
        missing = [k for k in required if k not in data]
        if missing:
            raise ValueError(f'Missing required CSV fields: {missing}')

        t = np.array(data['t'], dtype=np.float64)
        q = np.column_stack([
            np.array(data['dp1'], dtype=np.float64),
            np.array(data['dp2'], dtype=np.float64),
            np.array(data['dp3'], dtype=np.float64),
        ])
        qd = np.column_stack([
            np.array(data['dv1'], dtype=np.float64),
            np.array(data['dv2'], dtype=np.float64),
            np.array(data['dv3'], dtype=np.float64),
        ])

        if 'da1' in data and 'da2' in data and 'da3' in data:
            qdd = np.column_stack([
                np.array(data['da1'], dtype=np.float64),
                np.array(data['da2'], dtype=np.float64),
                np.array(data['da3'], dtype=np.float64),
            ])
        else:
            qdd = np.column_stack([
                np.gradient(qd[:, 0], t),
                np.gradient(qd[:, 1], t),
                np.gradient(qd[:, 2], t),
            ])

        if len(t) < 2:
            raise ValueError('CSV trajectory must contain at least 2 timesteps')

        # Keep controller timer in sync with CSV spacing.
        self.dt = float(t[1] - t[0])
        self.t = t
        self.q = q
        self.qd = qd
        self.qdd = qdd
        self.n_points = len(t)
        self.current_idx = 0

        try:
            from .path_planner import forward_kinematics_puma
        except ImportError:
            from path_planner import forward_kinematics_puma
        self.xyz = np.array([forward_kinematics_puma(qq) for qq in self.q], dtype=np.float64)

        self.get_logger().info(
            f'Loaded CSV trajectory: {csv_path} | points={self.n_points} | dt={self.dt:.4f}s'
        )

    def _publish_torque(self, tau):
        self.msg1.data = [float(tau[0])]
        self.msg2.data = [float(tau[1])]
        self.msg3.data = [float(tau[2])]
        self.pub1.publish(self.msg1)
        self.pub2.publish(self.msg2)
        self.pub3.publish(self.msg3)

    def stabilization_callback(self):
        target = self.q[0]
        error_pos = target - self.current_joint_pos
        max_err = float(np.max(np.abs(error_pos)))

        if max_err < 3.5e-6:
            self.get_logger().info(
                f'✓ Start position reached. Max error: {math.degrees(max_err):.6f} deg'
            )
            if self.stabilization_timer:
                self.stabilization_timer.cancel()
            self.stabilization_complete = True
            return

        self.stabilization_iterations += 1
        if self.stabilization_iterations >= 12000:
            self.get_logger().error(
                f'Stabilization timeout. Max error: {math.degrees(max_err):.4f} deg'
            )
            if self.stabilization_timer:
                self.stabilization_timer.cancel()
            self.stabilization_complete = True
            return

        self.integral_error += error_pos * self.dt
        self.integral_error = np.clip(
            self.integral_error,
            -np.array([0.5, 1.0, 1.0], dtype=np.float64),
            np.array([0.5, 1.0, 1.0], dtype=np.float64),
        )

        q2, q3 = self.current_joint_pos[1], self.current_joint_pos[2]
        gravity = np.array([0.0, -44.0 * math.cos(q2), -12.0 * math.cos(q2 + q3)], dtype=np.float64)
        tau = (
            self.kp_stab * error_pos
            + self.ki_stab * self.integral_error
            - self.kd_stab * self.current_joint_vel
            + gravity
        )
        tau = np.clip(tau, -np.array([100.0, 100.0, 50.0]), np.array([100.0, 100.0, 50.0]))
        self._publish_torque(tau)

        if self.stabilization_iterations % 50 == 0:
            self.get_logger().info(
                'stabilizing | '
                f't={self.stabilization_iterations * self.dt:.1f}s | '
                f'max_err={math.degrees(max_err):.3f} deg'
            )

    def _append_log_row(
        self,
        t_now,
        q_des,
        qd_des,
        qdd_des,
        tau_delan,
        tau_dnn,
        tau_fb,
        tau_total,
        e_pos,
        e_vel,
        gru_active,
    ):
        self.log_rows.append([
            float(t_now),
            float(q_des[0]), float(q_des[1]), float(q_des[2]),
            float(qd_des[0]), float(qd_des[1]), float(qd_des[2]),
            float(qdd_des[0]), float(qdd_des[1]), float(qdd_des[2]),
            float(self.current_joint_pos[0]), float(self.current_joint_pos[1]), float(self.current_joint_pos[2]),
            float(self.current_joint_vel[0]), float(self.current_joint_vel[1]), float(self.current_joint_vel[2]),
            float(tau_delan[0]), float(tau_delan[1]), float(tau_delan[2]),
            float(tau_dnn[0]), float(tau_dnn[1]), float(tau_dnn[2]),
            float(tau_fb[0]), float(tau_fb[1]), float(tau_fb[2]),
            float(tau_total[0]), float(tau_total[1]), float(tau_total[2]),
            float(self.current_joint_eff[0]), float(self.current_joint_eff[1]), float(self.current_joint_eff[2]),
            float(e_pos[0]), float(e_pos[1]), float(e_pos[2]),
            float(e_vel[0]), float(e_vel[1]), float(e_vel[2]),
            1 if gru_active else 0,
        ])

    def trajectory_callback(self):
        if self.current_idx >= self.n_points:
            if self.timer:
                self.timer.cancel()
            return

        q_des = self.q[self.current_idx]
        qd_des = self.qd[self.current_idx]
        qdd_des = self.qdd[self.current_idx]

        try:
            tau_dnn, tau_delan, gru_active = self.dnn.predict(q_des, qd_des, qdd_des)
        except Exception as exc:
            self.get_logger().error(f'DNN prediction failed at idx {self.current_idx}: {exc}')
            tau_dnn = np.zeros(3, dtype=np.float64)
            tau_delan = np.zeros(3, dtype=np.float64)
            gru_active = False

        e_pos = q_des - self.current_joint_pos
        e_vel = qd_des - self.current_joint_vel

        tau_fb = np.zeros(3, dtype=np.float64)
        if self.use_feedback:
            self.traj_integral_error += e_pos * self.dt
            self.traj_integral_error = np.clip(
                self.traj_integral_error,
                -np.array([0.3, 0.5, 0.5], dtype=np.float64),
                np.array([0.3, 0.5, 0.5], dtype=np.float64),
            )
            tau_fb = self.kp * e_pos + self.kd * e_vel + self.ki * self.traj_integral_error

        tau_total = np.clip(tau_dnn + tau_fb, -self.torque_limits, self.torque_limits)
        self._publish_torque(tau_total)

        self._append_log_row(
            self.t[self.current_idx],
            q_des,
            qd_des,
            qdd_des,
            tau_delan,
            tau_dnn,
            tau_fb,
            tau_total,
            e_pos,
            e_vel,
            gru_active,
        )

        if self.current_idx % 50 == 0:
            max_err_deg = math.degrees(np.max(np.abs(e_pos)))
            mode = 'GRU+DeLaN' if gru_active else f'DeLaN-only warmup {self.current_idx}/{self.dnn.SEQ_LEN}'
            self.get_logger().info(
                f'idx={self.current_idx}/{self.n_points} | err={max_err_deg:.2f} deg | '
                f'tau=[{tau_total[0]:.2f}, {tau_total[1]:.2f}, {tau_total[2]:.2f}] | {mode}'
            )

        self.current_idx += 1

    def save_log(self):
        if not self.log_rows:
            self.get_logger().warning('No log rows to save')
            return
        header = [
            't',
            'q_des_1', 'q_des_2', 'q_des_3',
            'qd_des_1', 'qd_des_2', 'qd_des_3',
            'qdd_des_1', 'qdd_des_2', 'qdd_des_3',
            'q_act_1', 'q_act_2', 'q_act_3',
            'qd_act_1', 'qd_act_2', 'qd_act_3',
            'tau_delan_1', 'tau_delan_2', 'tau_delan_3',
            'tau_dnn_1', 'tau_dnn_2', 'tau_dnn_3',
            'tau_fb_1', 'tau_fb_2', 'tau_fb_3',
            'tau_total_1', 'tau_total_2', 'tau_total_3',
            'tau_sensed_1', 'tau_sensed_2', 'tau_sensed_3',
            'e_pos_1', 'e_pos_2', 'e_pos_3',
            'e_vel_1', 'e_vel_2', 'e_vel_3',
            'gru_active',
        ]
        with open(self.log_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self.log_rows)
        self.get_logger().info(f'✓ Log saved: {self.log_path}')

    def run(self):
        self.get_logger().info('Waiting for joint states...')
        while not self.joint_states_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.joint_states_received:
            self.get_logger().error('No joint states received, exiting')
            return

        self.get_logger().info(
            f'Current position (deg): {np.degrees(self.current_joint_pos).tolist()}'
        )

        if self.t is None:
            self.plan_trajectory()
            self.trajectory_mode = 'target'
        self.publish_expected_path_marker()

        start_target = self.q[0]
        init_err = float(np.max(np.abs(start_target - self.current_joint_pos)))
        if init_err <= self.skip_stabilization_threshold_rad:
            self.get_logger().info('Phase 1 skipped: already near trajectory start')
        else:
            self.get_logger().info('=' * 80)
            if self.trajectory_mode == 'csv':
                self.get_logger().info('PHASE 1: STABILIZATION TO CSV START')
            else:
                self.get_logger().info('PHASE 1: STABILIZATION TO TRAJECTORY START')
            self.get_logger().info('=' * 80)
            self.integral_error[:] = 0.0
            self.stabilization_iterations = 0
            self.stabilization_complete = False
            self.stabilization_timer = self.create_timer(self.dt, self.stabilization_callback)
            while not self.stabilization_complete and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.001)
            if not rclpy.ok():
                return

        self.get_logger().info('=' * 80)
        if self.trajectory_mode == 'csv':
            self.get_logger().info('PHASE 2: DNN CSV TRAJECTORY EXECUTION')
        else:
            self.get_logger().info('PHASE 2: DNN TRAJECTORY EXECUTION')
        self.get_logger().info('=' * 80)
        self.current_idx = 0
        self.traj_integral_error[:] = 0.0
        self.timer = self.create_timer(self.dt, self.trajectory_callback)

        while rclpy.ok() and self.current_idx < self.n_points:
            rclpy.spin_once(self, timeout_sec=0.001)

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
    parser = argparse.ArgumentParser(description='DNN torque controller (target or CSV trajectory)')
    parser.add_argument('--target', type=float, nargs=3, default=None, help='Target joint angles (deg)')
    parser.add_argument('--csv-path', type=str, default=None, help='Trajectory CSV path (t,dp,dv,optional da)')
    parser.add_argument(
        '--delan-model',
        type=str,
        default=os.path.join(DNN_TEST_DIR, 'fyp_jax_delan_50.jax'),
        help='Path to DeLaN .jax model file',
    )
    parser.add_argument(
        '--gru-model',
        type=str,
        default=os.path.join(DNN_TEST_DIR, 'best_GRUResidual.pt'),
        help='Path to GRU .pt model file',
    )
    parser.add_argument(
        '--scaler',
        type=str,
        default=os.path.join(DNN_TEST_DIR, 'feature_scaler.pkl'),
        help='Path to feature scaler .pkl file',
    )
    parser.add_argument('--dt', type=float, default=0.01, help='Controller period in seconds')
    parser.add_argument('--kp', type=float, nargs=3, default=None, help='Feedback Kp [j1 j2 j3]')
    parser.add_argument('--kd', type=float, nargs=3, default=None, help='Feedback Kd [j1 j2 j3]')
    parser.add_argument('--ki', type=float, nargs=3, default=None, help='Feedback Ki [j1 j2 j3]')
    parser.add_argument('--no-feedback', action='store_true', help='Use feedforward only')
    parser.add_argument('--torque-limits', type=float, nargs=3, default=None, help='Torque limits [j1 j2 j3]')
    parser.add_argument('--vel-filter-alpha', type=float, default=0.25, help='Velocity filter alpha')
    parser.add_argument(
        '--skip-stabilization-threshold-deg',
        type=float,
        default=1.0,
        help='Skip stabilization if start error is below this threshold',
    )
    args = parser.parse_args()

    if (args.target is None) == (args.csv_path is None):
        parser.error('Provide exactly one of --target or --csv-path')

    rclpy.init()
    node = DNNTorqueController(
        target_rad=np.deg2rad(np.array(args.target if args.target is not None else [0.0, 0.0, 0.0], dtype=np.float64)),
        delan=args.delan_model,
        gru=args.gru_model,
        scaler=args.scaler,
        dt=args.dt,
        kp=args.kp,
        kd=args.kd,
        ki=args.ki,
        use_feedback=not args.no_feedback,
        torque_limits=args.torque_limits,
        vel_filter_alpha=args.vel_filter_alpha,
        skip_stabilization_threshold_deg=args.skip_stabilization_threshold_deg,
    )

    try:
        if args.csv_path is not None:
            node.load_trajectory_from_csv(args.csv_path)
            node.trajectory_mode = 'csv'
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
