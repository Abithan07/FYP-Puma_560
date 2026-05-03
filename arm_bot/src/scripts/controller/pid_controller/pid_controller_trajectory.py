#!/usr/bin/env python3
"""PID Trajectory-Tracking ROS 2 Torque Controller.

This controller tracks joint trajectories using PID control.
Supports two modes:
1. Generated trajectories: --target 0 45 30 --T 24
2. Loaded from CSV file: --trajectory-csv /path/to/trajectory.csv

The trajectory must be in row-wise format with minimum jerk profile.
"""

import argparse
import math
import csv
import os
import time
from typing import Optional, Tuple
import re

import numpy as np
import pandas as pd
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


# ===================== TRAJECTORY GENERATION HELPERS =====================

def min_jerk_profile(t: np.ndarray, T: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Compute minimum jerk trajectory profile.
    
    Parameters:
    -----------
    t : ndarray
        Time vector
    T : float
        Total trajectory duration
        
    Returns:
    --------
    f : ndarray
        Position profile (normalized 0 to 1)
    fd : ndarray
        Velocity profile
    fdd : ndarray
        Acceleration profile
    """
    tau = t / T
    
    f = 10*tau**3 - 15*tau**4 + 6*tau**5
    fd = (30*tau**2 - 60*tau**3 + 30*tau**4) / T
    fdd = (60*tau - 180*tau**2 + 120*tau**3) / (T**2)
    
    return f, fd, fdd


def generate_trajectory(target_deg: np.ndarray, T_total: float, 
                       dt: float = 0.01, 
                       q_start_deg: np.ndarray = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Generate minimum jerk trajectory from start to target.
    
    Parameters:
    -----------
    target_deg : ndarray
        Target joint angles in degrees [q1, q2, q3]
    T_total : float
        Trajectory duration in seconds
    dt : float
        Sampling time
    q_start_deg : ndarray
        Starting joint angles in degrees (default: [0, 45, 135])
        
    Returns:
    --------
    t : ndarray
        Time vector
    q : ndarray
        Joint positions (N, 3) in radians
    qd : ndarray
        Joint velocities (N, 3) in rad/s
    qdd : ndarray
        Joint accelerations (N, 3) in rad/s^2
    """
    if q_start_deg is None:
        q_start_deg = np.array([0.0, 45.0, 135.0])
    
    q_start = np.deg2rad(q_start_deg)
    q_end = np.deg2rad(target_deg)
    
    # Generate time vector
    t = np.arange(0, T_total + dt, dt)
    N = len(t)
    
    # Compute minimum jerk profile
    f, fd, fdd = min_jerk_profile(t, T_total)
    
    # Compute joint trajectories
    dq = q_end - q_start
    
    q = np.zeros((N, 3))
    qd = np.zeros((N, 3))
    qdd = np.zeros((N, 3))
    
    for j in range(3):
        dqj = dq[j]
        q[:, j] = q_start[j] + dqj * f
        qd[:, j] = dqj * fd
        qdd[:, j] = dqj * fdd
    
    return t, q, qd, qdd


def load_trajectory_from_csv(csv_path: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Load trajectory from row-wise CSV file.
    
    Expected format:
        row 0: t, t0, t1, ...
        row 1: dp1, dp1_0, dp1_1, ...
        row 2: dp2, dp2_0, dp2_1, ...
        row 3: dp3, dp3_0, dp3_1, ...
        row 4: dv1, dv1_0, dv1_1, ...
        row 5: dv2, dv2_0, dv2_1, ...
        row 6: dv3, dv3_0, dv3_1, ...
        row 7: da1, da1_0, da1_1, ... (optional)
        row 8: da2, da2_0, da2_1, ... (optional)
        row 9: da3, da3_0, da3_1, ... (optional)
    
    Parameters:
    -----------
    csv_path : str
        Path to trajectory CSV file
        
    Returns:
    --------
    t : ndarray
        Time vector
    q : ndarray
        Joint positions (N, 3) in radians
    qd : ndarray
        Joint velocities (N, 3) in rad/s
    qdd : ndarray
        Joint accelerations (N, 3) in rad/s^2 (zeros if not in CSV)
    """
    df = pd.read_csv(csv_path, header=None, dtype=str)
    
    series = {}
    for i in range(df.shape[0]):
        raw_name = df.iat[i, 0]
        if pd.isna(raw_name):
            continue
        
        # Normalize name: "dp1", "dp 1", "DP1" -> "dp1"
        key = "".join(ch for ch in str(raw_name).lower() if ch.isalnum())
        row_vals = pd.to_numeric(df.iloc[i, 1:], errors="coerce").to_numpy(dtype=float)
        series[key] = row_vals
    
    # Extract time and joint data
    t = series.get('t', None)
    if t is None:
        raise ValueError("Time vector 't' not found in CSV")
    
    # Extract positions (dp1, dp2, dp3)
    dp1 = series.get('dp1', None)
    dp2 = series.get('dp2', None)
    dp3 = series.get('dp3', None)
    
    if any(v is None for v in [dp1, dp2, dp3]):
        raise ValueError("Missing position data (dp1, dp2, dp3) in CSV")
    
    # Extract velocities (dv1, dv2, dv3)
    dv1 = series.get('dv1', None)
    dv2 = series.get('dv2', None)
    dv3 = series.get('dv3', None)
    
    if any(v is None for v in [dv1, dv2, dv3]):
        raise ValueError("Missing velocity data (dv1, dv2, dv3) in CSV")
    
    # Extract accelerations (da1, da2, da3) - optional
    da1 = series.get('da1', None)
    da2 = series.get('da2', None)
    da3 = series.get('da3', None)
    
    # Ensure all arrays have same length
    n = len(t)
    dp1, dp2, dp3 = dp1[:n], dp2[:n], dp3[:n]
    dv1, dv2, dv3 = dv1[:n], dv2[:n], dv3[:n]
    
    q = np.column_stack([dp1, dp2, dp3])
    qd = np.column_stack([dv1, dv2, dv3])
    
    if any(v is None for v in [da1, da2, da3]):
        # Use finite differences to compute accelerations
        qdd = np.diff(qd, axis=0) / np.diff(t)[:, np.newaxis]
        qdd = np.vstack([qdd[0], qdd])  # Replicate first row
    else:
        da1, da2, da3 = da1[:n], da2[:n], da3[:n]
        qdd = np.column_stack([da1, da2, da3])
    
    return t, q, qd, qdd


class PIDTrajectoryController(Node):
    LOGS_DIR = os.path.join(os.path.dirname(__file__), 'logs')

    def __init__(
        self,
        kp=None,
        ki=None,
        kd=None,
        torque_limits=None,
        torque_bias=None,
        dt=0.01,
        vel_filter_alpha=0.25,
        settle_tolerance_deg=1.0,
        use_feedforward=True,
    ):
        super().__init__('pid_trajectory_controller')

        self.dt = float(dt)
        self.vel_filter_alpha = float(np.clip(vel_filter_alpha, 0.0, 1.0))
        self.settle_tolerance_rad = math.radians(float(settle_tolerance_deg))
        self.use_feedforward = bool(use_feedforward)

        self.kp = np.array(kp if kp is not None else [50.0, 200.0, 150.0], dtype=np.float64)
        self.ki = np.array(ki if ki is not None else [5.0, 25.0, 20.0], dtype=np.float64)
        self.kd = np.array(kd if kd is not None else [12.0, 35.0, 10.0], dtype=np.float64)
        self.torque_limits = np.array(
            torque_limits if torque_limits is not None else [2.0, 45.0, 10.0],
            dtype=np.float64,
        )
        # Default bias (applied to commanded torques). Matches requested start torques.
        if torque_bias is None:
            torque_bias = [0.0, -21.8606490720699, -3.5527136788005e-14]
        self.torque_bias = np.array(torque_bias, dtype=np.float64)

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
        self.traj_idx = 0
        self.trajectory_complete = False
        self.timer = None

        # Trajectory data
        self.traj_t = None  # Time vector
        self.traj_q = None  # Position trajectory (N, 3)
        self.traj_qd = None  # Velocity trajectory (N, 3)
        self.traj_qdd = None  # Acceleration trajectory (N, 3)
        self.traj_n = 0  # Number of points

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
        self.current_path_points = []
        self.last_log_path = None
        self.pending_log_path = None

        self.get_logger().info('=' * 70)
        self.get_logger().info('PID TRAJECTORY CONTROLLER')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Kp={self.kp.tolist()}  Ki={self.ki.tolist()}  Kd={self.kd.tolist()}')
        self.get_logger().info(f'Torque limits: {self.torque_limits.tolist()} Nm')
        self.get_logger().info(f'Control rate: {1.0 / self.dt:.1f} Hz')
        self.get_logger().info(f'Feedforward enabled: {self.use_feedforward}')

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
        rp = PIDTrajectoryController._mat_vec_mul(r, p_local)
        p_new = [p[0] + rp[0], p[1] + rp[1], p[2] + rp[2]]
        r_new = PIDTrajectoryController._mat_mul(r, r_local)
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
        if self.traj_q is None:
            self.get_logger().warning('No trajectory set; cannot publish expected path')
            return

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

        for q in self.traj_q:
            xyz = self._fk_tip_world(q[0], q[1], q[2])
            pt = Point()
            pt.x = xyz[0]
            pt.y = xyz[1]
            pt.z = xyz[2]
            marker.points.append(pt)

        self.marker_pub.publish(marker)
        self.get_logger().info(f'Published expected end-effector path ({len(marker.points)} points)')

    def _path_color(self, r, g, b, a=0.95):
        color = type('Color', (), {})()
        color.r = float(r)
        color.g = float(g)
        color.b = float(b)
        color.a = float(a)
        return color

    def _publish_line_strip_marker(self, points, namespace, marker_id, color, scale=0.012):
        if not points:
            return
        mk = Marker()
        mk.header.frame_id = 'world'
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns = namespace
        mk.id = marker_id
        mk.type = Marker.LINE_STRIP
        mk.action = Marker.ADD
        mk.scale.x = scale
        mk.color.r = color.r
        mk.color.g = color.g
        mk.color.b = color.b
        mk.color.a = color.a
        mk.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        for x, y, z in points:
            pt = Point()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = float(z)
            mk.points.append(pt)
        self.marker_pub.publish(mk)

    def publish_current_path_marker(self):
        if not self.current_path_points:
            return
        self._publish_line_strip_marker(
            self.current_path_points,
            'current_path',
            100,
            self._path_color(1.0, 0.2, 0.2, 0.95),
            scale=0.014,
        )

    def _build_incremental_log_path(self, csv_path: str, mode_key: str) -> str:
        base_dir = PIDTrajectoryController.LOGS_DIR
        os.makedirs(base_dir, exist_ok=True)
        name = os.path.splitext(os.path.basename(os.path.expanduser(csv_path)))[0]
        prefix = f'{name}_{mode_key}_log_'
        pat = re.compile(rf'^{re.escape(prefix)}(\d+)\.csv$')
        max_r = 0
        try:
            for fn in os.listdir(base_dir):
                m = pat.match(fn)
                if m:
                    max_r = max(max_r, int(m.group(1)))
        except OSError:
            pass
        return os.path.join(base_dir, f'{prefix}{max_r + 1}.csv')

    def _resolve_log_path(self) -> str:
        if self.pending_log_path:
            return self.pending_log_path
        src = getattr(self, 'csv_path', None)
        if src is None:
            try:
                start_deg = np.degrees(self.traj_q[0])
                name = 'generated_' + '_'.join([f"{int(x)}" for x in start_deg])
            except Exception:
                name = 'generated'
            src = name
        self.pending_log_path = self._build_incremental_log_path(src, 'pid')
        return self.pending_log_path

    def set_trajectory(self, t: np.ndarray, q: np.ndarray, qd: np.ndarray, qdd: np.ndarray):
        """
        Set the trajectory to track.
        
        Parameters:
        -----------
        t : ndarray
            Time vector
        q : ndarray
            Position trajectory (N, 3) in radians
        qd : ndarray
            Velocity trajectory (N, 3) in rad/s
        qdd : ndarray
            Acceleration trajectory (N, 3) in rad/s^2
        """
        self.traj_t = np.array(t, dtype=np.float64)
        self.traj_q = np.array(q, dtype=np.float64)
        self.traj_qd = np.array(qd, dtype=np.float64)
        self.traj_qdd = np.array(qdd, dtype=np.float64)
        self.traj_n = len(t)
        
        self.get_logger().info(f'Trajectory set: {self.traj_n} points, duration: {t[-1]:.2f}s')
        self.get_logger().info(f'Target start: {np.degrees(q[0]).tolist()}')
        self.get_logger().info(f'Target end: {np.degrees(q[-1]).tolist()}')

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
        if self.traj_n == 0:
            self.get_logger().error('No trajectory set. Use set_trajectory() first.')
            return
        
        self.integral_error[:] = 0.0
        self.traj_idx = 0
        self.trajectory_complete = False
        self.timer = self.create_timer(self.dt, self.control_callback)

    def control_callback(self):
        # Check if trajectory is complete
        if self.traj_idx >= self.traj_n - 1:
            if not self.trajectory_complete:
                self.trajectory_complete = True
                self.get_logger().info(
                    f'Trajectory complete at t={self.traj_t[self.traj_idx]:.3f}s'
                )
            self.traj_idx = self.traj_n - 1  # Stay at last point
        
        # Get reference trajectory at current index
        idx = min(self.traj_idx, self.traj_n - 1)
        q_ref = self.traj_q[idx]
        qd_ref = self.traj_qd[idx]
        qdd_ref = self.traj_qdd[idx] if self.traj_qdd is not None else np.zeros(3)
        
        # Compute tracking errors
        error_pos = q_ref - self.current_joint_pos
        error_vel = qd_ref - self.current_joint_vel
        
        # PID control with optional feedforward
        self.integral_error += error_pos * self.dt
        self.integral_error = np.clip(self.integral_error, -0.75, 0.75)
        
        tau_feedback = self.kp * error_pos + self.kd * error_vel + self.ki * self.integral_error
        
        # Feedforward from reference acceleration (optional)
        if self.use_feedforward:
            tau_ff = 0.1 * qdd_ref  # Small feedforward gain
            tau = tau_feedback + tau_ff
        else:
            tau = tau_feedback

        # Apply configured torque bias and clip
        tau = tau + self.torque_bias
        tau = np.clip(tau, -self.torque_limits, self.torque_limits)

        self.msg1.data = [float(tau[0])]
        self.msg2.data = [float(tau[1])]
        self.msg3.data = [float(tau[2])]
        self.pub1.publish(self.msg1)
        self.pub2.publish(self.msg2)
        self.pub3.publish(self.msg3)

        max_err = float(np.max(np.abs(error_pos)))

        self.log_t.append(f'{self.traj_t[idx]:.3f}')
        self.log_dp1.append(f'{q_ref[0]:.8f}')
        self.log_dp2.append(f'{q_ref[1]:.8f}')
        self.log_dp3.append(f'{q_ref[2]:.8f}')
        self.log_q1.append(f'{self.current_joint_pos[0]:.8f}')
        self.log_q2.append(f'{self.current_joint_pos[1]:.8f}')
        self.log_q3.append(f'{self.current_joint_pos[2]:.8f}')
        self.log_qd1.append(f'{self.current_joint_vel[0]:.8f}')
        self.log_qd2.append(f'{self.current_joint_vel[1]:.8f}')
        self.log_qd3.append(f'{self.current_joint_vel[2]:.8f}')
        self.log_dv1.append(f'{qd_ref[0]:.8f}')
        self.log_dv2.append(f'{qd_ref[1]:.8f}')
        self.log_dv3.append(f'{qd_ref[2]:.8f}')
        self.log_tau1.append(f'{tau[0]:.8f}')
        self.log_tau2.append(f'{tau[1]:.8f}')
        self.log_tau3.append(f'{tau[2]:.8f}')
        self.log_tau_sensed1.append(f'{self.current_joint_eff[0]:.8f}')
        self.log_tau_sensed2.append(f'{self.current_joint_eff[1]:.8f}')
        self.log_tau_sensed3.append(f'{self.current_joint_eff[2]:.8f}')

        if self.traj_idx % 50 == 0:
            self.get_logger().info(
                f'idx={self.traj_idx} t={self.traj_t[idx]:.3f}s | err_deg={math.degrees(max_err):.3f} | '
                f'tau=[{tau[0]:.2f}, {tau[1]:.2f}, {tau[2]:.2f}]'
            )
        
        # Increment trajectory index for next control cycle
        # Append current end-effector tip for live path drawing and publish periodically
        try:
            tip = self._fk_tip_world(self.current_joint_pos[0], self.current_joint_pos[1], self.current_joint_pos[2])
            self.current_path_points.append(tip)
            if self.traj_idx % 10 == 0:
                self.publish_current_path_marker()
        except Exception:
            pass

        self.traj_idx += 1

    def save_log(self) -> Optional[str]:
        try:
            # Build (once) the log path so plots always share the same stem.
            path = self._resolve_log_path()

            # Convert in-memory logs (stored as strings) to numeric arrays
            n = len(self.log_t)
            t = np.array([float(x) for x in self.log_t]) if n else np.array([])
            q_des = np.column_stack([
                np.array([float(x) for x in self.log_dp1]),
                np.array([float(x) for x in self.log_dp2]),
                np.array([float(x) for x in self.log_dp3]),
            ]) if n else np.zeros((0,3))
            q_act = np.column_stack([
                np.array([float(x) for x in self.log_q1]),
                np.array([float(x) for x in self.log_q2]),
                np.array([float(x) for x in self.log_q3]),
            ]) if n else np.zeros((0,3))
            qd_des = np.column_stack([
                np.array([float(x) for x in self.log_dv1]),
                np.array([float(x) for x in self.log_dv2]),
                np.array([float(x) for x in self.log_dv3]),
            ]) if n else np.zeros((0,3))
            qdd_des = np.array(self.traj_qdd[:n], dtype=np.float64) if (n and self.traj_qdd is not None) else np.zeros((n, 3))
            qd_act = np.column_stack([
                np.array([float(x) for x in self.log_qd1]),
                np.array([float(x) for x in self.log_qd2]),
                np.array([float(x) for x in self.log_qd3]),
            ]) if n else np.zeros((0,3))
            tau_cmd = np.column_stack([
                np.array([float(x) for x in self.log_tau1]),
                np.array([float(x) for x in self.log_tau2]),
                np.array([float(x) for x in self.log_tau3]),
            ]) if n else np.zeros((0,3))
            tau_sensed = np.column_stack([
                np.array([float(x) for x in self.log_tau_sensed1]),
                np.array([float(x) for x in self.log_tau_sensed2]),
                np.array([float(x) for x in self.log_tau_sensed3]),
            ]) if n else np.zeros((0,3))

            # Compute errors (desired - actual)
            e_pos = q_des - q_act if n else np.zeros((0,3))
            e_vel = qd_des - qd_act if n else np.zeros((0,3))

            # Write time-step rows with header compatible with torque_publisher_dnn.py
            header = [
                't',
                'q_des_1','q_des_2','q_des_3',
                'qd_des_1','qd_des_2','qd_des_3',
                'qdd_des_1','qdd_des_2','qdd_des_3',
                'q_act_1','q_act_2','q_act_3',
                'qd_act_1','qd_act_2','qd_act_3',
                'tau_fb_1','tau_fb_2','tau_fb_3',
                'tau_total_1','tau_total_2','tau_total_3',
                'tau_sensed_1','tau_sensed_2','tau_sensed_3',
                'e_pos_1','e_pos_2','e_pos_3',
                'e_vel_1','e_vel_2','e_vel_3',
            ]

            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                for i in range(n):
                    row = [
                        f'{t[i]:.3f}',
                        f'{q_des[i,0]:.8f}', f'{q_des[i,1]:.8f}', f'{q_des[i,2]:.8f}',
                        f'{qd_des[i,0]:.8f}', f'{qd_des[i,1]:.8f}', f'{qd_des[i,2]:.8f}',
                        f'{qdd_des[i,0]:.8f}', f'{qdd_des[i,1]:.8f}', f'{qdd_des[i,2]:.8f}',
                        f'{q_act[i,0]:.8f}', f'{q_act[i,1]:.8f}', f'{q_act[i,2]:.8f}',
                        f'{qd_act[i,0]:.8f}', f'{qd_act[i,1]:.8f}', f'{qd_act[i,2]:.8f}',
                        f'{tau_cmd[i,0]:.8f}', f'{tau_cmd[i,1]:.8f}', f'{tau_cmd[i,2]:.8f}',
                        f'{tau_cmd[i,0]:.8f}', f'{tau_cmd[i,1]:.8f}', f'{tau_cmd[i,2]:.8f}',
                        f'{tau_sensed[i,0]:.8f}', f'{tau_sensed[i,1]:.8f}', f'{tau_sensed[i,2]:.8f}',
                        f'{e_pos[i,0]:.8f}', f'{e_pos[i,1]:.8f}', f'{e_pos[i,2]:.8f}',
                        f'{e_vel[i,0]:.8f}', f'{e_vel[i,1]:.8f}', f'{e_vel[i,2]:.8f}',
                    ]
                    writer.writerow(row)

            self.last_log_path = path
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
            if not log_path:
                log_path = self.last_log_path
            if not log_path:
                log_path = self.pending_log_path
            if not log_path:
                log_path = self._resolve_log_path()
            png_path = os.path.splitext(log_path)[0] + '_tracking.png'
            plt.savefig(png_path, dpi=300, bbox_inches='tight')
            plt.close(fig)
            self.get_logger().info(f'✓ Tracking plots saved: {png_path}')
            # Also save analyzer-style individual plots: torques, trajectory, velocity
            try:
                base = os.path.splitext(log_path)[0]

                # Torque breakdown (one subplot per joint)
                fig_t, axes_t = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
                for j, ax in enumerate(axes_t, start=1):
                    tau_cmd_j = [tc1, tc2, tc3][j-1]
                    tau_sen_j = [ts1, ts2, ts3][j-1]
                    ax.plot(t, tau_cmd_j, 'r-', linewidth=1.6, label='Commanded')
                    if np.any(~np.isclose(tau_sen_j, 0.0)):
                        ax.plot(t, tau_sen_j, 'purple', linestyle='--', linewidth=1.4, label='Sensed')
                    ax.set_ylabel(f'Joint {j} torque (Nm)')
                    ax.grid(True, alpha=0.3)
                    ax.legend(loc='upper right')
                axes_t[-1].set_xlabel('Time (s)')
                fig_t.suptitle('PID Controller — Torque Breakdown')
                plt.tight_layout()
                torques_path = f'{base}_torques.png'
                fig_t.savefig(torques_path, dpi=150, bbox_inches='tight')
                plt.close(fig_t)
                self.get_logger().info(f'✓ Torque breakdown saved: {torques_path}')

                # Trajectory (desired vs actual in degrees)
                fig_tr, axes_tr = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
                for j, ax in enumerate(axes_tr, start=1):
                    qd = [dp1, dp2, dp3][j-1]
                    qa = [q1, q2, q3][j-1]
                    ax.plot(t, np.degrees(qd), 'b-', linewidth=1.8, label='Desired')
                    ax.plot(t, np.degrees(qa), 'r--', linewidth=1.4, label='Actual')
                    ax.set_ylabel(f'Joint {j} (°)')
                    ax.grid(True, alpha=0.3)
                    ax.legend(loc='upper right')
                axes_tr[-1].set_xlabel('Time (s)')
                fig_tr.suptitle('PID Controller — Position Tracking')
                plt.tight_layout()
                traj_path = f'{base}_trajectory.png'
                fig_tr.savefig(traj_path, dpi=150, bbox_inches='tight')
                plt.close(fig_tr)
                self.get_logger().info(f'✓ Trajectory saved: {traj_path}')

                # Velocity (desired vs actual)
                fig_v, axes_v = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
                for j, ax in enumerate(axes_v, start=1):
                    vd = [dv1, dv2, dv3][j-1]
                    va = [qd1, qd2, qd3][j-1]
                    ax.plot(t, vd, 'b-', linewidth=1.8, label='Desired')
                    ax.plot(t, va, 'r--', linewidth=1.4, label='Actual')
                    ax.set_ylabel(f'Joint {j} (rad/s)')
                    ax.grid(True, alpha=0.3)
                    ax.legend(loc='upper right')
                axes_v[-1].set_xlabel('Time (s)')
                fig_v.suptitle('PID Controller — Velocity Tracking')
                plt.tight_layout()
                vel_path = f'{base}_velocity.png'
                fig_v.savefig(vel_path, dpi=150, bbox_inches='tight')
                plt.close(fig_v)
                self.get_logger().info(f'✓ Velocity saved: {vel_path}')
            except Exception as exc2:
                self.get_logger().warning(f'Failed to save individual analyzer plots: {exc2}')
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
    parser = argparse.ArgumentParser(description='PID trajectory-tracking torque controller')
    
    # Trajectory input: either generate or load
    traj_group = parser.add_mutually_exclusive_group(required=True)
    traj_group.add_argument(
        '--target',
        type=float,
        nargs=3,
        help='Target joint angles (deg) to generate trajectory: --target 0 45 30'
    )
    traj_group.add_argument(
        '--trajectory-csv',
        type=str,
        help='Load trajectory from CSV file'
    )
    
    # Optional parameters
    parser.add_argument('--T', type=float, default=None, help='Trajectory duration (seconds) - only with --target')
    parser.add_argument('--start', type=float, nargs=3, default=None, help='Start configuration (deg) for generated trajectory')
    parser.add_argument('--kp', type=float, nargs=3, default=None, help='PID Kp gains [j1 j2 j3]')
    parser.add_argument('--ki', type=float, nargs=3, default=None, help='PID Ki gains [j1 j2 j3]')
    parser.add_argument('--kd', type=float, nargs=3, default=None, help='PID Kd gains [j1 j2 j3]')
    parser.add_argument('--torque-limits', type=float, nargs=3, default=None, help='Torque limits [j1 j2 j3]')
    parser.add_argument('--dt', type=float, default=0.01, help='Controller period in seconds')
    parser.add_argument('--vel-filter-alpha', type=float, default=0.25, help='Velocity filter alpha')
    parser.add_argument('--settle-tolerance-deg', type=float, default=1.0, help='Tolerance for settled state')
    parser.add_argument('--no-feedforward', action='store_true', help='Disable feedforward term')
    
    args = parser.parse_args()

    rclpy.init()
    node = PIDTrajectoryController(
        kp=args.kp,
        ki=args.ki,
        kd=args.kd,
        torque_limits=args.torque_limits,
        dt=args.dt,
        vel_filter_alpha=args.vel_filter_alpha,
        settle_tolerance_deg=args.settle_tolerance_deg,
        use_feedforward=not args.no_feedforward,
    )

    try:
        # Load or generate trajectory
        if args.target is not None:
            # Generate trajectory
            target_deg = np.array(args.target, dtype=np.float64)
            T_total = args.T
            
            # Compute minimum duration if not specified
            if T_total is None:
                # Default constraint-based calculation
                v_max = 2.0
                a_max = 7.0
                q_start_deg = args.start if args.start is not None else np.array([0.0, 45.0, 135.0])
                dq = np.abs(np.deg2rad(target_deg) - np.deg2rad(q_start_deg))
                T_vel = np.max(1.875 * dq / v_max)
                T_acc = np.max(np.sqrt(5.77 * dq / a_max))
                T_total = max(T_vel, T_acc)
                node.get_logger().info(f'Auto-computed trajectory duration: {T_total:.2f}s')
            
            node.get_logger().info(f'Generating trajectory to {target_deg}° with T={T_total}s')
            t, q, qd, qdd = generate_trajectory(
                target_deg,
                T_total,
                dt=args.dt,
                q_start_deg=args.start
            )
            node.set_trajectory(t, q, qd, qdd)
            # Record a source name for log naming (generated trajectories)
            try:
                start_deg = np.degrees(q[0])
                node.csv_path = 'generated_' + '_'.join([f"{int(x)}" for x in start_deg])
            except Exception:
                node.csv_path = 'generated'
            
        else:
            # Load trajectory from CSV
            csv_path = args.trajectory_csv
            node.get_logger().info(f'Loading trajectory from: {csv_path}')
            t, q, qd, qdd = load_trajectory_from_csv(csv_path)
            node.set_trajectory(t, q, qd, qdd)
            node.csv_path = csv_path

        # Wait for joint states
        node.get_logger().info('Waiting for joint states...')
        while not node.joint_states_received and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.joint_states_received:
            node.get_logger().error('No joint states received, exiting')
            return

        node.get_logger().info('Publishing expected path and starting trajectory tracking')
        try:
            node.publish_expected_path_marker()
        except Exception:
            node.get_logger().warning('Failed to publish expected path marker')
        node.start()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
