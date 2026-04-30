#!/usr/bin/env python3
"""
Computed Torque Control Publisher - Full Dynamic Control + Feedback
Implements τ = D(q)q̈ + C(q,q̇) + G(q) + Kp(q_des-q_act) + Kd(q̇_des-q̇_act)
with comprehensive logging of all control terms and tracking errors.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
import csv
import math
import subprocess
import time
import os
import argparse
import sys
import re
import numpy as np

# Add path to import inverse dynamics model
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

try:
    from inverse_dynamics_model import (
        compute_inverse_dynamics,
        compute_gravity_analytical,
        compute_D_matrix,
        compute_coriolis,
        compute_christoffel_symbols
    )
    HAS_INV_DYN = True
except ImportError:
    HAS_INV_DYN = False
    def compute_inverse_dynamics(q, qd, qdd):
        """Fallback: return zeros if module not available"""
        return np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3)

class TorquePublisher(Node):
    def __init__(self, csv_path=None, skip_stabilization_threshold_deg=1.0, 
                 kp=None, kd=None, ki=None, use_feedback=True, use_model=True,
                 dynamics_frame='desired', torque_limits=None,
                 vel_filter_alpha=0.25, log_path=None):
        super().__init__('torque_publisher')
        
        # Control law configuration
        self.use_feedback = use_feedback
        self.use_model = use_model
        self.dynamics_frame = dynamics_frame
        self.vel_filter_alpha = float(np.clip(vel_filter_alpha, 0.0, 1.0))
        
        # Control gains (can be overridden by arguments)
        if kp is None:
            # self.kp = np.array([30.0, 100.0, 50.0])  # Proportional gains
            self.kp = np.array([30.0, 100.0, 200.0])  # Proportional gains
        else:
            self.kp = np.array(kp)
            
        if kd is None:
            # self.kd = np.array([4.0, 10.0, 5.0])  # Derivative gains
            self.kd = np.array([4.0, 10.0, 20.0])  # Derivative gains
        else:
            self.kd = np.array(kd)

        if ki is None:
            # Keep small integral action to remove slow bias without windup.
            self.ki = np.array([0.2, 1.0, 0.8])
        else:
            self.ki = np.array(ki)

        if torque_limits is None:
            self.torque_limits = np.array([100.0, 100.0, 60.0])
        else:
            self.torque_limits = np.array(torque_limits)
        
        # Publishers for torque commands (QoS=10 for reliability)
        self.pub1 = self.create_publisher(Float64MultiArray, '/joint_1_controller/commands', 10)
        self.pub2 = self.create_publisher(Float64MultiArray, '/joint_2_controller/commands', 10)
        self.pub3 = self.create_publisher(Float64MultiArray, '/joint_3_controller/commands', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.recording_state_pub = self.create_publisher(
            Bool,
            '/line_drawer/recording_active',
            state_qos,
        )
        self._publish_recording_state(False)
        
        # Subscriber to monitor joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Load CSV data (pre-load ALL data to avoid I/O delays during execution)
        if csv_path is None:
            self.csv_path = os.path.expanduser('/data/ros2/ros2_ws2/arm_bot/src/scripts/script_resources/path_021_joint_states_modified.csv')
        else:
            self.csv_path = os.path.expanduser(csv_path)
        self.load_trajectory_data()
        
        # Pre-allocate message objects (avoid allocation overhead during control loop)
        self.msg1 = Float64MultiArray()
        self.msg2 = Float64MultiArray()
        self.msg3 = Float64MultiArray()
        
        # State variables
        self.current_idx = 0
        self.current_joint_pos = np.array([0.0, 0.0, 0.0])
        self.current_joint_vel = np.array([0.0, 0.0, 0.0])
        self.current_joint_efforts = np.array([0.0, 0.0, 0.0])  # Sensed torques from Gazebo
        self.joint_states_received = False
        self.trajectory_active = False
        self.trajectory_timer = None
        
        # PID control timer for stabilization phase
        self.stabilization_timer = None
        self.stabilization_complete = False
        self.stabilization_iterations = 0
        
        # Integral error accumulation for PID
        self.integral_error = np.array([0.0, 0.0, 0.0])
        self.traj_integral_error = np.array([0.0, 0.0, 0.0])
        
        # Logger subprocess and service clients
        self.logger_process = None
        self.logger_start_client = None
        self.logger_stop_client = None
        
        # Logging infrastructure for comprehensive data capture
        self.log_path = log_path or self._build_incremental_log_path(self.csv_path)
        self.log_file = None
        self.log_writer = None
        self.log_data = []  # Buffer for batch writing
        self.log_buffer_size = 100  # Write every N timesteps

        # Skip stabilization if current pose is already very close to CSV start pose.
        self.skip_stabilization_threshold_rad = math.radians(skip_stabilization_threshold_deg)
        
        self.get_logger().info('='*80)
        self.get_logger().info('COMPUTED TORQUE CONTROL PUBLISHER - FULL DYNAMIC FEEDBACK')
        self.get_logger().info('='*80)
        self.get_logger().info(f'Loaded {len(self.time_data)} trajectory points')
        self.get_logger().info(f'Trajectory duration: {self.time_data[-1]:.2f}s')
        self.get_logger().info(f'Control frequency: 100 Hz (dt={self.dt:.4f}s)')
        self.get_logger().info(f'Initial target: [{self.dp1[0]:.4f}, {self.dp2[0]:.4f}, {self.dp3[0]:.4f}] rad')
        self.get_logger().info(f'Model-based control: {self.use_model}')
        self.get_logger().info(f'Feedback control: {self.use_feedback}')
        self.get_logger().info(f'Dynamics frame: {self.dynamics_frame} (for consistent model/feedback decomposition)')
        self.get_logger().info(
            f'Control gains: Kp={self.kp.tolist()}, Kd={self.kd.tolist()}, Ki={self.ki.tolist()}'
        )
        self.get_logger().info(f'Torque limits: {self.torque_limits.tolist()} Nm')
        self.get_logger().info(f'Velocity filter alpha: {self.vel_filter_alpha:.3f}')
        self.get_logger().info(f'Logging to: {self.log_path}')
        self.get_logger().info(
            f'Skip stabilization threshold: {skip_stabilization_threshold_deg:.3f} deg '
            f'({self.skip_stabilization_threshold_rad:.6f} rad)'
        )

    # Fixed directory for all trajectories and logs
    LOGS_DIR = '/home/priyankan/Desktop/FYP-Puma_560/arm_bot/src/scripts/logs'

    @staticmethod
    def _build_incremental_log_path(csv_path: str) -> str:
        """Generate <trajectory_name>_ctc_log_<run>.csv in the fixed logs directory."""
        csv_path_expanded = os.path.expanduser(csv_path)
        base_dir = TorquePublisher.LOGS_DIR
        os.makedirs(base_dir, exist_ok=True)

        trajectory_name = os.path.splitext(os.path.basename(csv_path_expanded))[0]
        prefix = f"{trajectory_name}_ctc_log_"
        pattern = re.compile(rf"^{re.escape(prefix)}(\d+)\.csv$")

        max_run_idx = 0
        try:
            for fname in os.listdir(base_dir):
                match = pattern.match(fname)
                if match:
                    max_run_idx = max(max_run_idx, int(match.group(1)))
        except OSError:
            # Fall back to run index 1 if directory listing fails.
            max_run_idx = 0

        next_run_idx = max_run_idx + 1
        return os.path.join(base_dir, f"{prefix}{next_run_idx}.csv")

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
        rp = TorquePublisher._mat_vec_mul(r, p_local)
        p_new = [p[0] + rp[0], p[1] + rp[1], p[2] + rp[2]]
        r_new = TorquePublisher._mat_mul(r, r_local)
        return r_new, p_new

    def _fk_tip_world(self, q1, q2, q3):
        # Base: world -> base_link, then base_link -> link_1 (fixed joint).
        r = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        p = [0.0, 0.0, 0.0]
        r, p = self._compose(r, p, self._rot_z(0.0), [0.5, 0.5, 0.0])
        r, p = self._compose(r, p, self._rot_z(0.0), [0.0, 0.0, 0.1])

        # joint_1: origin then rotation about z.
        r, p = self._compose(r, p, self._rot_z(0.0), [0.0, 0.0, 0.6718])
        r, p = self._compose(r, p, self._rot_z(q1), [0.0, 0.0, 0.0])

        # joint_2: origin with rpy(-pi/2, 0, 0), then rotation about z.
        r, p = self._compose(r, p, self._rot_x(-math.pi / 2.0), [0.0, 0.2435, 0.0])
        r, p = self._compose(r, p, self._rot_z(q2), [0.0, 0.0, 0.0])

        # joint_3: origin then rotation about z.
        r, p = self._compose(r, p, self._rot_z(0.0), [0.4318, 0.0, -0.094])
        r, p = self._compose(r, p, self._rot_z(q3), [0.0, 0.0, 0.0])

        # Tip offset in link_3 frame (same as line_drawer default).
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

        for q1, q2, q3 in zip(self.dp1, self.dp2, self.dp3):
            xyz = self._fk_tip_world(q1, q2, q3)
            pt = Point()
            pt.x = xyz[0]
            pt.y = xyz[1]
            pt.z = xyz[2]
            marker.points.append(pt)

        self.marker_pub.publish(marker)
        self.get_logger().info(f'Published expected end-effector path ({len(marker.points)} points)')
    
    @staticmethod
    def generate_and_save_trajectory(q_start, q_end, save_path,
                                     dt=0.01, v_max=2.0, a_max=7.0,
                                     possible_T=None):
        """Generate a minimum-jerk trajectory from q_start to q_end and save as a CTC-compatible CSV.

        Returns (save_path, T_total).
        """
        if possible_T is None:
            possible_T = np.arange(12, 25, 5)  # [12, 17, 22] seconds

        q_start = np.array(q_start, dtype=float)
        q_end   = np.array(q_end,   dtype=float)
        dq_abs  = np.abs(q_end - q_start)

        # Kinematic lower bound on duration
        T_vel = float(np.max(1.875 * dq_abs / v_max))
        T_acc = float(np.max(np.sqrt(5.77 * dq_abs / a_max)))
        T_min = max(T_vel, T_acc)

        T_rand  = float(np.random.choice(possible_T))
        T_total = max(T_min, T_rand)

        t   = np.arange(0, T_total + dt, dt)
        tau = t / T_total

        # Minimum-jerk profile
        f   = 10*tau**3 - 15*tau**4 + 6*tau**5
        fd  = (30*tau**2 - 60*tau**3 + 30*tau**4) / T_total
        fdd = (60*tau    - 180*tau**2 + 120*tau**3) / T_total**2

        dq_vec = q_end - q_start
        q   = q_start + np.outer(f,   dq_vec)
        qd  =           np.outer(fd,  dq_vec)
        qdd =           np.outer(fdd, dq_vec)

        traj = np.vstack([
            t,
            q[:, 0],  q[:, 1],  q[:, 2],
            qd[:, 0], qd[:, 1], qd[:, 2],
            qdd[:, 0],qdd[:, 1],qdd[:, 2],
        ])

        labels = np.array(
            ["t", "dp1", "dp2", "dp3", "dv1", "dv2", "dv3", "da1", "da2", "da3"]
        ).reshape(-1, 1)

        traj_str = []
        for i, row in enumerate(traj):
            fmt = '%.3f' if i == 0 else '%.8f'
            traj_str.append(np.char.mod(fmt, row))
        traj_str = np.array(traj_str)

        os.makedirs(os.path.dirname(os.path.abspath(save_path)), exist_ok=True)
        np.savetxt(
            save_path,
            np.hstack((labels, traj_str)),
            delimiter=",",
            fmt="%s",
        )
        return save_path, T_total

    def load_trajectory_data(self):
        """Load trajectory data from CSV file (executed once at startup)"""
        data = {}
        with open(self.csv_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if row:
                    key = row[0]
                    values = [float(val) for val in row[1:]]
                    data[key] = values
        
        # Extract trajectory arrays (stored in memory for fast access)
        self.time_data = np.array(data['t'])
        self.dp1 = np.array(data['dp1'])  # Desired joint positions
        self.dp2 = np.array(data['dp2'])
        self.dp3 = np.array(data['dp3'])
        self.dv1 = np.array(data['dv1'])  # Desired joint velocities (if available)
        self.dv2 = np.array(data['dv2'])
        self.dv3 = np.array(data['dv3'])

        # Use provided accelerations when available, otherwise estimate.
        if 'da1' in data and 'da2' in data and 'da3' in data:
            self.da1 = np.array(data['da1'])
            self.da2 = np.array(data['da2'])
            self.da3 = np.array(data['da3'])
        else:
            self.da1 = np.gradient(self.dv1, self.time_data)
            self.da2 = np.gradient(self.dv2, self.time_data)
            self.da3 = np.gradient(self.dv3, self.time_data)
        
        # Calculate time step (should be 0.01s for 100Hz)
        self.dt = self.time_data[1] - self.time_data[0] if len(self.time_data) > 1 else 0.01
        
        # Pre-compute reference trajectory arrays for faster access
        self.n_points = len(self.time_data)
    
    def joint_state_callback(self, msg):
        """Minimal callback - just update state variables"""
        try:
            idx1 = msg.name.index('joint_1')
            idx2 = msg.name.index('joint_2')
            idx3 = msg.name.index('joint_3')
            
            self.current_joint_pos = np.array([
                msg.position[idx1],
                msg.position[idx2],
                msg.position[idx3]
            ])
            
            if len(msg.velocity) >= 3:
                raw_vel = np.array([
                    msg.velocity[idx1],
                    msg.velocity[idx2],
                    msg.velocity[idx3]
                ])
                self.current_joint_vel = np.array([
                    (1.0 - self.vel_filter_alpha) * self.current_joint_vel[i]
                    + self.vel_filter_alpha * raw_vel[i]
                    for i in range(3)
                ])
            
            # Extract sensed torques (efforts) from Gazebo
            if len(msg.effort) >= 3:
                self.current_joint_efforts = np.array([
                    msg.effort[idx1],
                    msg.effort[idx2],
                    msg.effort[idx3]
                ])
            
            self.joint_states_received = True
        except (ValueError, IndexError):
            pass
    
    def launch_logger(self):
        """Launch the triggered logger as a subprocess"""
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            logger_script = os.path.join(script_dir, 'continuous_logger_triggered.py')
            
            self.get_logger().info('Launching triggered logger subprocess...')
            # Don't pipe stdout/stderr - let subprocess output directly to console
            # This prevents ROS2 initialization blocking issues in subprocess
            cmd = ['python3', logger_script]
            # Pass dataset path to logger so it can include it in the log filename
            if self.csv_path:
                cmd.extend(['--dataset-path', self.csv_path])
            self.logger_process = subprocess.Popen(
                cmd,
                stdout=None,  # Inherit parent's stdout
                stderr=None   # Inherit parent's stderr
            )
            
            # Give logger time to initialize and create services (100ms)
            time.sleep(0.1)
            
            # Create service clients
            self.logger_start_client = self.create_client(Trigger, '/logger/start')
            self.logger_stop_client = self.create_client(Trigger, '/logger/stop')
            
            # Wait for services to be available (max 5 seconds)
            timeout = 5.0
            start_time = time.time()
            while not self.logger_start_client.wait_for_service(timeout_sec=0.1):
                if time.time() - start_time > timeout:
                    self.get_logger().error('Logger start service not available!')
                    return False
                rclpy.spin_once(self, timeout_sec=0.01)
            
            self.get_logger().info('✓ Logger subprocess ready')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to launch logger: {e}')
            return False
    
    def start_logger(self):
        """Call the logger start service"""
        if not self.logger_start_client:
            self.get_logger().error('Logger client not initialized')
            return False
        
        request = Trigger.Request()
        future = self.logger_start_client.call_async(request)
        
        # Wait for response (with timeout)
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 1.0:
                self.get_logger().error('Logger start service timeout')
                return False
            rclpy.spin_once(self, timeout_sec=0.01)
        
        response = future.result()
        if response.success:
            self.get_logger().info('✓ Logger recording started')
            self._publish_recording_state(True)
        else:
            self.get_logger().error(f'Logger start failed: {response.message}')
        
        return response.success
    
    def stop_logger(self):
        """Call the logger stop service"""
        if not self.logger_stop_client:
            self.get_logger().error('Logger client not initialized')
            return False
        
        request = Trigger.Request()
        future = self.logger_stop_client.call_async(request)
        
        # Wait for response (with timeout)
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 1.0:
                self.get_logger().error('Logger stop service timeout')
                return False
            rclpy.spin_once(self, timeout_sec=0.01)
        
        response = future.result()
        if response.success:
            self.get_logger().info(f'✓ Logger stopped: {response.message}')
            self._publish_recording_state(False)
        else:
            self.get_logger().error(f'Logger stop failed: {response.message}')
        
        return response.success
    
    def shutdown_logger(self):
        """Terminate the logger subprocess"""
        self._publish_recording_state(False)
        if self.logger_process:
            self.logger_process.terminate()
            try:
                self.logger_process.wait(timeout=2.0)
                self.get_logger().info('✓ Logger subprocess terminated')
            except subprocess.TimeoutExpired:
                self.logger_process.kill()
                self.get_logger().warning('Logger subprocess killed (timeout)')

    def _publish_recording_state(self, active):
        msg = Bool()
        msg.data = bool(active)
        self.recording_state_pub.publish(msg)
    
    def open_log_file(self):
        """Initialize CSV log file with header"""
        try:
            self.log_file = open(self.log_path, 'w', newline='')
            self.log_writer = csv.writer(self.log_file)
            
            # CSV header with all required columns
            header = [
                't',  # time
                'q_des_1', 'q_des_2', 'q_des_3',  # desired positions
                'qd_des_1', 'qd_des_2', 'qd_des_3',  # desired velocities
                'qdd_des_1', 'qdd_des_2', 'qdd_des_3',  # desired accelerations
                'q_act_1', 'q_act_2', 'q_act_3',  # actual positions
                'qd_act_1', 'qd_act_2', 'qd_act_3',  # actual velocities
                'tau_model_1', 'tau_model_2', 'tau_model_3',  # model-based torques
                'tau_fb_1', 'tau_fb_2', 'tau_fb_3',  # feedback torques
                'tau_total_1', 'tau_total_2', 'tau_total_3',  # total (commanded) torques
                'tau_sensed_1', 'tau_sensed_2', 'tau_sensed_3',  # sensed torques from Gazebo
                'e_pos_1', 'e_pos_2', 'e_pos_3',  # position tracking errors
                'e_vel_1', 'e_vel_2', 'e_vel_3',  # velocity tracking errors
            ]
            self.log_writer.writerow(header)
            self.log_file.flush()
            self.get_logger().info(f'✓ Log file opened: {self.log_path}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to open log file: {e}')
            return False
    
    def log_timestep(self, t, q_des, qd_des, qdd_des, q_act, qd_act, 
                     tau_model, tau_fb, tau_total, tau_sensed, e_pos, e_vel):
        """Buffer a timestep of data for logging"""
        row = [
            f'{t:.3f}',
        ]
        # Desired states
        for val in q_des:
            row.append(f'{val:.8f}')
        for val in qd_des:
            row.append(f'{val:.8f}')
        for val in qdd_des:
            row.append(f'{val:.8f}')
        # Actual states
        for val in q_act:
            row.append(f'{val:.8f}')
        for val in qd_act:
            row.append(f'{val:.8f}')
        # Torques
        for val in tau_model:
            row.append(f'{val:.8f}')
        for val in tau_fb:
            row.append(f'{val:.8f}')
        for val in tau_total:
            row.append(f'{val:.8f}')
        # Sensed torques from Gazebo
        for val in tau_sensed:
            row.append(f'{val:.8f}')
        # Errors
        for val in e_pos:
            row.append(f'{val:.8f}')
        for val in e_vel:
            row.append(f'{val:.8f}')
        
        self.log_data.append(row)
        
        # Flush buffer periodically
        if len(self.log_data) >= self.log_buffer_size:
            self.flush_log()
    
    def flush_log(self):
        """Write buffered data to CSV file"""
        if self.log_writer and self.log_data:
            try:
                self.log_writer.writerows(self.log_data)
                self.log_file.flush()
                self.log_data = []
            except Exception as e:
                self.get_logger().error(f'Error writing to log: {e}')
    
    def close_log_file(self):
        """Close and finalize the log file"""
        self.flush_log()
        if self.log_file:
            self.log_file.close()
            self.get_logger().info(f'✓ Log file closed: {self.log_path}')
    
    def compute_control_torques(self, idx):
        """
        Compute control torques using computed torque control law:
        τ_total = τ_model + τ_fb
        τ_model = D(q)q̈ + C(q,q̇) + G(q)
        τ_fb = Kp*e_pos + Kd*e_vel
        
        Returns: tau_model, tau_fb, tau_total, D, C, G
        """
        # Get desired states at this timestep
        q_des = np.array([self.dp1[idx], self.dp2[idx], self.dp3[idx]])
        qd_des = np.array([self.dv1[idx], self.dv2[idx], self.dv3[idx]])
        qdd_des = np.array([self.da1[idx], self.da2[idx], self.da3[idx]])
        
        # Compute errors
        e_pos = q_des - self.current_joint_pos
        e_vel = qd_des - self.current_joint_vel
        
        # Integral action in trajectory phase with anti-windup.
        self.traj_integral_error += e_pos * self.dt
        max_i = np.array([0.4, 0.8, 0.8])
        self.traj_integral_error = np.clip(self.traj_integral_error, -max_i, max_i)

        v_fb = np.zeros(3)
        if self.use_feedback:
            v_fb = self.kp * e_pos + self.kd * e_vel + self.ki * self.traj_integral_error

        # Full CTC virtual acceleration.
        v = qdd_des + v_fb

        if self.use_model and HAS_INV_DYN:
            try:
                if self.dynamics_frame == 'actual':
                    q_dyn = self.current_joint_pos
                    qd_dyn = self.current_joint_vel
                else:
                    q_dyn = q_des
                    qd_dyn = qd_des

                # Feedforward model term and full model+feedback term.
                tau_model, _, _, _ = compute_inverse_dynamics(q_dyn, qd_dyn, qdd_des)
                tau_total, D, C, G = compute_inverse_dynamics(q_dyn, qd_dyn, v)
                tau_fb = tau_total - tau_model
            except Exception as e:
                self.get_logger().warn(f'Inverse dynamics computation failed: {e}')
                tau_model = np.zeros(3)
                tau_fb = v_fb
                tau_total = tau_fb
                D = np.eye(3)
                C = np.zeros(3)
                G = np.zeros(3)
        else:
            tau_model = np.zeros(3)
            tau_fb = v_fb
            tau_total = tau_fb
            D = np.eye(3)
            C = np.zeros(3)
            G = np.zeros(3)

        tau_total = np.clip(tau_total, -self.torque_limits, self.torque_limits)
        
        return tau_model, tau_fb, tau_total, D, C, G, q_des, qd_des, qdd_des, e_pos, e_vel
    
    def stabilization_callback(self):
        """Timer callback for PID stabilization at 100Hz"""
        target_pos = np.array([self.dp1[0], self.dp2[0], self.dp3[0]])
        
        # Calculate position errors
        errors = target_pos - self.current_joint_pos
        max_error = np.max(np.abs(errors))
        
        # Check convergence (strict threshold: 0.2°)
        if max_error < 0.0000035:  # 0.0002° in radians
            self.get_logger().info(f'✓ Initial position reached! Max error: {max_error*180/3.14159:.6f}°')
            if self.stabilization_timer:
                self.stabilization_timer.cancel()
            self.stabilization_complete = True
            return
        
        # Timeout check (120 seconds = 12000 iterations at 100Hz)
        self.stabilization_iterations += 1
        if self.stabilization_iterations >= 12000:
            self.get_logger().error(f'Timeout! Failed to reach 0.5° target. Current error: {max_error*180/3.14159:.6f}°')
            if self.stabilization_timer:
                self.stabilization_timer.cancel()
            self.stabilization_complete = True
            return
        
        # Full PID control with gravity compensation and integral windup protection
        kp = np.array([50.0, 200.0, 150.0])  # Proportional gains
        ki = np.array([5.0, 25.0, 20.0])     # Integral gains
        kd = np.array([12.0, 35.0, 10.0])    # Derivative gains
        
        # Accumulate integral error (with anti-windup)
        dt = 0.01  # 100Hz = 0.01s
        self.integral_error += errors * dt
        # Anti-windup: clamp integral to prevent excessive accumulation
        max_integral = np.array([0.5, 1.0, 1.0])
        self.integral_error = np.clip(self.integral_error, -max_integral, max_integral)
        
        # Gravity compensation (feedforward)
        q2 = self.current_joint_pos[1]
        q3 = self.current_joint_pos[2]
        gravity_comp = np.array([
            0.0,
            -44.0 * np.cos(q2),           
            -12.0 * np.cos(q2 + q3)       
        ])
        
        # Full PID control law: τ = Kp*e + Ki*∫e + Kd*ė + g(q)
        torques = kp * errors + ki * self.integral_error - kd * self.current_joint_vel + gravity_comp
        
        # Torque saturation
        max_torques = np.array([100.0, 100.0, 50.0])
        torques = np.clip(torques, -max_torques, max_torques)
        
        # Publish torques
        self.msg1.data = [torques[0]]
        self.msg2.data = [torques[1]]
        self.msg3.data = [torques[2]]
        
        self.pub1.publish(self.msg1)
        self.pub2.publish(self.msg2)
        self.pub3.publish(self.msg3)
        
        # Log progress every 50 iterations (0.5s)
        if self.stabilization_iterations % 50 == 0:
            self.get_logger().info(
                f'  t={self.stabilization_iterations/100:.1f}s | '
                f'Error: [{errors[0]*180/3.14159:.5f}°, {errors[1]*180/3.14159:.5f}°, {errors[2]*180/3.14159:.5f}°] | '
                f'Max: {max_error*180/3.14159:.5f}°'
            )

    def trajectory_callback(self):
        """Timer callback for trajectory execution with computed torque control at 100Hz"""
        if self.current_idx >= len(self.time_data):
            self.get_logger().info('='*80)
            self.get_logger().info('TRAJECTORY EXECUTION COMPLETED')
            self.get_logger().info('='*80)

            if self.trajectory_timer:
                self.trajectory_timer.cancel()
            self.trajectory_active = False
            return
        
        # Compute control torques using computed torque control law
        try:
            tau_model, tau_fb, tau_total, D, C, G, q_des, qd_des, qdd_des, e_pos, e_vel = \
                self.compute_control_torques(self.current_idx)
        except Exception as e:
            self.get_logger().error(f'Error computing torques at index {self.current_idx}: {e}')
            tau_total = np.zeros(3)
            tau_model = np.zeros(3)
            tau_fb = np.zeros(3)
            q_des = np.array([self.dp1[self.current_idx], self.dp2[self.current_idx], self.dp3[self.current_idx]])
            qd_des = np.array([self.dv1[self.current_idx], self.dv2[self.current_idx], self.dv3[self.current_idx]])
            qdd_des = np.array([self.da1[self.current_idx], self.da2[self.current_idx], self.da3[self.current_idx]])
            e_pos = q_des - self.current_joint_pos
            e_vel = qd_des - self.current_joint_vel
        
        # Publish computed torques
        self.msg1.data = [tau_total[0]]
        self.msg2.data = [tau_total[1]]
        self.msg3.data = [tau_total[2]]
        
        self.pub1.publish(self.msg1)
        self.pub2.publish(self.msg2)
        self.pub3.publish(self.msg3)
        
        # Log data every timestep
        if self.log_writer:
            self.log_timestep(
                self.time_data[self.current_idx],
                q_des, qd_des, qdd_des,
                self.current_joint_pos, self.current_joint_vel,
                tau_model, tau_fb, tau_total, self.current_joint_efforts,
                e_pos, e_vel
            )
        
        # Increment index
        self.current_idx += 1
        
        # Optional: Log progress every 50 points (0.5s)
        if self.current_idx % 50 == 0 and self.current_idx < len(self.time_data):
            max_err_deg = np.max(np.abs(e_pos)) * 180 / 3.14159
            
            self.get_logger().info(
                f't={self.time_data[self.current_idx]:.1f}s | '
                f'idx={self.current_idx}/{len(self.time_data)} | '
                f'τ=[{tau_total[0]:.1f}, {tau_total[1]:.1f}, {tau_total[2]:.1f}] Nm | '
                f'err={max_err_deg:.1f}°'
            )
    
    def run(self):
        """Main execution sequence"""
        self.get_logger().info('Waiting for joint states...')
        
        # Wait for joint states (blocking)
        while not self.joint_states_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.joint_states_received:
            self.get_logger().error('Failed to receive joint states!')
            return
        
        self.get_logger().info(f'Current position: [{self.current_joint_pos[0]:.4f}, {self.current_joint_pos[1]:.4f}, {self.current_joint_pos[2]:.4f}] rad')

        target_pos = np.array([self.dp1[0], self.dp2[0], self.dp3[0]])
        initial_errors = target_pos - self.current_joint_pos
        initial_max_error = np.max(np.abs(initial_errors))
        skip_stabilization = initial_max_error <= self.skip_stabilization_threshold_rad
        
        if skip_stabilization:
            self.get_logger().info('='*80)
            self.get_logger().info('PHASE 1: SKIPPED (Already near CSV start pose)')
            self.get_logger().info('='*80)
            self.get_logger().info(
                f'Initial max error: {np.degrees(initial_max_error):.4f} deg '
                f'<= threshold {np.degrees(self.skip_stabilization_threshold_rad):.4f} deg'
            )
        else:
            # Phase 1: Move to initial position using PID control
            self.get_logger().info('='*80)
            self.get_logger().info('PHASE 1: STABILIZATION (Moving to initial position)')
            self.get_logger().info('='*80)
            self.get_logger().info('Target: 0.5° convergence with full PID + gravity compensation')
            self.get_logger().info(
                f'Initial max error: {np.degrees(initial_max_error):.4f} deg '
                f'> threshold {np.degrees(self.skip_stabilization_threshold_rad):.4f} deg'
            )
            self.stabilization_iterations = 0
            self.stabilization_complete = False
            self.integral_error = np.array([0.0, 0.0, 0.0])  # Reset integral error
            self.stabilization_timer = self.create_timer(0.01, self.stabilization_callback)  # 100 Hz

            # Wait for stabilization to complete
            while not self.stabilization_complete and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.001)

            if not rclpy.ok():
                return

            self.get_logger().info('Holding position for 1 second...')
            # Hold with timer (100Hz)
            hold_iterations = [0]

            def hold_callback():
                hold_iterations[0] += 1
                if hold_iterations[0] >= 100:  # 1 second
                    hold_timer.cancel()
                    return
                # Keep publishing last stabilization torque
                self.stabilization_callback()

            hold_timer = self.create_timer(0.01, hold_callback)
            while hold_iterations[0] < 100 and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.001)
        
        # Launch logger subprocess
        self.get_logger().info('='*80)
        self.get_logger().info('Launching data logger...')
        if not self.launch_logger():
            self.get_logger().error('Failed to launch logger, continuing without logging')
        
        # Open local log file for comprehensive data capture
        self.get_logger().info('='*80)
        self.get_logger().info('Opening local log file for computed torque data...')
        if not self.open_log_file():
            self.get_logger().warning('Failed to open local log file')
        
        # Draw expected Cartesian trajectory before torque commands start.
        self.publish_expected_path_marker()
        
        # Phase 2: Execute trajectory with computed torque control
        self.get_logger().info('='*80)
        self.get_logger().info('PHASE 2: TRAJECTORY EXECUTION (Computed Torque Control)')
        self.get_logger().info('='*80)
        self.get_logger().info(f'Publishing {len(self.time_data)} torque commands at 100Hz...')
        self.get_logger().info(f'Control law: τ = τ_model + τ_fb (model={self.use_model}, feedback={self.use_feedback})')
        
        # Start logger 100ms before trajectory execution
        time.sleep(0.1)
        if self.logger_start_client:
            self.start_logger()
        
        # Small delay to ensure logger is recording before first torque command
        time.sleep(0.01)
        
        self.current_idx = 0
        self.traj_integral_error = np.array([0.0, 0.0, 0.0])
        self.trajectory_active = True
        
        # Create 100Hz timer for trajectory execution
        # Period = 1/100 = 0.01 seconds = 10ms
        self.trajectory_timer = self.create_timer(0.01, self.trajectory_callback)
        
        # Spin until trajectory completes
        while self.trajectory_active and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)
        
        # Stop logger 100ms after trajectory completion
        time.sleep(0.1)
        if self.logger_stop_client:
            self.stop_logger()
        
        # Close local log file
        self.close_log_file()
        
        # Give logger time to write final data
        time.sleep(0.1)
        
        self.get_logger().info('Computed torque control publisher shutting down.')
        self.get_logger().info(f'Data logged to: {self.log_path}')

def main(args=None):
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Computed Torque Control Publisher')
    parser.add_argument(
        '--csv-path',
        type=str,
        default=None,
        help='Path to the CSV file containing trajectory data'
    )
    parser.add_argument(
        '--skip-stabilization-threshold-deg',
        type=float,
        default=1.0,
        help='Skip stabilization if initial max joint error is <= this threshold in degrees (default: 1.0)'
    )
    parser.add_argument(
        '--kp',
        type=float,
        nargs=3,
        default=None,
        help='Proportional gains [kp1, kp2, kp3] (default: 30 100 50)'
    )
    parser.add_argument(
        '--kd',
        type=float,
        nargs=3,
        default=None,
        help='Derivative gains [kd1, kd2, kd3] (default: 4 10 5)'
    )
    parser.add_argument(
        '--ki',
        type=float,
        nargs=3,
        default=None,
        help='Integral gains [ki1, ki2, ki3] (default: 0.2 1.0 0.8)'
    )
    parser.add_argument(
        '--dynamics-frame',
        type=str,
        choices=['actual', 'desired'],
        default='desired',
        help='State used for D,C,G evaluation (default: desired for consistent model decomposition)'
    )
    parser.add_argument(
        '--torque-limits',
        type=float,
        nargs=3,
        default=None,
        help='Torque limits [tau1 tau2 tau3] Nm (default: 100 100 60)'
    )
    parser.add_argument(
        '--vel-filter-alpha',
        type=float,
        default=0.25,
        help='Joint velocity low-pass alpha in [0,1] (default: 0.25)'
    )
    parser.add_argument(
        '--no-feedback',
        action='store_true',
        help='Disable feedback control (model-based only)'
    )
    parser.add_argument(
        '--no-model',
        action='store_true',
        help='Disable model-based control (feedback only)'
    )
    parser.add_argument(
        '--log-path',
        type=str,
        default=None,
        help='Output path for log CSV file'
    )
    parser.add_argument(
        '--q-end',
        type=float,
        nargs=3,
        default=None,
        metavar=('Q1_DEG', 'Q2_DEG', 'Q3_DEG'),
        help=(
            'End joint positions in degrees [q1, q2, q3]. '
            'Generates a min-jerk trajectory on the fly and saves it before running CTC. '
            'Mutually exclusive with --csv-path.'
        ),
    )
    parser.add_argument(
        '--q-start',
        type=float,
        nargs=3,
        default=[0.0, 45.0, 135.0],
        metavar=('Q1_DEG', 'Q2_DEG', 'Q3_DEG'),
        help='Start joint positions in degrees for on-the-fly generation (default: 0 45 135)',
    )
    parsed_args = parser.parse_args()

    # --- On-the-fly trajectory generation from end-point ---
    if parsed_args.q_end is not None:
        if parsed_args.csv_path is not None:
            print('Error: --q-end and --csv-path are mutually exclusive.', file=sys.stderr)
            sys.exit(1)

        q_start_rad = np.deg2rad(parsed_args.q_start)
        q_end_rad   = np.deg2rad(parsed_args.q_end)

        save_dir = TorquePublisher.LOGS_DIR
        os.makedirs(save_dir, exist_ok=True)

        q_tag = '_'.join(f'{v:.1f}' for v in parsed_args.q_end)
        save_path = os.path.join(save_dir, f'gen_traj_qend_{q_tag}.csv')

        print(f'Generating min-jerk trajectory: q_start={parsed_args.q_start} deg -> q_end={parsed_args.q_end} deg')
        _, T_total = TorquePublisher.generate_and_save_trajectory(
            q_start_rad, q_end_rad, save_path
        )
        print(f'Trajectory saved ({T_total:.2f}s): {save_path}')
        parsed_args.csv_path = save_path

    rclpy.init(args=args)
    node = TorquePublisher(
        csv_path=parsed_args.csv_path,
        skip_stabilization_threshold_deg=parsed_args.skip_stabilization_threshold_deg,
        kp=parsed_args.kp,
        kd=parsed_args.kd,
        ki=parsed_args.ki,
        use_feedback=not parsed_args.no_feedback,
        use_model=not parsed_args.no_model,
        dynamics_frame=parsed_args.dynamics_frame,
        torque_limits=parsed_args.torque_limits,
        vel_filter_alpha=parsed_args.vel_filter_alpha,
        log_path=parsed_args.log_path,
    )
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        # Shutdown logger if running
        node.shutdown_logger()
        
        # Close log file
        node.close_log_file()

        # Zero out torques on shutdown
        zero_msg = Float64MultiArray()
        zero_msg.data = [0.0]
        node.pub1.publish(zero_msg)
        node.pub2.publish(zero_msg)
        node.pub3.publish(zero_msg)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

