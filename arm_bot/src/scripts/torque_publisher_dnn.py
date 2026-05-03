#!/usr/bin/env python3
"""
DNN Torque Publisher — Online Inference
Implements τ = τ_dnn(q, q̇, q̈) + Kp·e + Kd·ė + Ki·∫e dt

τ_dnn is predicted ONLINE at every 100 Hz timestep:
  Step 1 — DeLaN (JAX physics baseline): instant, any timestep
  Step 2 — GRU residual (PyTorch):       applied once 128-step history is built
            During the 128-step warmup, DeLaN-only torque is used.

Models are loaded once at startup from DNN_TEST_DIR.
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
from collections import deque

# ── DNN model imports (heavy — only loaded when the node actually starts) ──
DNN_TEST_DIR = '/home/priyankan/Desktop/FYP-Puma_560/DNN_test'


# ======================================================================== #
#  Online DNN inference engine                                              #
# ======================================================================== #
class DNNInferenceEngine:
    """
    Loads DeLaN (JAX/Haiku) and GRU residual (PyTorch) once at construction,
    then exposes a predict() method for per-timestep online inference.
    """

    SEQ_LEN   = 128
    N_DOF     = 3
    INPUT_DIM = 12   # q(3) + qd(3) + qdd(3) + tau_delan(3)
    HIDDEN_DIM = 64
    N_LAYERS   = 4

    def __init__(self, delan_path, gru_path=None, scaler_path=None, use_gru=True):
        # Prevent JAX from reserving all GPU memory
        os.environ['XLA_PYTHON_CLIENT_PREALLOCATE'] = 'false'
        os.environ['XLA_PYTHON_CLIENT_ALLOCATOR']   = 'platform'

        import torch
        import torch.nn as nn
        import dill as pickle
        import joblib
        import jax
        import jax.numpy as jnp
        import haiku as hk
        import functools

        self._torch  = torch
        self._jnp    = jnp
        self._jax    = jax
        self.device  = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # ── DeLaN ────────────────────────────────────────────────────────
        print(f'  Loading DeLaN model from {delan_path}')
        with open(delan_path, 'rb') as f:
            saved = pickle.load(f)
        params, hyper = saved['params'], saved['hyper']
        activations   = {'tanh': jnp.tanh, 'softplus': jax.nn.softplus}

        def mass_matrix_fn(q, n_dof, shape, activation, epsilon, shift):
            n_out     = int((n_dof**2 + n_dof) / 2)
            idx_diag  = (np.arange(n_dof) + 1)
            idx_diag  = (idx_diag * (idx_diag + 1) / 2 - 1).astype(int)
            idx_tril  = np.setdiff1d(np.arange(n_out), idx_diag)
            cat_idx   = np.hstack((idx_diag, idx_tril))
            idx       = np.arange(cat_idx.size)[np.argsort(cat_idx)]
            mat_idx   = np.tril_indices(n_dof)
            net = hk.nets.MLP(output_sizes=shape + (n_out,), activation=activation,
                               name='mass_matrix')
            z   = jnp.concatenate([jnp.cos(q), jnp.sin(q)], axis=-1)
            l_d, l_od = jnp.split(net(z), [n_dof], axis=-1)
            l_d = jax.nn.softplus(l_d + shift) + epsilon
            vec = jnp.concatenate((l_d, l_od), axis=-1)[..., idx]
            L   = jnp.zeros((n_dof, n_dof)).at[mat_idx].set(vec[:])
            return jnp.matmul(L, L.transpose())

        def kinetic_energy_fn(q, qd, n_dof, shape, activation, epsilon, shift):
            M = mass_matrix_fn(q, n_dof, shape, activation, epsilon, shift)
            return 0.5 * jnp.dot(qd, jnp.dot(M, qd))

        def potential_energy_fn(q, shape, activation):
            net = hk.nets.MLP(output_sizes=shape + (1,), activation=activation,
                               name='potential_energy')
            z   = jnp.concatenate([jnp.cos(q), jnp.sin(q)], axis=-1)
            return net(z)

        def structured_lagrangian_fn(q, qd, n_dof, shape, activation, epsilon, shift):
            return (kinetic_energy_fn(q, qd, n_dof, shape, activation, epsilon, shift)
                    - potential_energy_fn(q, shape, activation).squeeze())

        def dynamics_model(params, key, q, qd, qdd, tau, lagrangian, n_dof):
            vmap_dim = (None, None, 0, 0)
            batch_mm = jax.vmap(jnp.matmul, (0, 0))
            lag_vg   = jax.value_and_grad(lagrangian, argnums=[2, 3])
            L, (dLdq, dLdqd) = jax.vmap(lag_vg, vmap_dim)(params, key, q, qd)
            lag_hess = jax.hessian(lagrangian, argnums=[2, 3])
            (_, (d2L_dqddq, d2Ld2qd)) = jax.vmap(lag_hess, vmap_dim)(params, key, q, qd)
            tau_pred = batch_mm(d2Ld2qd, qdd) + batch_mm(d2L_dqddq, qd) - dLdq
            return None, tau_pred, None, None

        shape      = (hyper['n_width'],) * hyper['n_depth']
        activation = activations[hyper['activation']]
        epsilon    = hyper['diagonal_epsilon']
        shift      = hyper['diagonal_shift']

        lag_fn = hk.transform(functools.partial(
            structured_lagrangian_fn, n_dof=self.N_DOF,
            shape=shape, activation=activation, epsilon=epsilon, shift=shift))

        self._delan_params = params
        self._delan_fn = jax.jit(functools.partial(
            dynamics_model, lagrangian=lag_fn.apply, n_dof=self.N_DOF))

        # Warm up JAX JIT with a dummy call so first real call isn't slow
        print('  Warming up JAX JIT...')
        _q0  = jnp.zeros((1, self.N_DOF))
        _qd0 = jnp.zeros((1, self.N_DOF))
        self._delan_fn(params, None, _q0, _qd0, _q0, _q0)
        print('  DeLaN ready.')

        self._use_gru = bool(use_gru)
        self._gru = None
        self._scaler = None

        # ── GRU residual ─────────────────────────────────────────────────
        if self._use_gru:
            if not gru_path or not scaler_path:
                raise ValueError('gru_path and scaler_path are required when use_gru=True')

            print(f'  Loading GRU model from {gru_path}')

            class GRUResidual(nn.Module):
                def __init__(self):
                    super().__init__()
                    self.gru = nn.GRU(
                        input_size=DNNInferenceEngine.INPUT_DIM,
                        hidden_size=DNNInferenceEngine.HIDDEN_DIM,
                        num_layers=DNNInferenceEngine.N_LAYERS,
                        batch_first=True, dropout=0.1)
                    self.fc = nn.Linear(DNNInferenceEngine.HIDDEN_DIM, DNNInferenceEngine.N_DOF)
                def forward(self, x):
                    h, _ = self.gru(x)
                    return self.fc(h[:, -1, :])

            self._gru = GRUResidual().to(self.device)
            self._gru.load_state_dict(torch.load(gru_path, map_location=self.device))
            self._gru.eval()

            # ── Scaler ───────────────────────────────────────────────────
            print(f'  Loading feature scaler from {scaler_path}')
            import joblib
            self._scaler = joblib.load(scaler_path)
        else:
            print('  GRU residual disabled; using DeLaN only.')

        # Rolling feature buffer  [q, qd, qdd, tau_delan]  shape (12,) each step
        self._buffer = deque(maxlen=self.SEQ_LEN)
        print('  DNNInferenceEngine ready.')

    # ------------------------------------------------------------------ #
    def predict(self, q, qd, qdd):
        """
        Online per-timestep inference.

        Parameters
        ----------
        q, qd, qdd : array-like (3,)  desired kinematics at current step

        Returns
        -------
        tau_dnn    : np.ndarray (3,)  DeLaN + GRU (or DeLaN-only during warmup)
        tau_delan  : np.ndarray (3,)  physics baseline only
        gru_active : bool             False during the 128-step warmup period
        """
        jnp = self._jnp
        q_j   = jnp.array(q,   dtype=jnp.float32)[None, :]   # (1,3)
        qd_j  = jnp.array(qd,  dtype=jnp.float32)[None, :]
        qdd_j = jnp.array(qdd, dtype=jnp.float32)[None, :]

        _, tau_d, _, _ = self._delan_fn(
            self._delan_params, None, q_j, qd_j, qdd_j, jnp.zeros_like(q_j))
        tau_delan = np.array(tau_d[0], dtype=np.float64)

        if not self._use_gru:
            return tau_delan.copy(), tau_delan.copy(), False

        # Append normalised features to buffer
        raw_feat = np.concatenate([q, qd, qdd, tau_delan]).reshape(1, -1)  # (1,12)
        self._buffer.append(self._scaler.transform(raw_feat)[0])           # (12,)

        if len(self._buffer) < self.SEQ_LEN:
            return tau_delan.copy(), tau_delan.copy(), False

        # Build (1, SEQ_LEN, 12) tensor and run GRU
        window = self._torch.FloatTensor(
            np.array(self._buffer)[None, :, :]).to(self.device)      # (1,128,12)
        with self._torch.no_grad():
            tau_res = self._gru(window).cpu().numpy()[0]              # (3,)

        tau_dnn = tau_delan + tau_res
        return tau_dnn, tau_delan, True


# ======================================================================== #
#  ROS 2 node                                                               #
# ======================================================================== #
class DNNTorquePublisher(Node):

    LOGS_DIR = '/home/priyankan/Desktop/FYP-Puma_560/arm_bot/src/scripts/logs'

    def __init__(self, csv_path,
                 delan_path, gru_path, scaler_path,
                 skip_stabilization_threshold_deg=1.0,
                 kp=None, kd=None, ki=None,
                 use_feedback=True,
                 use_model=True,
                 use_gru=True,
                 torque_limits=None,
                 vel_filter_alpha=0.25,
                 log_path=None):
        super().__init__('dnn_torque_publisher')

        self.use_feedback    = use_feedback
        self.use_model       = use_model
        self.use_gru         = use_gru
        self.mode_key, self.mode_label = self._resolve_mode()
        self.vel_filter_alpha = float(np.clip(vel_filter_alpha, 0.0, 1.0))
        # self.kp = np.array(kp if kp is not None else [30.0, 80.0, 40.0 ]) #old 5.0, 20.0, 10.0
        # self.kd = np.array(kd if kd is not None else [5.0, 12.0, 8.0 ]) #old 1.0,  3.0,  2.0 
        # self.ki = np.array(ki if ki is not None else [0.5, 1.0, 0.5 ]) #old 0.05, 0.2,  0.1 
        self.kp = np.array(kp if kp is not None else [ 15.0, 25.0, 13.0 ])
        self.kd = np.array(kd if kd is not None else [ 2.0,  5.0,  2.5 ])
        self.ki = np.array(ki if ki is not None else [ 0.1, 0.2,  0.1 ])
        self.torque_limits = np.array(
            torque_limits if torque_limits is not None else [100.0, 100.0, 60.0])

        # Publishers
        self.pub1 = self.create_publisher(Float64MultiArray, '/joint_1_controller/commands', 10)
        self.pub2 = self.create_publisher(Float64MultiArray, '/joint_2_controller/commands', 10)
        self.pub3 = self.create_publisher(Float64MultiArray, '/joint_3_controller/commands', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        state_qos = QoSProfile(depth=1,
                               reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.recording_state_pub = self.create_publisher(
            Bool, '/line_drawer/recording_active', state_qos)
        self._publish_recording_state(False)

        # Subscriber
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # Load trajectory CSV (same format as CTC)
        self.csv_path = os.path.expanduser(csv_path)
        self.load_trajectory_data()

        # Load DNN models (only if using model)
        if self.use_model:
            if self.use_gru:
                self.get_logger().info('Loading DNN models (DeLaN + GRU)...')
            else:
                self.get_logger().info('Loading DeLaN model only...')
            self.dnn = DNNInferenceEngine(
                delan_path,
                gru_path=gru_path,
                scaler_path=scaler_path,
                use_gru=self.use_gru,
            )
            self.get_logger().info('DNN models loaded and ready.')
        else:
            self.dnn = None

        # Pre-allocate messages
        self.msg1 = Float64MultiArray()
        self.msg2 = Float64MultiArray()
        self.msg3 = Float64MultiArray()

        # State
        self.current_idx       = 0
        self.current_joint_pos = np.zeros(3)
        self.current_joint_vel = np.zeros(3)
        self.current_joint_efforts = np.zeros(3)  # Sensed torques from Gazebo
        self.joint_states_received  = False
        self.trajectory_active      = False
        self.trajectory_timer       = None
        self.stabilization_timer    = None
        self.stabilization_complete = False
        self.stabilization_iterations = 0
        self.integral_error      = np.zeros(3)
        self.traj_integral_error = np.zeros(3)

        # Logger subprocess
        self.logger_process     = None
        self.logger_start_client = None
        self.logger_stop_client  = None

        # Log file
        self.log_path   = log_path or self._build_incremental_log_path(self.csv_path, self.mode_key)
        self.log_file   = None
        self.log_writer = None
        self.log_data   = []
        self.log_buffer_size = 100
        self.current_path_points = []

        self.skip_stabilization_threshold_rad = math.radians(skip_stabilization_threshold_deg)

        self.get_logger().info('=' * 80)
        self.get_logger().info(self.mode_label)
        self.get_logger().info('=' * 80)
        self.get_logger().info(
            f'Trajectory: {self.csv_path}  ({self.n_points} pts, {self.time_data[-1]:.2f}s)')
        self.get_logger().info(
            f'Model: {self.use_model}  Feedback: {self.use_feedback}  '
            f'GRU: {self.use_gru}  '
            f'Kp={self.kp.tolist()}  Kd={self.kd.tolist()}  Ki={self.ki.tolist()}')
        if self.use_model and self.use_gru:
            self.get_logger().info(f'GRU warmup: first {self.dnn.SEQ_LEN} steps use DeLaN only')
        self.get_logger().info(f'Torque limits: {self.torque_limits.tolist()} Nm')
        self.get_logger().info(f'Log: {self.log_path}')

    # ------------------------------------------------------------------ #
    def _resolve_mode(self):
        if not self.use_model and self.use_feedback:
            return 'pid', 'PID FEEDBACK ONLY'
        if not self.use_model and not self.use_feedback:
            return 'open_loop', 'OPEN LOOP'
        if self.use_model and self.use_gru and self.use_feedback:
            return 'pid_dnn', 'PID + DNN (DeLaN + GRU)'
        if self.use_model and self.use_gru:
            return 'dnn', 'DNN (DeLaN + GRU)'
        if self.use_model and self.use_feedback:
            return 'pid_delan', 'PID + DeLaN ONLY'
        if self.use_model:
            return 'delan', 'DeLaN ONLY'
        return 'open_loop', 'OPEN LOOP'

    # ------------------------------------------------------------------ #
    @staticmethod
    def _build_incremental_log_path(csv_path: str, mode_key: str) -> str:
        base_dir = DNNTorquePublisher.LOGS_DIR
        os.makedirs(base_dir, exist_ok=True)
        name   = os.path.splitext(os.path.basename(os.path.expanduser(csv_path)))[0]
        prefix = f'{name}_{mode_key}_log_'
        pat    = re.compile(rf'^{re.escape(prefix)}(\d+)\.csv$')
        max_r  = 0
        try:
            for fn in os.listdir(base_dir):
                m = pat.match(fn)
                if m:
                    max_r = max(max_r, int(m.group(1)))
        except OSError:
            pass
        return os.path.join(base_dir, f'{prefix}{max_r + 1}.csv')

    # ------------------------------------------------------------------ #
    def load_trajectory_data(self):
        """Load trajectory CSV (same row format as CTC)."""
        data = {}
        with open(self.csv_path, 'r') as f:
            for row in csv.reader(f):
                if row:
                    data[row[0].strip()] = [float(v) for v in row[1:]]

        self.time_data = np.array(data['t'])
        self.dp1 = np.array(data['dp1']); self.dp2 = np.array(data['dp2'])
        self.dp3 = np.array(data['dp3'])
        self.dv1 = np.array(data['dv1']); self.dv2 = np.array(data['dv2'])
        self.dv3 = np.array(data['dv3'])
        if 'da1' in data:
            self.da1 = np.array(data['da1']); self.da2 = np.array(data['da2'])
            self.da3 = np.array(data['da3'])
        else:
            self.da1 = np.gradient(self.dv1, self.time_data)
            self.da2 = np.gradient(self.dv2, self.time_data)
            self.da3 = np.gradient(self.dv3, self.time_data)

        self.dt       = self.time_data[1] - self.time_data[0] if len(self.time_data) > 1 else 0.01
        self.n_points = len(self.time_data)

    # ------------------------------------------------------------------ #
    @staticmethod
    def _path_color(r, g, b, a=0.95):
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
        mk.header.stamp    = self.get_clock().now().to_msg()
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

    def _load_log_tip_points(self, log_path):
        points = []
        try:
            with open(log_path, 'r', newline='') as f:
                reader = csv.DictReader(f)
                required = {'q_act_1', 'q_act_2', 'q_act_3'}
                if not reader.fieldnames or not required.issubset(reader.fieldnames):
                    return points
                for row in reader:
                    try:
                        q1 = float(row['q_act_1'])
                        q2 = float(row['q_act_2'])
                        q3 = float(row['q_act_3'])
                    except (TypeError, ValueError, KeyError):
                        continue
                    points.append(self._fk_tip_world(q1, q2, q3))
        except OSError:
            return []
        return points

    def _historical_log_candidates(self):
        base_dir = DNNTorquePublisher.LOGS_DIR
        if not os.path.isdir(base_dir):
            return []

        stem = os.path.splitext(os.path.basename(os.path.expanduser(self.csv_path)))[0]
        mode_patterns = [
            ('pid', re.compile(rf'^{re.escape(stem)}_pid_log_(\d+)\.csv$')),
            ('delan', re.compile(rf'^{re.escape(stem)}_delan_log_(\d+)\.csv$')),
            ('dnn', re.compile(rf'^{re.escape(stem)}_dnn_log_(\d+)\.csv$')),
            ('pid_delan', re.compile(rf'^{re.escape(stem)}_pid_delan_log_(\d+)\.csv$')),
            ('pid_dnn', re.compile(rf'^{re.escape(stem)}_pid_dnn_log_(\d+)\.csv$')),
        ]

        latest = {}
        for filename in os.listdir(base_dir):
            full_path = os.path.join(base_dir, filename)
            if not os.path.isfile(full_path):
                continue

            matched_mode = None
            matched_run = None
            for mode_name, pattern in mode_patterns:
                m = pattern.match(filename)
                if m:
                    matched_mode = mode_name
                    matched_run = int(m.group(1))
                    break

            if matched_mode is None:
                legacy_pattern = re.compile(rf'^{re.escape(stem)}_dnn_log_(\d+)\.csv$')
                m = legacy_pattern.match(filename)
                if m:
                    matched_mode = 'pid_dnn'
                    matched_run = int(m.group(1))

            if matched_mode is None or matched_mode == self.mode_key:
                continue

            current = latest.get(matched_mode)
            if current is None or matched_run > current[0]:
                latest[matched_mode] = (matched_run, full_path)

        ordered_modes = ['pid', 'delan', 'dnn', 'pid_delan', 'pid_dnn']
        return [latest[m][1] for m in ordered_modes if m in latest]

    # ------------------------------------------------------------------ #
    #  FK for Cartesian path marker (same as CTC)                         #
    # ------------------------------------------------------------------ #
    @staticmethod
    def _mat_mul(a, b):
        return [[sum(a[r][k]*b[k][c] for k in range(3)) for c in range(3)]
                for r in range(3)]

    @staticmethod
    def _mat_vec_mul(a, v):
        return [sum(a[r][c]*v[c] for c in range(3)) for r in range(3)]

    @staticmethod
    def _rot_x(a):
        c, s = math.cos(a), math.sin(a)
        return [[1,0,0],[0,c,-s],[0,s,c]]

    @staticmethod
    def _rot_z(a):
        c, s = math.cos(a), math.sin(a)
        return [[c,-s,0],[s,c,0],[0,0,1]]

    @staticmethod
    def _compose(r, p, rl, pl):
        rp = DNNTorquePublisher._mat_vec_mul(r, pl)
        return (DNNTorquePublisher._mat_mul(r, rl),
                [p[0]+rp[0], p[1]+rp[1], p[2]+rp[2]])

    def _fk_tip_world(self, q1, q2, q3):
        r = [[1,0,0],[0,1,0],[0,0,1]]; p = [0,0,0]
        for rl, pl in [
            (self._rot_z(0), [0.5,0.5,0]),  (self._rot_z(0), [0,0,0.1]),
            (self._rot_z(0), [0,0,0.6718]), (self._rot_z(q1), [0,0,0]),
            (self._rot_x(-math.pi/2), [0,0.2435,0]), (self._rot_z(q2), [0,0,0]),
            (self._rot_z(0), [0.4318,0,-0.094]),      (self._rot_z(q3), [0,0,0]),
        ]:
            r, p = self._compose(r, p, rl, pl)
        t = self._mat_vec_mul(r, [0,-0.233,0])
        return [p[0]+t[0], p[1]+t[1], p[2]+t[2]]

    def publish_expected_path_marker(self):
        desired_points = [self._fk_tip_world(q1, q2, q3)
                          for q1, q2, q3 in zip(self.dp1, self.dp2, self.dp3)]
        self._publish_line_strip_marker(
            desired_points, 'expected_path', 0, self._path_color(1.0, 1.0, 1.0, 0.95))
        self.get_logger().info(f'Published expected path ({len(desired_points)} points)')

        historical_logs = self._historical_log_candidates()
        for idx, log_path in enumerate(historical_logs, start=1):
            points = self._load_log_tip_points(log_path)
            if not points:
                continue
            namespace = f'historical_path_{idx}'
            color = self._path_color(0.85, 0.45, 0.15, 0.6)
            self._publish_line_strip_marker(points, namespace, idx, color, scale=0.01)
            self.get_logger().info(
                f'Published historical path from {os.path.basename(log_path)} ({len(points)} points)')

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

    # ------------------------------------------------------------------ #
    def joint_state_callback(self, msg):
        try:
            i1 = msg.name.index('joint_1')
            i2 = msg.name.index('joint_2')
            i3 = msg.name.index('joint_3')
            self.current_joint_pos = np.array(
                [msg.position[i1], msg.position[i2], msg.position[i3]])
            if len(msg.velocity) >= 3:
                raw = np.array([msg.velocity[i1], msg.velocity[i2], msg.velocity[i3]])
                a   = self.vel_filter_alpha
                self.current_joint_vel = (1.0 - a) * self.current_joint_vel + a * raw
            if len(msg.effort) >= 3:
                self.current_joint_efforts = np.array(
                    [msg.effort[i1], msg.effort[i2], msg.effort[i3]])
            self.joint_states_received = True
        except (ValueError, IndexError):
            pass

    # ------------------------------------------------------------------ #
    #  Logger subprocess                                                   #
    # ------------------------------------------------------------------ #
    def _publish_recording_state(self, active):
        msg = Bool(); msg.data = bool(active)
        self.recording_state_pub.publish(msg)

    def launch_logger(self):
        try:
            script_dir  = os.path.dirname(os.path.abspath(__file__))
            logger_path = os.path.join(script_dir, 'continuous_logger_triggered.py')
            cmd = ['python3', logger_path, '--dataset-path', self.csv_path]
            self.logger_process     = subprocess.Popen(cmd, stdout=None, stderr=None)
            time.sleep(0.1)
            self.logger_start_client = self.create_client(Trigger, '/logger/start')
            self.logger_stop_client  = self.create_client(Trigger, '/logger/stop')
            t0 = time.time()
            while not self.logger_start_client.wait_for_service(timeout_sec=0.1):
                if time.time() - t0 > 5.0:
                    self.get_logger().error('Logger service unavailable!')
                    return False
                rclpy.spin_once(self, timeout_sec=0.01)
            self.get_logger().info('✓ Logger ready')
            return True
        except Exception as e:
            self.get_logger().error(f'Logger launch failed: {e}')
            return False

    def _call_service(self, client, timeout=1.0):
        fut = client.call_async(Trigger.Request())
        t0  = time.time()
        while not fut.done():
            if time.time() - t0 > timeout:
                return None
            rclpy.spin_once(self, timeout_sec=0.01)
        return fut.result()

    def start_logger(self):
        if not self.logger_start_client:
            return False
        res = self._call_service(self.logger_start_client)
        if res and res.success:
            self.get_logger().info('✓ Logger recording started')
            self._publish_recording_state(True)
        return res.success if res else False

    def stop_logger(self):
        if not self.logger_stop_client:
            return False
        res = self._call_service(self.logger_stop_client)
        if res and res.success:
            self.get_logger().info(f'✓ Logger stopped: {res.message}')
            self._publish_recording_state(False)
        return res.success if res else False

    def shutdown_logger(self):
        self._publish_recording_state(False)
        if self.logger_process:
            self.logger_process.terminate()
            try:
                self.logger_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.logger_process.kill()

    # ------------------------------------------------------------------ #
    #  Log file                                                            #
    # ------------------------------------------------------------------ #
    def open_log_file(self):
        try:
            os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
            self.log_file   = open(self.log_path, 'w', newline='')
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow([
                't',
                'q_des_1',    'q_des_2',    'q_des_3',
                'qd_des_1',   'qd_des_2',   'qd_des_3',
                'qdd_des_1',  'qdd_des_2',  'qdd_des_3',
                'q_act_1',    'q_act_2',    'q_act_3',
                'qd_act_1',   'qd_act_2',   'qd_act_3',
                'tau_delan_1','tau_delan_2','tau_delan_3',  # physics baseline
                'tau_dnn_1',  'tau_dnn_2',  'tau_dnn_3',   # DeLaN + GRU
                'tau_fb_1',   'tau_fb_2',   'tau_fb_3',    # PD+I correction
                'tau_total_1','tau_total_2','tau_total_3',
                'tau_sensed_1','tau_sensed_2','tau_sensed_3',  # Sensed torques from Gazebo
                'e_pos_1',    'e_pos_2',    'e_pos_3',
                'e_vel_1',    'e_vel_2',    'e_vel_3',
                'gru_active',                               # 0=warmup, 1=GRU on
            ])
            self.log_file.flush()
            self.get_logger().info(f'✓ Log: {self.log_path}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to open log: {e}')
            return False

    def log_timestep(self, t, q_des, qd_des, qdd_des, q_act, qd_act,
                     tau_delan, tau_dnn, tau_fb, tau_total, tau_sensed, e_pos, e_vel, gru_active):
        row = [f'{t:.3f}']
        for v in [*q_des, *qd_des, *qdd_des, *q_act, *qd_act,
                  *tau_delan, *tau_dnn, *tau_fb, *tau_total, *tau_sensed, *e_pos, *e_vel]:
            row.append(f'{v:.8f}')
        row.append('1' if gru_active else '0')
        self.log_data.append(row)
        if len(self.log_data) >= self.log_buffer_size:
            self.flush_log()

    def flush_log(self):
        if self.log_writer and self.log_data:
            try:
                self.log_writer.writerows(self.log_data)
                self.log_file.flush()
                self.log_data = []
            except Exception as e:
                self.get_logger().error(f'Log write error: {e}')

    def close_log_file(self):
        self.flush_log()
        if self.log_file:
            self.log_file.close()
            self.get_logger().info(f'✓ Log closed: {self.log_path}')

    # ------------------------------------------------------------------ #
    #  Control: online DNN + small feedback                               #
    # ------------------------------------------------------------------ #
    def compute_torques(self, idx):
        q_des   = np.array([self.dp1[idx], self.dp2[idx], self.dp3[idx]])
        qd_des  = np.array([self.dv1[idx], self.dv2[idx], self.dv3[idx]])
        qdd_des = np.array([self.da1[idx], self.da2[idx], self.da3[idx]])

        e_pos = q_des  - self.current_joint_pos
        e_vel = qd_des - self.current_joint_vel

        # Online DNN prediction (DeLaN every step, GRU after warmup if enabled)
        if self.use_model:
            tau_dnn, tau_delan, gru_active = self.dnn.predict(q_des, qd_des, qdd_des)
        else:
            tau_dnn = np.zeros(3)
            tau_delan = np.zeros(3)
            gru_active = False

        tau_fb = np.zeros(3)
        if self.use_feedback:
            self.traj_integral_error += e_pos * self.dt
            self.traj_integral_error = np.clip(
                self.traj_integral_error,
                -np.array([0.3, 0.5, 0.5]), np.array([0.3, 0.5, 0.5]))
            tau_fb = (self.kp * e_pos
                      + self.kd * e_vel
                      + self.ki * self.traj_integral_error)
            if not self.use_model:
                q2, q3 = self.current_joint_pos[1], self.current_joint_pos[2]
                gravity = np.array([0.0, -44.0 * np.cos(q2), -12.0 * np.cos(q2 + q3)])
                tau_fb = tau_fb + gravity

        tau_total = np.clip(tau_dnn + tau_fb, -self.torque_limits, self.torque_limits)
        return tau_dnn, tau_delan, tau_fb, tau_total, q_des, qd_des, qdd_des, e_pos, e_vel, gru_active

    # ------------------------------------------------------------------ #
    #  Phase 1: PID stabilisation                                         #
    # ------------------------------------------------------------------ #
    def stabilization_callback(self):
        target  = np.array([self.dp1[0], self.dp2[0], self.dp3[0]])
        errors  = target - self.current_joint_pos
        max_err = np.max(np.abs(errors))

        if max_err < 0.0000035:
            self.get_logger().info(
                f'✓ Start position reached. Max error: {math.degrees(max_err):.6f}°')
            if self.stabilization_timer:
                self.stabilization_timer.cancel()
            self.stabilization_complete = True
            return

        self.stabilization_iterations += 1
        if self.stabilization_iterations >= 12000:
            self.get_logger().error(
                f'Stabilisation timeout. Max error: {math.degrees(max_err):.4f}°')
            if self.stabilization_timer:
                self.stabilization_timer.cancel()
            self.stabilization_complete = True
            return

        kp = np.array([50.0, 200.0, 150.0])
        ki = np.array([ 5.0,  25.0,  20.0])
        kd = np.array([12.0,  35.0,  10.0])
        self.integral_error += errors * 0.01
        self.integral_error = np.clip(self.integral_error,
                                      -np.array([0.5, 1.0, 1.0]),
                                       np.array([0.5, 1.0, 1.0]))
        q2, q3 = self.current_joint_pos[1], self.current_joint_pos[2]
        gravity = np.array([0.0, -44.0*np.cos(q2), -12.0*np.cos(q2+q3)])
        torques = np.clip(kp*errors + ki*self.integral_error
                          - kd*self.current_joint_vel + gravity,
                          -np.array([100.0, 100.0, 50.0]),
                           np.array([100.0, 100.0, 50.0]))
        self.msg1.data = [torques[0]]
        self.msg2.data = [torques[1]]
        self.msg3.data = [torques[2]]
        self.pub1.publish(self.msg1)
        self.pub2.publish(self.msg2)
        self.pub3.publish(self.msg3)

        if self.stabilization_iterations % 50 == 0:
            self.get_logger().info(
                f'  t={self.stabilization_iterations/100:.1f}s  '
                f'err=[{math.degrees(errors[0]):.3f}°, {math.degrees(errors[1]):.3f}°, '
                f'{math.degrees(errors[2]):.3f}°]  max={math.degrees(max_err):.3f}°')

    # ------------------------------------------------------------------ #
    #  Phase 2: DNN trajectory execution                                  #
    # ------------------------------------------------------------------ #
    def trajectory_callback(self):
        if self.current_idx >= self.n_points:
            self.get_logger().info('=' * 80)
            self.get_logger().info('TRAJECTORY EXECUTION COMPLETED')
            self.get_logger().info('=' * 80)
            if self.trajectory_timer:
                self.trajectory_timer.cancel()
            self.trajectory_active = False
            return

        try:
            (tau_dnn, tau_delan, tau_fb, tau_total,
             q_des, qd_des, qdd_des, e_pos, e_vel, gru_active) = \
                self.compute_torques(self.current_idx)
        except Exception as e:
            self.get_logger().error(f'Torque error at idx {self.current_idx}: {e}')
            tau_total = tau_dnn = tau_delan = tau_fb = np.zeros(3)
            idx = self.current_idx
            q_des   = np.array([self.dp1[idx], self.dp2[idx], self.dp3[idx]])
            qd_des  = np.array([self.dv1[idx], self.dv2[idx], self.dv3[idx]])
            qdd_des = np.array([self.da1[idx], self.da2[idx], self.da3[idx]])
            e_pos   = q_des - self.current_joint_pos
            e_vel   = qd_des - self.current_joint_vel
            gru_active = False

        self.msg1.data = [tau_total[0]]
        self.msg2.data = [tau_total[1]]
        self.msg3.data = [tau_total[2]]
        self.pub1.publish(self.msg1)
        self.pub2.publish(self.msg2)
        self.pub3.publish(self.msg3)

        self.current_path_points.append(self._fk_tip_world(*self.current_joint_pos))
        self.publish_current_path_marker()

        if self.log_writer:
            self.log_timestep(
                self.time_data[self.current_idx],
                q_des, qd_des, qdd_des,
                self.current_joint_pos, self.current_joint_vel,
                tau_delan, tau_dnn, tau_fb, tau_total, self.current_joint_efforts,
                e_pos, e_vel, gru_active)

        self.current_idx += 1

        if self.current_idx % 50 == 0 and self.current_idx < self.n_points:
            max_err_deg = math.degrees(np.max(np.abs(e_pos)))
            if self.use_model and self.use_gru:
                gru_label = 'GRU+DeLaN' if gru_active else f'DeLaN-only (warmup {self.current_idx}/{self.dnn.SEQ_LEN})'
            elif self.use_model:
                gru_label = 'DeLaN only'
            else:
                gru_label = 'PID only'
            self.get_logger().info(
                f't={self.time_data[self.current_idx]:.1f}s | '
                f'idx={self.current_idx}/{self.n_points} | '
                f'τ=[{tau_total[0]:.1f},{tau_total[1]:.1f},{tau_total[2]:.1f}] Nm | '
                f'err={max_err_deg:.2f}° | {gru_label}')

    # ------------------------------------------------------------------ #
    #  Main run sequence                                                   #
    # ------------------------------------------------------------------ #
    def run(self):
        self.get_logger().info('Waiting for joint states...')
        while not self.joint_states_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.joint_states_received:
            self.get_logger().error('No joint states received!')
            return

        self.get_logger().info(
            f'Current position: {np.degrees(self.current_joint_pos).tolist()} deg')

        target      = np.array([self.dp1[0], self.dp2[0], self.dp3[0]])
        init_err    = np.max(np.abs(target - self.current_joint_pos))
        skip_stab   = init_err <= self.skip_stabilization_threshold_rad

        if skip_stab:
            self.get_logger().info('PHASE 1: SKIPPED (already near trajectory start)')
        else:
            self.get_logger().info('=' * 80)
            self.get_logger().info('PHASE 1: STABILISATION')
            self.get_logger().info('=' * 80)
            self.stabilization_complete   = False
            self.stabilization_iterations = 0
            self.integral_error           = np.zeros(3)
            self.stabilization_timer = self.create_timer(0.01, self.stabilization_callback)
            while not self.stabilization_complete and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.001)
            if not rclpy.ok():
                return

            self.get_logger().info('Holding for 1 second...')
            hold_iters = [0]
            def hold_cb():
                hold_iters[0] += 1
                if hold_iters[0] >= 100:
                    hold_t.cancel(); return
                self.stabilization_callback()
            hold_t = self.create_timer(0.01, hold_cb)
            while hold_iters[0] < 100 and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.001)

        # Logger + log file
        self.get_logger().info('=' * 80)
        if not self.launch_logger():
            self.get_logger().warning('Logger unavailable, continuing without topic logging')
        if not self.open_log_file():
            self.get_logger().warning('Could not open log file')

        self.publish_expected_path_marker()

        self.get_logger().info('=' * 80)
        if not self.use_model:
            phase2_label = 'PHASE 2: TRAJECTORY EXECUTION (PID feedback)'
        elif self.use_gru:
            phase2_label = 'PHASE 2: DNN TRAJECTORY EXECUTION (online inference)'
        else:
            phase2_label = 'PHASE 2: DeLaN TRAJECTORY EXECUTION'
        self.get_logger().info(phase2_label)
        self.get_logger().info('=' * 80)
        if self.use_model and self.use_gru:
            self.get_logger().info(
                f'{self.n_points} steps | dt={self.dt*1000:.1f}ms | '
                f'GRU active after step {self.dnn.SEQ_LEN}')
        elif self.use_model:
            self.get_logger().info(
                f'{self.n_points} steps | dt={self.dt*1000:.1f}ms | '
                f'DeLaN-only control')
        else:
            self.get_logger().info(
                f'{self.n_points} steps | dt={self.dt*1000:.1f}ms')

        time.sleep(0.1)
        if self.logger_start_client:
            self.start_logger()
        time.sleep(0.01)

        self.current_idx         = 0
        self.traj_integral_error = np.zeros(3)
        self.current_path_points = []
        self.trajectory_active   = True
        self.trajectory_timer    = self.create_timer(self.dt, self.trajectory_callback)

        while self.trajectory_active and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)

        time.sleep(0.1)
        if self.logger_stop_client:
            self.stop_logger()
        self.close_log_file()
        time.sleep(0.1)
        self.get_logger().info(f'Done. Log: {self.log_path}')


# ======================================================================== #
#  Entry point                                                               #
# ======================================================================== #
def main(args=None):
    parser = argparse.ArgumentParser(
        description='DNN Torque Publisher — online DeLaN + GRU inference per timestep')
    parser.add_argument(
        '--csv-path', type=str, required=True,
        help='Trajectory CSV (same format as CTC: t,dp1,dp2,dp3,dv1..da3)')
    parser.add_argument(
        '--delan-model', type=str,
        default=os.path.join(DNN_TEST_DIR, 'fyp_jax_delan_50.jax'),
        help='Path to DeLaN .jax model file')
    parser.add_argument(
        '--gru-model', type=str,
        default=os.path.join(DNN_TEST_DIR, 'best_GRUResidual.pt'),
        help='Path to GRU .pt model file')
    parser.add_argument(
        '--scaler', type=str,
        default=os.path.join(DNN_TEST_DIR, 'feature_scaler.pkl'),
        help='Path to feature scaler .pkl file')
    parser.add_argument(
        '--mode', type=str,
        choices=['pid-only', 'delan-only', 'dnn', 'pid-delan', 'pid-dnn'],
        default=None,
        help='Control test mode: pid-only, delan-only, dnn, pid-delan, or pid-dnn')
    parser.add_argument('--kp',  type=float, nargs=3, default=None,
                        help='Feedback Kp [j1 j2 j3] (default: 5 20 10)')
    parser.add_argument('--kd',  type=float, nargs=3, default=None,
                        help='Feedback Kd [j1 j2 j3] (default: 1 3 2)')
    parser.add_argument('--ki',  type=float, nargs=3, default=None,
                        help='Feedback Ki [j1 j2 j3] (default: 0.05 0.2 0.1)')
    parser.add_argument('--torque-limits', type=float, nargs=3, default=None,
                        help='Torque limits Nm (default: 100 100 60)')
    parser.add_argument('--vel-filter-alpha', type=float, default=0.25,
                        help='Velocity low-pass alpha (default: 0.25)')
    parser.add_argument('--skip-stabilization-threshold-deg', type=float, default=1.0,
                        help='Skip Phase 1 if start error < this deg (default: 1.0)')
    parser.add_argument('--no-feedback', action='store_true',
                        help='Use DNN feedforward only (no PD+I correction)')
    parser.add_argument('--no-model', action='store_true',
                        help='Use PID feedback only (no DNN model)')
    parser.add_argument('--log-path', type=str, default=None,
                        help='Override auto-generated log path')
    parsed = parser.parse_args()

    mode_map = {
        'pid-only':   (True,  False, False),
        'delan-only': (False, True,  False),
        'dnn':        (False, True,  True),
        'pid-delan':  (True,  True,  False),
        'pid-dnn':    (True,  True,  True),
    }
    if parsed.mode is not None:
        use_feedback, use_model, use_gru = mode_map[parsed.mode]
    else:
        use_feedback = not parsed.no_feedback
        use_model = not parsed.no_model
        use_gru = use_model

    rclpy.init(args=args)
    node = DNNTorquePublisher(
        csv_path=parsed.csv_path,
        delan_path=parsed.delan_model,
        gru_path=parsed.gru_model,
        scaler_path=parsed.scaler,
        skip_stabilization_threshold_deg=parsed.skip_stabilization_threshold_deg,
        kp=parsed.kp, kd=parsed.kd, ki=parsed.ki,
        use_feedback=use_feedback,
        use_model=use_model,
        use_gru=use_gru,
        torque_limits=parsed.torque_limits,
        vel_filter_alpha=parsed.vel_filter_alpha,
        log_path=parsed.log_path,
    )
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.shutdown_logger()
        node.close_log_file()
        zero = Float64MultiArray(); zero.data = [0.0]
        node.pub1.publish(zero)
        node.pub2.publish(zero)
        node.pub3.publish(zero)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
