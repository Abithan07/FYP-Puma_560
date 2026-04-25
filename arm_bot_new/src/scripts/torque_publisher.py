#!/usr/bin/env python3
"""
torque_publisher.py
Replays DNN-predicted torques into Gazebo at 100 Hz.

Usage:
  python3 torque_publisher.py <path_id>
  python3 torque_publisher.py <path_id> --log-dir ~/my_logs
  python3 torque_publisher.py <path_id> --data-dir /custom/data/dir

Arguments:
  path_id     Integer trajectory ID (e.g. 461).  The script loads
              <data_dir>/path_<path_id>_joint_states.csv automatically.

  --data-dir  Override the default DNN_test/Data directory.
              Default: ../../DNN_test/Data  relative to arm_bot_new/

  --log-dir   Directory for trajectory log CSV files.
              Default: ~/puma560_logs

CSV format (row-wise, produced by DNN inference scripts):
  t       — time stamps (s)
  dp1..3  — desired joint positions (rad)
  tau1..3 — torque commands (N·m)

Execution phases:
  1. STABILISATION — PID + gravity feedforward drives joints to the
                     trajectory start position [0°, 45°, 135°].
  2. HOLD          — Pure PD holds the start position for 0.5 s.
  3. TRAJECTORY    — Open-loop torque replay at 100 Hz.
"""

import argparse
import csv
import math
import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import Trigger

# ── Locate default data directory ────────────────────────────────────────────
# arm_bot_new/src/scripts/ → up 3 → FYP-Puma_560/ → DNN_test/Data
_SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_DATA = os.path.normpath(
    os.path.join(_SCRIPT_DIR, '..', '..', '..', 'DNN_test', 'Data')
)

# ── DH-consistent forward kinematics ────────────────────────────────────────
# Matches MATLAB: alpha=[0,-pi/2,0], a=[0,0,0.4318], d=[0,0.2435,-0.0934]
# Used only for end-effector tip logging (not for control).

def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return [[1,0,0],[0,c,-s],[0,s,c]]

def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return [[c,-s,0],[s,c,0],[0,0,1]]

def _mm(A, B):
    return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]

def _mv(A, v):
    return [sum(A[i][k]*v[k] for k in range(3)) for i in range(3)]

def _cmp(R, p, Rl, pl):
    rp = _mv(R, pl)
    return _mm(R, Rl), [p[i]+rp[i] for i in range(3)]

def fk_tip(q1, q2, q3):
    R = [[1,0,0],[0,1,0],[0,0,1]]; p = [0.0]*3
    R, p = _cmp(R, p, _rz(0),  [0.5,  0.5,    0.0   ])  # base_joint
    R, p = _cmp(R, p, _rz(0),  [0.0,  0.0,    0.1   ])  # joint_1_shoulder (fixed)
    R, p = _cmp(R, p, _rz(0),  [0.0,  0.0,    0.6718])  # joint_1 origin
    R, p = _cmp(R, p, _rz(q1), [0.0,  0.0,    0.0   ])  # joint_1 rotation
    R, p = _cmp(R, p, _rx(-math.pi/2), [0.0, 0.2435, 0.0])  # joint_2 origin (d2)
    R, p = _cmp(R, p, _rz(q2), [0.0,  0.0,    0.0   ])  # joint_2 rotation
    R, p = _cmp(R, p, _rz(0),  [0.4318, 0.0, -0.0934])  # joint_3 origin (a3, d3)
    R, p = _cmp(R, p, _rz(q3), [0.0,  0.0,    0.0   ])  # joint_3 rotation
    tip  = _mv(R, [0.0, -0.32, 0.0])
    return [p[i]+tip[i] for i in range(3)]


class TorquePublisher(Node):

    def __init__(self, csv_path: str, log_dir: str):
        super().__init__('torque_publisher')

        # ── Publishers ──────────────────────────────────────────────────
        self.pub1 = self.create_publisher(Float64MultiArray,
                                          '/joint_1_controller/commands', 10)
        self.pub2 = self.create_publisher(Float64MultiArray,
                                          '/joint_2_controller/commands', 10)
        self.pub3 = self.create_publisher(Float64MultiArray,
                                          '/joint_3_controller/commands', 10)
        rec_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.rec_pub = self.create_publisher(Bool, '/logger/recording_active', rec_qos)
        self._set_rec(False)

        # ── Subscriber ──────────────────────────────────────────────────
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        # ── State ───────────────────────────────────────────────────────
        self.pos       = [0.0, 0.0, 0.0]
        self.vel       = [0.0, 0.0, 0.0]
        self.js_ready  = False
        self.integral  = [0.0, 0.0, 0.0]
        self.stab_done = False
        self.stab_iter = 0
        self.traj_idx  = 0
        self.traj_done = False
        self.log_dir   = os.path.expanduser(log_dir)

        self._logger_proc    = None
        self._start_client   = None
        self._stop_client    = None
        self._logger_stopped = True   # becomes False once logger is running

        # Pre-allocated messages
        self.m1 = Float64MultiArray()
        self.m2 = Float64MultiArray()
        self.m3 = Float64MultiArray()

        # ── Load CSV ────────────────────────────────────────────────────
        self._load(csv_path)
        self.csv_path = csv_path

        self.get_logger().info('='*60)
        self.get_logger().info('PUMA 560 Torque Publisher — Ideal Model')
        self.get_logger().info(f'  CSV            : {csv_path}')
        self.get_logger().info(f'  Points         : {len(self.t)}')
        self.get_logger().info(f'  Duration       : {self.t[-1]:.2f} s')
        self.get_logger().info(f'  dt             : {self.dt*1000:.1f} ms')
        self.get_logger().info(
            f'  Start position : [{self.dp1[0]:.4f}, {self.dp2[0]:.4f}, '
            f'{self.dp3[0]:.4f}] rad')
        self.get_logger().info('='*60)

    # ── CSV ──────────────────────────────────────────────────────────────
    def _load(self, path: str):
        if not os.path.isfile(path):
            raise FileNotFoundError(
                f'CSV not found: {path}\n'
                f'Run: python3 inf7.py <path_id>  from DNN_test/ first.')

        data = {}
        with open(path) as f:
            for row in csv.reader(f):
                if row:
                    data[row[0].strip()] = [float(v) for v in row[1:]]

        missing = [k for k in ('t','dp1','dp2','dp3','tau1','tau2','tau3')
                   if k not in data]
        if missing:
            raise ValueError(f'CSV is missing rows: {missing}')

        self.t    = data['t']
        self.dp1  = data['dp1']
        self.dp2  = data['dp2']
        self.dp3  = data['dp3']
        self.tau1 = data['tau1']
        self.tau2 = data['tau2']
        self.tau3 = data['tau3']
        self.dt   = self.t[1] - self.t[0] if len(self.t) > 1 else 0.01

    # ── Helpers ───────────────────────────────────────────────────────────
    def _pub(self, t1, t2, t3):
        self.m1.data = [t1]; self.pub1.publish(self.m1)
        self.m2.data = [t2]; self.pub2.publish(self.m2)
        self.m3.data = [t3]; self.pub3.publish(self.m3)

    def _set_rec(self, active: bool):
        m = Bool(); m.data = bool(active); self.rec_pub.publish(m)

    def _grav(self):
        q2, q3 = self.pos[1], self.pos[2]
        return [0.0,
                -44.0 * math.cos(q2),
                -12.0 * math.cos(q2 + q3)]

    # ── Joint state callback ──────────────────────────────────────────────
    def _js_cb(self, msg):
        try:
            i1 = msg.name.index('joint_1')
            i2 = msg.name.index('joint_2')
            i3 = msg.name.index('joint_3')
            self.pos = [msg.position[i1], msg.position[i2], msg.position[i3]]
            if len(msg.velocity) > max(i1, i2, i3):
                self.vel = [msg.velocity[i1], msg.velocity[i2], msg.velocity[i3]]
            self.js_ready = True
        except (ValueError, IndexError):
            pass

    # ── Phase 1: PID stabilisation ────────────────────────────────────────
    def _stab_cb(self):
        target = [self.dp1[0], self.dp2[0], self.dp3[0]]
        err    = [target[i] - self.pos[i] for i in range(3)]
        max_e  = max(abs(e) for e in err)

        self.stab_iter += 1

        if max_e < 3.5e-6:
            self.get_logger().info(
                f'Stabilised after {self.stab_iter/100:.1f}s. '
                f'Max error: {max_e*180/math.pi:.6f}°')
            self._stab_timer.cancel()
            self.stab_done = True
            return

        if self.stab_iter >= 12000:
            self.get_logger().error(
                f'Stabilisation timeout. Error: {max_e*180/math.pi:.4f}°')
            self._stab_timer.cancel()
            self.stab_done = True
            return

        kp  = [50.0, 200.0, 150.0]
        ki  = [5.0,   25.0,  20.0]
        kd  = [12.0,  35.0,  10.0]
        lim = [100.0, 100.0,  50.0]
        dt  = 0.01

        for i in range(3):
            self.integral[i] = max(-1.0, min(1.0,
                                  self.integral[i] + err[i] * dt))

        g   = self._grav()
        tau = [max(-lim[i], min(lim[i],
               kp[i]*err[i] + ki[i]*self.integral[i]
               - kd[i]*self.vel[i] + g[i]))
               for i in range(3)]

        self._pub(*tau)

        if self.stab_iter % 100 == 0:
            self.get_logger().info(
                f't={self.stab_iter/100:.1f}s | '
                f'err=[{err[0]*180/math.pi:.3f}°, '
                f'{err[1]*180/math.pi:.3f}°, '
                f'{err[2]*180/math.pi:.3f}°]')

    # ── Phase 2: PD hold at start position ───────────────────────────────
    # Keeps position at trajectory start while the logger is being launched.
    def _hold_cb(self):
        target = [self.dp1[0], self.dp2[0], self.dp3[0]]
        err    = [target[i] - self.pos[i] for i in range(3)]
        kp     = [50.0, 200.0, 150.0]
        kd     = [12.0,  35.0,  10.0]
        lim    = [100.0, 100.0,  50.0]
        g      = self._grav()
        tau    = [max(-lim[i], min(lim[i],
                  kp[i]*err[i] - kd[i]*self.vel[i] + g[i]))
                  for i in range(3)]
        self._pub(*tau)
        self._hold_count += 1
        if self._hold_count >= 50:
            self._hold_timer.cancel()

    # ── Phase 3: Open-loop torque replay ────────────────────────────────
    def _traj_cb(self):
        i = self.traj_idx
        if i >= len(self.t):
            self.get_logger().info('Trajectory complete.')
            self._traj_timer.cancel()
            self.traj_done = True
            return

        self._pub(self.tau1[i], self.tau2[i], self.tau3[i])
        self.traj_idx += 1

        if self.traj_idx % 100 == 0 and self.traj_idx < len(self.t):
            j = self.traj_idx
            err = max(abs(self.dp1[j]-self.pos[0]),
                      abs(self.dp2[j]-self.pos[1]),
                      abs(self.dp3[j]-self.pos[2]))
            self.get_logger().info(
                f't={self.t[j]:.1f}s | '
                f'tau=[{self.tau1[i]:.2f}, {self.tau2[i]:.2f}, '
                f'{self.tau3[i]:.2f}] Nm | '
                f'max_err={err*180/math.pi:.2f}°')

    # ── Logger management ─────────────────────────────────────────────────
    def _launch_logger(self) -> bool:
        script = os.path.join(_SCRIPT_DIR, 'triggered_logger.py')
        if not os.path.isfile(script):
            self.get_logger().warning(f'Logger script not found: {script}')
            return False

        os.makedirs(self.log_dir, exist_ok=True)
        self._logger_proc = subprocess.Popen([
            'python3', script,
            '--dataset-path', self.csv_path,
            '--log-dir', self.log_dir,
        ])
        time.sleep(0.2)

        self._start_client = self.create_client(Trigger, '/logger/start')
        self._stop_client  = self.create_client(Trigger, '/logger/stop')

        deadline = time.time() + 5.0
        while not self._start_client.wait_for_service(timeout_sec=0.1):
            if time.time() > deadline:
                self.get_logger().error('Logger service not available.')
                return False
            rclpy.spin_once(self, timeout_sec=0.01)

        self.get_logger().info('Logger subprocess ready.')
        return True

    def _call_svc(self, client) -> bool:
        if client is None:
            return False
        fut      = client.call_async(Trigger.Request())
        deadline = time.time() + 2.0
        while not fut.done():
            if time.time() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.01)
        return fut.result().success

    def _stop_logger(self):
        if self._logger_stopped:
            return
        self._logger_stopped = True
        self._set_rec(False)
        self._call_svc(self._stop_client)
        if self._logger_proc and self._logger_proc.poll() is None:
            self._logger_proc.terminate()
            try:
                self._logger_proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._logger_proc.kill()

    # ── Main sequence ─────────────────────────────────────────────────────
    def run(self):
        # Wait for first /joint_states message
        self.get_logger().info('Waiting for /joint_states ...')
        while not self.js_ready and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        # ── Phase 1: Stabilisation ───────────────────────────────────────
        self.get_logger().info('='*60)
        self.get_logger().info('PHASE 1: Stabilising to start position')
        self.get_logger().info('='*60)
        self._stab_timer = self.create_timer(0.01, self._stab_cb)
        while not self.stab_done and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)

        # ── Phase 2: Hold while logger starts ───────────────────────────
        self.get_logger().info('Holding start position for 0.5 s ...')
        self._hold_count = 0
        self._hold_timer = self.create_timer(0.01, self._hold_cb)
        while self._hold_count < 50 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)

        # ── Launch logger ────────────────────────────────────────────────
        logger_ok = self._launch_logger()
        time.sleep(0.05)
        if logger_ok:
            if self._call_svc(self._start_client):
                self._logger_stopped = False
                self._set_rec(True)
                self.get_logger().info('Logger recording started.')
            else:
                self.get_logger().warning('Logger start service call failed.')

        # ── Phase 3: Trajectory replay ───────────────────────────────────
        self.get_logger().info('='*60)
        self.get_logger().info(
            f'PHASE 3: Open-loop torque replay ({len(self.t)} steps @ 100 Hz)')
        self.get_logger().info('='*60)
        self.traj_idx  = 0
        self.traj_done = False
        self._traj_timer = self.create_timer(0.01, self._traj_cb)
        while not self.traj_done and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)

        time.sleep(0.1)  # let logger capture last samples
        self._stop_logger()


def main():
    parser = argparse.ArgumentParser(
        description='PUMA 560 ideal-model torque publisher',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            'Examples:\n'
            '  python3 torque_publisher.py 461\n'
            '  python3 torque_publisher.py 601 --log-dir ~/my_logs\n'
            '  python3 torque_publisher.py 461 --data-dir /custom/Data\n'
        ),
    )
    parser.add_argument(
        'path_id', type=int,
        help='Trajectory ID (e.g. 461).  CSV loaded from <data_dir>/path_NNN_joint_states.csv',
    )
    parser.add_argument(
        '--data-dir', default=None,
        help=f'Directory containing path_NNN_joint_states.csv files.\n'
             f'Default: {_DEFAULT_DATA}',
    )
    parser.add_argument(
        '--log-dir', default='~/puma560_logs',
        help='Directory for logger output CSV files. Default: ~/puma560_logs',
    )
    args = parser.parse_args()

    data_dir = os.path.expanduser(args.data_dir) if args.data_dir else _DEFAULT_DATA
    csv_path = os.path.join(data_dir, f'path_{args.path_id:03d}_joint_states.csv')

    rclpy.init()
    node = TorquePublisher(csv_path=csv_path, log_dir=args.log_dir)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node._stop_logger()
        zero = Float64MultiArray(); zero.data = [0.0]
        node.pub1.publish(zero)
        node.pub2.publish(zero)
        node.pub3.publish(zero)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
