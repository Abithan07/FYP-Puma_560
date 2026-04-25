#!/usr/bin/env python3
"""
Inverse Dynamics Pipeline – Step 1: Trajectory Player
======================================================
Reads a trajectory CSV (row-major: t, q1, q2, q3, dq1, dq2, dq3),
drives the PUMA-560 3-DOF arm in Gazebo using a PD+gravity-compensation
effort controller so the robot closely follows the commanded path,
then records the commanded joint torques as the inverse-dynamics labels.

Why record tau_cmd rather than the "raw" effort state?
  In gz_ros2_control/ForwardCommandController the effort *state* interface
  simply echoes the last written command; there is no independent torque
  sensor.  Under tight tracking (small position and velocity error) the
  commanded effort equals the true inverse dynamics torque:
      tau_cmd ≈ M(q)*ddq + C(q,dq)*dq + G(q)   [Newton-Euler]
  The PD correction (Kp*e + Kd*de) accounts for whatever small mismatch
  remains, but since tracking error is tiny these terms are small.
  The downstream build_dataset script can subtract them for an even
  cleaner estimate.

Output (intermediate raw CSV, column-major):
  Columns: t | q1_ref q2_ref q3_ref | dq1_ref … | q1_act … | dq1_act …
           | tau1_cmd tau2_cmd tau3_cmd | tau1_act tau2_act tau3_act
           | e1 e2 e3 (position errors, rad)

Requirements:
  - arm_bot package sourced (provides joint_1/2/3_controller effort topics)
  - Controller update_rate MUST match the CSV sample rate:
       my_controllers.yaml → controller_manager → update_rate: 1000
    (or 100 if your CSV dt = 0.01 s)
  - Gazebo running with use_sim_time:=true

Usage:
  python3 1_trajectory_player.py \\
      --input  /path/to/input.csv \\
      --raw    /path/to/raw_output.csv \\
      [--ctrl-rate 1000]
"""

import argparse
import csv
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# ─── Physical constants ────────────────────────────────────────────────────────

# Gravity compensation (empirically tuned for this URDF – see inertial_macros.xacro)
# tau_grav = [0,  G2*cos(q2),  G3*cos(q2+q3)]
_G_COEFF = [0.0, -44.0, -12.0]

# PD gains for TRAJECTORY TRACKING PHASE (no integral → no wind-up artefacts)
# High proportional gain → tight tracking → tau_cmd ≈ tau_ID
_KP = [300.0, 600.0, 400.0]
_KD = [25.0,  50.0,  30.0]

# PID gains for STABILISATION PHASE (full PID to reach initial pose reliably)
_KP_STAB = [80.0,  250.0, 180.0]
_KI_STAB = [5.0,    20.0,  15.0]
_KD_STAB = [15.0,   40.0,  25.0]

# Torque saturation (Nm)
_TAU_MAX = [200.0, 200.0, 100.0]

# Stabilisation convergence threshold (rad) – 0.001 rad ≈ 0.057°
_STAB_TOL = 0.001

# Maximum stabilisation time (s)
_STAB_TIMEOUT = 30.0

# Hold time at initial pose before trajectory starts (s)
_HOLD_TIME = 0.5


# ─── CSV helpers ──────────────────────────────────────────────────────────────

def load_input_csv(path: str) -> dict:
    """
    Load row-major CSV.  First token per row is the label; remaining tokens
    are floats.  Accepts both (q1/q2/q3, dq1/dq2/dq3) and legacy
    (dp1/dp2/dp3, dv1/dv2/dv3) naming.
    """
    data: dict[str, list[float]] = {}
    with open(path, 'r') as f:
        for row in csv.reader(f):
            if len(row) < 2:
                continue
            key = row[0].strip()
            try:
                data[key] = [float(v) for v in row[1:]]
            except ValueError:
                pass  # skip header / empty rows

    # Normalise key names to q1/q2/q3, dq1/dq2/dq3
    for old, new in [('dp1', 'q1'), ('dp2', 'q2'), ('dp3', 'q3'),
                     ('dv1', 'dq1'), ('dv2', 'dq2'), ('dv3', 'dq3')]:
        if old in data and new not in data:
            data[new] = data.pop(old)

    required = ['t', 'q1', 'q2', 'q3', 'dq1', 'dq2', 'dq3']
    missing = [k for k in required if k not in data]
    if missing:
        raise KeyError(f'Input CSV is missing rows: {missing}')

    # Verify all rows have equal length
    n = len(data['t'])
    for k in required:
        if len(data[k]) != n:
            raise ValueError(f'Row "{k}" has {len(data[k])} values, expected {n}')

    return data


def save_raw_csv(path: str, records: list[dict]) -> None:
    """Save list-of-dicts as a column-major CSV (one column per time step)."""
    if not records:
        raise RuntimeError('No records to save.')
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fields = list(records[0].keys())
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)


# ─── Node ─────────────────────────────────────────────────────────────────────

class IDTrajectoryPlayer(Node):
    """
    Event-driven trajectory player.  Each /joint_states message triggers one
    control step so the node is synchronised to the physics clock automatically.
    """

    JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3']

    # ── init ──────────────────────────────────────────────────────────────────

    def __init__(self, input_csv: str, raw_output: str, ctrl_rate: int):
        super().__init__('id_trajectory_player')
        self.raw_output = raw_output

        # Publishers for effort commands (depth=1 → always send the latest)
        self._pub = [
            self.create_publisher(Float64MultiArray,
                                  f'/joint_{i+1}_controller/commands', 1)
            for i in range(3)
        ]
        self._msgs = [Float64MultiArray() for _ in range(3)]

        # Subscriber (depth=1 → process every new measurement, drop old ones)
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(JointState, '/joint_states',
                                 self._js_callback, qos)

        # Current joint state
        self._q = [0.0, 0.0, 0.0]
        self._dq = [0.0, 0.0, 0.0]
        self._tau_state = [0.0, 0.0, 0.0]
        self._js_ready = False

        # Load reference trajectory
        traj = load_input_csv(input_csv)
        self._t_ref  = traj['t']
        self._q_ref  = [traj['q1'], traj['q2'], traj['q3']]
        self._dq_ref = [traj['dq1'], traj['dq2'], traj['dq3']]
        self._N      = len(self._t_ref)
        self._csv_dt = (self._t_ref[1] - self._t_ref[0]) if self._N > 1 else 0.001

        self._ctrl_rate = ctrl_rate
        self._ctrl_dt   = 1.0 / ctrl_rate

        # Phase management
        # 'wait' → 'stabilise' → 'hold' → 'execute' → 'done'
        self._phase = 'wait'
        self._stab_iter   = 0
        self._hold_iter   = 0
        self._exec_idx    = 0
        self._intg_err    = [0.0, 0.0, 0.0]
        self._records: list[dict] = []

        self.get_logger().info(
            f'Loaded {self._N} waypoints  dt={self._csv_dt*1000:.2f} ms  '
            f'ctrl={ctrl_rate} Hz'
        )

    # ── control helpers ───────────────────────────────────────────────────────

    @staticmethod
    def _gravity_comp(q: list) -> list:
        """G(q) compensation torques for PUMA 560 3-DOF."""
        return [
            0.0,
            _G_COEFF[1] * math.cos(q[1]),
            _G_COEFF[2] * math.cos(q[1] + q[2]),
        ]

    @staticmethod
    def _clamp(val: float, limit: float) -> float:
        return max(-limit, min(limit, val))

    def _pd_control(self, q_ref: list, dq_ref: list,
                    q: list, dq: list) -> list:
        """PD + gravity compensation – the core tracking controller.

        tau_cmd = Kp*(q_ref - q) + Kd*(dq_ref - dq) + G(q)

        Under near-perfect tracking (e ≈ 0, de ≈ 0) the physics engine
        must supply tau_cmd to sustain ddq ≈ ddq_ref, so:
            tau_cmd ≈ M(q)*ddq_ref + C(q,dq)*dq_ref + G(q) = tau_ID
        """
        G = self._gravity_comp(q)
        tau = [
            _KP[i] * (q_ref[i] - q[i]) + _KD[i] * (dq_ref[i] - dq[i]) + G[i]
            for i in range(3)
        ]
        return [self._clamp(tau[i], _TAU_MAX[i]) for i in range(3)]

    def _pid_stabilise(self, q_target: list) -> tuple[list, float]:
        """Full PID + gravity comp used only during the stabilisation phase."""
        errs = [q_target[i] - self._q[i] for i in range(3)]
        for i in range(3):
            self._intg_err[i] += errs[i] * self._ctrl_dt
            self._intg_err[i] = self._clamp(self._intg_err[i], 2.0)
        G = self._gravity_comp(self._q)
        tau = [
            _KP_STAB[i] * errs[i]
            + _KI_STAB[i] * self._intg_err[i]
            - _KD_STAB[i] * self._dq[i]
            + G[i]
            for i in range(3)
        ]
        tau = [self._clamp(tau[i], _TAU_MAX[i]) for i in range(3)]
        return tau, max(abs(e) for e in errs)

    # ── publish helpers ───────────────────────────────────────────────────────

    def _publish(self, tau: list) -> None:
        for i in range(3):
            self._msgs[i].data = [float(tau[i])]
            self._pub[i].publish(self._msgs[i])

    def _zero_torques(self) -> None:
        self._publish([0.0, 0.0, 0.0])

    # ── joint state callback ─────────────────────────────────────────────────

    def _js_callback(self, msg: JointState) -> None:
        # Update state
        try:
            for i, name in enumerate(self.JOINT_NAMES):
                idx = msg.name.index(name)
                self._q[i]         = msg.position[idx]
                self._dq[i]        = msg.velocity[idx] if msg.velocity else 0.0
                self._tau_state[i] = msg.effort[idx]  if msg.effort  else 0.0
        except (ValueError, IndexError):
            return
        self._js_ready = True

        # Dispatch to the active phase
        if   self._phase == 'stabilise': self._step_stabilise()
        elif self._phase == 'hold':      self._step_hold()
        elif self._phase == 'execute':   self._step_execute()

    # ── phase: stabilise ─────────────────────────────────────────────────────

    def _step_stabilise(self) -> None:
        q0 = [self._q_ref[j][0] for j in range(3)]
        tau, max_err = self._pid_stabilise(q0)
        self._publish(tau)
        self._stab_iter += 1

        if max_err < _STAB_TOL:
            self.get_logger().info(
                f'[Stabilise] Converged  err={math.degrees(max_err):.4f}°'
            )
            self._start_hold()
        elif self._stab_iter * self._ctrl_dt > _STAB_TIMEOUT:
            self.get_logger().warn(
                f'[Stabilise] Timeout  err={math.degrees(max_err):.4f}°  '
                f'continuing anyway'
            )
            self._start_hold()
        elif self._stab_iter % max(1, int(2.0 / self._ctrl_dt)) == 0:
            self.get_logger().info(
                f'[Stabilise] t={self._stab_iter*self._ctrl_dt:.1f}s  '
                f'err={math.degrees(max_err):.4f}°'
            )

    def _start_hold(self) -> None:
        self._hold_iter = 0
        self._intg_err  = [0.0, 0.0, 0.0]
        self._phase     = 'hold'
        self.get_logger().info(f'[Hold] Holding for {_HOLD_TIME:.1f} s …')

    # ── phase: hold ───────────────────────────────────────────────────────────

    def _step_hold(self) -> None:
        q0   = [self._q_ref[j][0]  for j in range(3)]
        dq0  = [self._dq_ref[j][0] for j in range(3)]
        tau  = self._pd_control(q0, dq0, self._q, self._dq)
        self._publish(tau)
        self._hold_iter += 1
        if self._hold_iter >= int(_HOLD_TIME / self._ctrl_dt):
            self.get_logger().info('[Execute] Starting trajectory …')
            self._exec_idx = 0
            self._phase    = 'execute'

    # ── phase: execute ────────────────────────────────────────────────────────

    def _step_execute(self) -> None:
        if self._exec_idx >= self._N:
            self.get_logger().info('[Execute] Trajectory complete.')
            self._zero_torques()
            self._phase = 'done'
            return

        idx = self._exec_idx
        q_ref  = [self._q_ref[j][idx]  for j in range(3)]
        dq_ref = [self._dq_ref[j][idx] for j in range(3)]

        # ── Core control ──────────────────────────────────────────────────
        # tau_cmd is what the physics engine must supply to achieve this
        # motion → this IS the inverse dynamics torque under tight tracking.
        tau_cmd = self._pd_control(q_ref, dq_ref, self._q, self._dq)
        self._publish(tau_cmd)

        # Position tracking error (rad)
        e = [q_ref[i] - self._q[i] for i in range(3)]

        # ── Record ────────────────────────────────────────────────────────
        # We record both tau_cmd (what we sent → ≈ tau_ID) and tau_act
        # (effort state ← echoes previous command in gz_ros2_control).
        # build_dataset.py uses tau_cmd by default.
        self._records.append({
            't':        self._t_ref[idx],
            'q1_ref':   q_ref[0],  'q2_ref':   q_ref[1],  'q3_ref':   q_ref[2],
            'dq1_ref':  dq_ref[0], 'dq2_ref':  dq_ref[1], 'dq3_ref':  dq_ref[2],
            'q1_act':   self._q[0], 'q2_act':  self._q[1], 'q3_act':  self._q[2],
            'dq1_act':  self._dq[0],'dq2_act': self._dq[1],'dq3_act': self._dq[2],
            'tau1_cmd': tau_cmd[0], 'tau2_cmd':tau_cmd[1], 'tau3_cmd':tau_cmd[2],
            'tau1_act': self._tau_state[0],
            'tau2_act': self._tau_state[1],
            'tau3_act': self._tau_state[2],
            'e1': e[0], 'e2': e[1], 'e3': e[2],
        })

        self._exec_idx += 1

        if self._exec_idx % max(1, self._ctrl_rate) == 0:
            max_e_deg = max(abs(ei) for ei in e) * 180 / math.pi
            self.get_logger().info(
                f'[Execute] {self._exec_idx}/{self._N}  '
                f't={self._t_ref[idx]:.3f} s  '
                f'max_err={max_e_deg:.3f}°  '
                f'τ=[{tau_cmd[0]:.1f}, {tau_cmd[1]:.1f}, {tau_cmd[2]:.1f}] Nm'
            )

    # ── run ───────────────────────────────────────────────────────────────────

    def run(self) -> None:
        self.get_logger().info('Waiting for /joint_states …')
        while not self._js_ready and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        if not rclpy.ok():
            return

        self.get_logger().info(
            '=== Phase 1 of 2: Stabilise to initial pose ==='
        )
        self._phase = 'stabilise'

        while self._phase != 'done' and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=self._ctrl_dt * 0.5)

        self._zero_torques()

        if self._records:
            save_raw_csv(self.raw_output, self._records)
            self.get_logger().info(
                f'Saved {len(self._records)} rows → {self.raw_output}'
            )

            # Quick tracking quality report
            max_errs = [
                max(abs(r[f'e{i+1}']) for r in self._records)
                for i in range(3)
            ]
            self.get_logger().info(
                f'Tracking quality (max |error|): '
                f'j1={math.degrees(max_errs[0]):.4f}°  '
                f'j2={math.degrees(max_errs[1]):.4f}°  '
                f'j3={math.degrees(max_errs[2]):.4f}°'
            )
        else:
            self.get_logger().error('No data recorded – trajectory was not executed.')


# ─── Default paths ────────────────────────────────────────────────────────────

_TRAJ_DIR = os.path.expanduser(
    '~/Desktop/FYP-Puma_560/Dataset/Trajectories'
)


def _resolve_paths(args) -> tuple[str, str]:
    """Return (input_csv, raw_csv) resolved from --id or explicit flags."""
    if args.id is not None:
        fid = f'{args.id:03d}'
        input_csv = os.path.join(_TRAJ_DIR, f'path_{fid}_trajectory.csv')
        raw_csv   = f'/tmp/id_raw_{fid}.csv'
    else:
        if args.input is None or args.raw is None:
            raise SystemExit('Provide either --id N or both --input and --raw.')
        input_csv = args.input
        raw_csv   = args.raw
    return input_csv, raw_csv


# ─── Entry point ──────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(
        description='ID Pipeline Step 1 – play trajectory and record torques',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            'Examples:\n'
            '  python3 1_trajectory_player.py --id 1\n'
            '  python3 1_trajectory_player.py --id 3 --ctrl-rate 100\n'
            '  python3 1_trajectory_player.py --input my.csv --raw /tmp/raw.csv\n'
        ),
    )
    ap.add_argument('--id', type=int, default=None,
                    metavar='N',
                    help='Trajectory file ID (e.g. 1 → path_001_trajectory.csv). '
                         'Sets --input and --raw automatically.')
    ap.add_argument('--input', default=None,
                    help='Input CSV path (overrides --id)')
    ap.add_argument('--raw', default=None,
                    help='Raw output CSV path (overrides --id default /tmp/id_raw_NNN.csv)')
    ap.add_argument('--ctrl-rate', type=int, default=1000,
                    help='Controller rate in Hz – MUST match update_rate in '
                         'my_controllers.yaml (default: 1000)')
    args = ap.parse_args()

    input_csv, raw_csv = _resolve_paths(args)
    print(f'Input  : {input_csv}')
    print(f'Raw out: {raw_csv}')

    rclpy.init()
    node = IDTrajectoryPlayer(
        input_csv=input_csv,
        raw_output=raw_csv,
        ctrl_rate=args.ctrl_rate,
    )
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node._zero_torques()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
