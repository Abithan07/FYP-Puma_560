#!/usr/bin/env python3
"""
PUMA-560 PATH + TRAJECTORY DATASET GENERATOR
Generates:
1) Joint Angles        -> {base_dir}/Angles/path_XXX_angles.csv
2) End Effector XYZ   -> {base_dir}/XYZ/path_XXX_xyz.csv
3) Full trajectory     -> {base_dir}/Trajectories/path_XXX_traj.csv

Trajectory CSV format (row-wise):
    t,    0.000, 0.010, 0.020, ...
    dp1,  0.00000000, ...
    dp2,  0.00000000, ...
    dp3,  0.00000000, ...
    dv1,  0.00000000, ...
    dv2,  0.00000000, ...
    dv3,  0.00000000, ...
    da1,  0.00000000, ...
    da2,  0.00000000, ...
    da3,  0.00000000, ...

Constraints:
- |velocity| <= 2 rad/s
- |acceleration| <= 7 rad/s^2

Batch mode (--csv):
    Reads a CSV with columns:
        path_id, q1-end, q2-end, q3-end, q1-mid, q2-mid, q3-mid, t-total, num-paths
    Each row defines one trajectory condition.
    - q1-mid, q2-mid, q3-mid: optional waypoint (leave blank or 0 to skip)
    - t-total: optional duration (leave blank or 0 to auto-compute)
    - num-paths: number of trajectories to generate for this condition
"""

import argparse
import csv
import numpy as np
import os
from typing import Tuple


# ===================== PUMA DH PARAMETERS =====================

ALPHA = np.array([0.0, -np.pi/2, 0.0, np.pi/2])
A_DH  = np.array([0.0,  0.0,     0.4318, 0.0])
D_DH  = np.array([0.0,  0.2435, -0.0934, 0.4331])

T_BASE = np.eye(4)
T_BASE[2, 3] = 0.6718

Tracking_file = './test_traj.csv'
Output_dir    = './test_output'


# ===================== CONFIG =====================

class TrajConfig:
    def __init__(self):
        self.dt         = 0.01
        self.possible_T = np.array([12, 16, 20, 24])
        self.v_max      = 2.0
        self.a_max      = 7.0

        self.q_start_deg = np.array([0.0, 45.0, 135.0])
        self.q_start     = np.deg2rad(self.q_start_deg)

        self.q4_deg = 0.0
        self.q4     = np.deg2rad(self.q4_deg)


# ===================== KINEMATICS =====================

def dh_transform(alpha: float, a: float, d: float, theta: float) -> np.ndarray:
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct,     -st,    0,    a    ],
        [st*ca,   ct*ca, -sa, -sa*d ],
        [st*sa,   ct*sa,  ca,  ca*d ],
        [0,       0,      0,   1    ]
    ], dtype=float)


def forward_kinematics(q: np.ndarray, q4: float) -> np.ndarray:
    q1, q2, q3 = q
    T = (T_BASE
         @ dh_transform(ALPHA[0], A_DH[0], D_DH[0], q1)
         @ dh_transform(ALPHA[1], A_DH[1], D_DH[1], q2)
         @ dh_transform(ALPHA[2], A_DH[2], D_DH[2], q3)
         @ dh_transform(ALPHA[3], A_DH[3], D_DH[3], q4))
    return T[:3, 3]


# ===================== TRAJECTORY PROFILES =====================

def min_jerk_profile(t: np.ndarray, T: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Standard minimum-jerk profile with zero boundary velocities (0 to 1)."""
    tau = t / T
    f   =  10*tau**3 - 15*tau**4 +  6*tau**5
    fd  = (30*tau**2 - 60*tau**3 + 30*tau**4) / T
    fdd = (60*tau   - 180*tau**2 + 120*tau**3) / T**2
    return f, fd, fdd


def min_jerk_profile_bounded(
    t: np.ndarray,
    T: float,
    v0_norm: float = 0.0,
    vf_norm: float = 0.0
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    5th-order polynomial with prescribed boundary velocities.
    Position is normalised 0 to 1. Boundary accelerations are zero.
    """
    T2, T3, T4, T5 = T**2, T**3, T**4, T**5

    A = np.array([
        [1, 0,   0,    0,     0,     0    ],
        [0, 1,   0,    0,     0,     0    ],
        [0, 0,   2,    0,     0,     0    ],
        [1, T,   T2,   T3,    T4,    T5   ],
        [0, 1,   2*T,  3*T2,  4*T3,  5*T4 ],
        [0, 0,   2,    6*T,  12*T2, 20*T3 ],
    ])
    b = np.array([0.0, v0_norm, 0.0, 1.0, vf_norm, 0.0])
    c = np.linalg.solve(A, b)

    f   = c[0] + c[1]*t +  c[2]*t**2 +  c[3]*t**3 +   c[4]*t**4 +   c[5]*t**5
    fd  =        c[1]   + 2*c[2]*t   + 3*c[3]*t**2 + 4*c[4]*t**3 + 5*c[5]*t**4
    fdd =                 2*c[2]     + 6*c[3]*t    +12*c[4]*t**2 +20*c[5]*t**3
    return f, fd, fdd


# ===================== WAYPOINT HELPERS =====================

def compute_junction_velocity(
    q_start: np.ndarray,
    q_mid:   np.ndarray,
    q_end:   np.ndarray,
    T1: float, T2: float,
    v_max: float, a_max: float
) -> np.ndarray:
    """
    Per-joint pass-through velocity at the waypoint.
    Direction reversal on a joint forces that joint to stop.
    Otherwise uses the minimum of what segment 1 can offer and
    segment 2 can accept, clamped by v_max and an acceleration limit.
    """
    dq1 = q_mid - q_start
    dq2 = q_end  - q_mid
    v_junction = np.zeros(len(q_start))

    for j in range(len(q_start)):
        d1, d2 = dq1[j], dq2[j]
        if d1 * d2 < 0:
            continue
        v1_offer    = 1.875 * abs(d1) / T1
        v2_accept   = 1.875 * abs(d2) / T2
        v_acc_limit = np.sqrt(2 * a_max * abs(d1)) * 0.5
        v_junction[j] = np.sign(d1) * min(v1_offer, v2_accept, v_max, v_acc_limit)

    return v_junction


def split_time_by_arc(
    q_start: np.ndarray,
    q_mid:   np.ndarray,
    q_end:   np.ndarray,
    T_total: float
) -> Tuple[float, float]:
    """Split T_total proportionally to joint-space arc length."""
    dist1 = np.linalg.norm(q_mid - q_start)
    dist2 = np.linalg.norm(q_end  - q_mid)
    total = dist1 + dist2

    if total < 1e-12:
        return T_total / 2.0, T_total / 2.0

    T1 = max(T_total * dist1 / total, 0.1 * T_total)
    T2 = max(T_total * dist2 / total, 0.1 * T_total)
    scale = T_total / (T1 + T2)
    return T1 * scale, T2 * scale


# ===================== TWO-SEGMENT TRAJECTORY =====================

def generate_two_segment_trajectory(
    q_start: np.ndarray,
    q_mid:   np.ndarray,
    q_end:   np.ndarray,
    T1: float, T2: float,
    dt: float,
    v_max: float, a_max: float
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Smooth two-segment trajectory: q_start -> q_mid -> q_end.
    The arm passes through q_mid without stopping.
    """
    n      = q_start.shape[0]
    dq1    = q_mid - q_start
    dq2    = q_end  - q_mid
    v_junc = compute_junction_velocity(q_start, q_mid, q_end, T1, T2, v_max, a_max)
    zeros  = np.zeros(n)

    def build_segment(t_seg, q_from, dq, v0, vf):
        N    = len(t_seg)
        q_   = np.zeros((N, n))
        qd_  = np.zeros((N, n))
        qdd_ = np.zeros((N, n))
        for j in range(n):
            q_[:, j] = q_from[j]
            if abs(dq[j]) < 1e-10:
                continue
            f, fd, fdd = min_jerk_profile_bounded(
                t_seg, t_seg[-1],
                v0_norm=v0[j] / dq[j],
                vf_norm=vf[j] / dq[j]
            )
            q_[:,  j] = q_from[j] + dq[j] * f
            qd_[:, j] = dq[j] * fd
            qdd_[:,j]  = dq[j] * fdd
        return q_, qd_, qdd_

    t1 = np.arange(0.0, T1 + dt * 0.5, dt)
    t2 = np.arange(0.0, T2 + dt * 0.5, dt)

    q1,  qd1,  qdd1  = build_segment(t1, q_start, dq1, zeros,   v_junc)
    q2,  qd2,  qdd2  = build_segment(t2, q_mid,   dq2, v_junc,  zeros)

    t_full   = np.concatenate([t1,  t1[-1] + t2[1:]])
    q_full   = np.vstack([q1,   q2[1:]])
    qd_full  = np.vstack([qd1,  qd2[1:]])
    qdd_full = np.vstack([qdd1, qdd2[1:]])

    return t_full, q_full, qd_full, qdd_full


# ===================== RECORD HELPERS =====================

def load_path_record(record_file: str) -> np.ndarray:
    if not os.path.exists(record_file):
        return np.empty((0, 5))
    try:
        data = np.loadtxt(record_file, delimiter=',')
    except (ValueError, OSError):
        return np.empty((0, 5))
    if data.size == 0:
        return np.empty((0, 5))
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] == 7:
        ids = np.arange(1, data.shape[0] + 1)
        return np.column_stack([ids, np.rad2deg(data[:, 3:6]), data[:, 6]])
    return data[:, :5] if data.shape[1] >= 5 else np.empty((0, 5))


def is_unique_path(candidate: np.ndarray, path_record: np.ndarray,
                   threshold: float = 1e-4) -> bool:
    if path_record.shape[0] == 0:
        return True
    diff = np.abs(path_record[:, 1:5] - candidate)
    return np.all(np.any(diff > threshold, axis=1))


# ===================== SAVE TRAJECTORY =====================

def save_trajectory_csv(filepath: str, t: np.ndarray,
                        q: np.ndarray, qd: np.ndarray, qdd: np.ndarray) -> None:
    """
    Save trajectory in row-wise format:
        label, value_t0, value_t1, value_t2, ...

    Each row is one variable. First entry of each row is the label.
    Time values use 3 decimal places; all others use 8.
    """
    rows = [
        ['t']   + [f'{v:.3f}'  for v in t],
        ['dp1'] + [f'{v:.8f}'  for v in q[:,   0]],
        ['dp2'] + [f'{v:.8f}'  for v in q[:,   1]],
        ['dp3'] + [f'{v:.8f}'  for v in q[:,   2]],
        ['dv1'] + [f'{v:.8f}'  for v in qd[:,  0]],
        ['dv2'] + [f'{v:.8f}'  for v in qd[:,  1]],
        ['dv3'] + [f'{v:.8f}'  for v in qd[:,  2]],
        ['da1'] + [f'{v:.8f}'  for v in qdd[:, 0]],
        ['da2'] + [f'{v:.8f}'  for v in qdd[:, 1]],
        ['da3'] + [f'{v:.8f}'  for v in qdd[:, 2]],
    ]
    with open(filepath, 'w') as f:
        for row in rows:
            f.write(','.join(row) + '\n')


# ===================== CSV BATCH LOADER =====================

def load_conditions_csv(csv_path: str) -> list:
    """
    Load trajectory conditions from a CSV file.

    Expected columns (order does not matter, matched by header name):
        path_id   : integer path ID for this condition
        q1-end    : joint 1 end angle (degrees)
        q2-end    : joint 2 end angle (degrees)
        q3-end    : joint 3 end angle (degrees)
        q1-mid    : joint 1 waypoint angle (degrees) — blank or 0 to skip
        q2-mid    : joint 2 waypoint angle (degrees) — blank or 0 to skip
        q3-mid    : joint 3 waypoint angle (degrees) — blank or 0 to skip
        t-total   : total trajectory duration (seconds) — blank or 0 to auto-compute
        num-paths : number of trajectories to generate for this condition

    Returns a list of dicts, one per row.
    """
    required = {'path_id', 'q1-end', 'q2-end', 'q3-end'}
    optional = {'q1-mid', 'q2-mid', 'q3-mid', 't-total', 'num-paths'}

    conditions = []

    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)

        # Strip whitespace from header names
        reader.fieldnames = [h.strip() for h in reader.fieldnames]

        missing = required - set(reader.fieldnames)
        if missing:
            raise ValueError(f"CSV is missing required columns: {missing}")

        for i, row in enumerate(reader):
            row = {k.strip(): v.strip() for k, v in row.items()}

            def get_float(col, default=0.0):
                val = row.get(col, '').strip()
                return float(val) if val != '' else default

            def get_int(col, default=0):
                val = row.get(col, '').strip()
                return int(float(val)) if val != '' else default

            path_id   = get_int('path_id')
            q_end_deg = np.array([get_float('q1-end'),
                                  get_float('q2-end'),
                                  get_float('q3-end')])
            num_paths = get_int('num-paths', default=1)
            t_total   = get_float('t-total', default=0.0)

            # Waypoint: treat as absent if all three are 0 or blank
            q1m = get_float('q1-mid', default=None) if 'q1-mid' in row else None
            q2m = get_float('q2-mid', default=None) if 'q2-mid' in row else None
            q3m = get_float('q3-mid', default=None) if 'q3-mid' in row else None

            # Detect "blank" mid: if user left cells empty they come as ''
            mid_raw = [row.get('q1-mid', ''), row.get('q2-mid', ''), row.get('q3-mid', '')]
            if all(v == '' or float(v) == 0.0 for v in mid_raw if v != ''):
                q_mid_deg = None
            else:
                q_mid_deg = np.array([
                    float(mid_raw[0]) if mid_raw[0] != '' else 0.0,
                    float(mid_raw[1]) if mid_raw[1] != '' else 0.0,
                    float(mid_raw[2]) if mid_raw[2] != '' else 0.0,
                ])

            conditions.append({
                'path_id':   path_id,
                'q_end_deg': q_end_deg,
                'q_mid_deg': q_mid_deg,
                't_total':   t_total if t_total > 0 else None,
                'num_paths': max(1, num_paths),
                'row_index': i + 2,   # for error messages (1-indexed + header)
            })

    print(f"Loaded {len(conditions)} condition(s) from '{csv_path}'")
    return conditions


# ===================== SINGLE TRAJECTORY RUNNER =====================

def run_one_trajectory(
    pid:       int,
    q_end_deg: np.ndarray,
    t_total:   float,               # None = auto-compute
    q_mid_deg: np.ndarray,          # None = no waypoint
    config:    TrajConfig,
    dirs:      dict,
    path_record: np.ndarray,
) -> np.ndarray:
    """
    Generate and save one trajectory. Returns updated path_record.
    """
    angle_dir = dirs['angle']
    xyz_dir   = dirs['xyz']
    traj_dir  = dirs['traj']

    q_end_deg = np.round(np.array(q_end_deg, dtype=float)).astype(int)
    q_end     = np.deg2rad(q_end_deg)

    use_waypoint = q_mid_deg is not None
    if use_waypoint:
        q_mid_deg = np.round(np.array(q_mid_deg, dtype=float)).astype(int)
        q_mid     = np.deg2rad(q_mid_deg)
        dq = np.maximum(np.abs(q_mid - config.q_start),
                        np.abs(q_end  - q_mid))
    else:
        dq = np.abs(q_end - config.q_start)

    T_min = max(np.max(1.875 * dq / config.v_max),
                np.max(np.sqrt(5.77 * dq / config.a_max)))
    T_total = max(t_total, T_min) if t_total is not None else T_min
    print(f"    Duration: {T_total:.2f}s  (min required: {T_min:.2f}s)")

    # Build trajectory
    if use_waypoint:
        T1, T2 = split_time_by_arc(config.q_start, q_mid, q_end, T_total)
        print(f"    Segment durations: T1={T1:.2f}s  T2={T2:.2f}s")
        t, q, qd, qdd = generate_two_segment_trajectory(
            config.q_start, q_mid, q_end,
            T1, T2, config.dt, config.v_max, config.a_max
        )
    else:
        t   = np.arange(0, T_total + config.dt, config.dt)
        f, fd, fdd = min_jerk_profile(t, T_total)
        dq_vec = q_end - config.q_start
        q   = config.q_start + dq_vec * f[:, None]
        qd  = dq_vec * fd[:, None]
        qdd = dq_vec * fdd[:, None]

    N   = len(t)
    xyz = np.array([forward_kinematics(q[k], config.q4) for k in range(N)])

    # Save files
    np.savetxt(os.path.join(angle_dir, f'path_{pid:03d}_angles.csv'),
               q,   delimiter=',', fmt='%.10f')
    np.savetxt(os.path.join(xyz_dir,   f'path_{pid:03d}_xyz.csv'),
               xyz, delimiter=',', fmt='%.10f')
    save_trajectory_csv(
        os.path.join(traj_dir, f'path_{pid:03d}_traj.csv'),
        t, q, qd, qdd
    )

    # Update path record
    new_rec     = np.array([[pid, q_end_deg[0], q_end_deg[1], q_end_deg[2], T_total]])
    path_record = np.vstack([path_record, new_rec])

    # Append to tracking file
    try:
        if not os.path.exists(Tracking_file):
            with open(Tracking_file, 'w') as tf:
                tf.write('pathid,dp1,dp2,dp3,T_total\n')
        with open(Tracking_file, 'a') as tf:
            tf.write(f"{pid},{int(q_end_deg[0])},{int(q_end_deg[1])},"
                     f"{int(q_end_deg[2])},{T_total:.6f}\n")
    except Exception as e:
        print(f"    Warning: could not write tracking file: {e}")

    return path_record


# ===================== MAIN GENERATOR =====================

def generate_trajectories(
    config:    TrajConfig,
    base_dir:  str,
    conditions: list,              # list of dicts from load_conditions_csv
) -> None:
    """
    Run all trajectory conditions from the loaded CSV.
    Each condition can generate one or more trajectories (num-paths).
    If num-paths > 1 for a condition, path_id increments from the base path_id.
    """
    angle_dir  = os.path.join(base_dir, 'Angles')
    xyz_dir    = os.path.join(base_dir, 'XYZ')
    traj_dir   = os.path.join(base_dir, 'Trajectories')
    record_dir = os.path.join(base_dir, 'JointStatesAll')
    for d in [angle_dir, xyz_dir, traj_dir, record_dir]:
        os.makedirs(d, exist_ok=True)

    dirs = {'angle': angle_dir, 'xyz': xyz_dir, 'traj': traj_dir}

    record_file = os.path.join(record_dir, 'all_paths_0.csv')
    path_record = load_path_record(record_file)

    total_conditions = len(conditions)
    total_trajs      = sum(c['num_paths'] for c in conditions)
    print(f"\nTotal conditions : {total_conditions}")
    print(f"Total trajectories: {total_trajs}\n")

    traj_count = 0
    for cond in conditions:
        base_pid  = cond['path_id']
        num_paths = cond['num_paths']

        for i in range(num_paths):
            pid = base_pid + i
            traj_count += 1
            print(f"[{traj_count}/{total_trajs}] path_id={pid}  "
                  f"q_end={cond['q_end_deg'].tolist()}  "
                  f"q_mid={cond['q_mid_deg'].tolist() if cond['q_mid_deg'] is not None else 'none'}  "
                  f"t_total={cond['t_total']}")

            path_record = run_one_trajectory(
                pid        = pid,
                q_end_deg  = cond['q_end_deg'],
                t_total    = cond['t_total'],
                q_mid_deg  = cond['q_mid_deg'],
                config     = config,
                dirs       = dirs,
                path_record= path_record,
            )

        # Save updated record after each condition
        np.savetxt(record_file, path_record, delimiter=',', fmt='%.6f')

    print(f"\nDone. Files saved to: {os.path.abspath(base_dir)}")


# ===================== CLI =====================

def main():
    parser = argparse.ArgumentParser(
        description='PUMA-560 Trajectory Dataset Generator',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
CSV file format (column-wise, one row per condition):
  path_id  : integer ID for the first trajectory of this condition
  q1-end   : joint 1 end angle (degrees)
  q2-end   : joint 2 end angle (degrees)
  q3-end   : joint 3 end angle (degrees)
  q1-mid   : joint 1 waypoint angle (degrees) — blank to skip waypoint
  q2-mid   : joint 2 waypoint angle (degrees) — blank to skip waypoint
  q3-mid   : joint 3 waypoint angle (degrees) — blank to skip waypoint
  t-total  : total duration in seconds — blank to auto-compute
  num-paths: number of trajectories to generate for this condition

Example CSV:
  path_id,q1-end,q2-end,q3-end,q1-mid,q2-mid,q3-mid,t-total,num-paths
  1,120,-30,80,60,10,110,20,3
  4,100,15,90,,,,,1
  5,-140,-45,200,,,,,2
        """
    )
    parser.add_argument('--csv',      type=str, required=True,
                        help='Path to conditions CSV file')
    parser.add_argument('--base-dir', type=str, default=None,
                        help='Output base directory (default: ./test_output)')
    parser.add_argument('--dt',       type=float, default=0.01)
    parser.add_argument('--v-max',    type=float, default=2.0)
    parser.add_argument('--a-max',    type=float, default=7.0)

    args = parser.parse_args()

    if not os.path.exists(args.csv):
        parser.error(f"CSV file not found: {args.csv}")

    config       = TrajConfig()
    config.dt    = args.dt
    config.v_max = args.v_max
    config.a_max = args.a_max

    base_dir   = args.base_dir if args.base_dir is not None else Output_dir
    conditions = load_conditions_csv(args.csv)

    generate_trajectories(
        config     = config,
        base_dir   = base_dir,
        conditions = conditions,
    )


if __name__ == '__main__':
    main()