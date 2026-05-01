#!/usr/bin/env python3
"""
PUMA-560 trajectory dataset generator.

Generates trajectory CSV files containing time, joint position, velocity,
and acceleration rows for each path.
"""

import argparse
import os
from typing import Tuple

import numpy as np


start_id = 1
Output_dir = "/home/priyankan/Desktop/FYP-Puma_560/Test_data"
Tracking_file = "/home/priyankan/Desktop/FYP-Puma_560/test_data_tracking.csv"


# ===================== SETTINGS =====================

class TrajConfig:
    """Trajectory generation configuration"""
    def __init__(self):
        self.dt = 0.01
        self.possible_T = np.array([12, 16, 20, 24])
        
        self.v_max = 2.0
        self.a_max = 7.0
        
        self.q_start_deg = np.array([0.0, 45.0, 135.0])
        self.q_start = np.deg2rad(self.q_start_deg)
        

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


def load_path_record(record_file: str) -> np.ndarray:
    """Load existing tracked paths as [pathid, dp1, dp2, dp3, T_total]."""
    if not os.path.exists(record_file):
        return np.empty((0, 5))

    try:
        data = np.genfromtxt(record_file, delimiter=",", skip_header=1)
    except (OSError, ValueError):
        return np.empty((0, 5))

    if data.size == 0:
        return np.empty((0, 5))

    if data.ndim == 1:
        data = data.reshape(1, -1)

    if data.shape[1] < 5:
        return np.empty((0, 5))

    return data[:, :5]


def is_unique_path(candidate: np.ndarray, path_record: np.ndarray, threshold: float = 1e-4) -> bool:
    """Return True when candidate [dp1, dp2, dp3, T_total] does not match an existing track."""
    if path_record.shape[0] == 0:
        return True

    existing = path_record[:, 1:5]
    diff = np.abs(existing - candidate)
    return np.all(np.any(diff > threshold, axis=1))

def generate_trajectories(num_paths: int, config: TrajConfig, start_id: int = None, base_dir: str = None) -> None:
    """
    Generate trajectory dataset.
    
    Parameters:
    -----------
    num_paths : int
        Number of trajectories to generate
    config : TrajConfig
        Configuration object
    start_id : int
        Starting path ID
    base_dir : str
        Base output directory.
    """

    module_start_id = globals().get('start_id', 1)
    if base_dir is None:
        base_dir = globals().get('Output_dir', "/home/priyankan/Desktop/FYP-Puma_560/Dataset")

    # traj_dir = os.path.join(base_dir, "Trajectories")
    traj_dir = base_dir

    os.makedirs(traj_dir, exist_ok=True)

    actual_start_id = start_id if start_id is not None else module_start_id
    path_record = load_path_record(Tracking_file)

    for p in range(num_paths):
        path_id_current = actual_start_id + p
        print(f"Generating trajectory {p + 1} / {num_paths} (path ID: {path_id_current})")

        valid_path = False
        while not valid_path:
            if np.random.rand() < 0.5:
                q1 = 100.0 + (150.0 - 100.0) * np.random.rand()
            else:
                q1 = -150.0 + (-100.0 - -150.0) * np.random.rand()

            q2 = -60.0 + (15.0 - -60.0) * np.random.rand()

            if np.random.rand() < 0.5:
                q3 = 60.0 + (100.0 - 60.0) * np.random.rand()
            else:
                q3 = 180.0 + (225.0 - 180.0) * np.random.rand()

            q_end_deg = np.round(np.array([q1, q2, q3])).astype(int)
            q_end = np.deg2rad(q_end_deg)
            dq = np.abs(q_end - config.q_start)

            T_vel = np.max(1.875 * dq / config.v_max)
            T_acc = np.max(np.sqrt(5.77 * dq / config.a_max))
            T_min = max(T_vel, T_acc)
            T_total = max(T_min, config.possible_T[np.random.randint(len(config.possible_T))])

            candidate = np.concatenate([q_end_deg, [T_total]])
            valid_path = is_unique_path(candidate, path_record)

        t = np.arange(0, T_total + config.dt, config.dt)
        N = len(t)

        f, fd, fdd = min_jerk_profile(t, T_total)

        dq = q_end - config.q_start

        q = np.zeros((N, 3))
        qd = np.zeros((N, 3))
        qdd = np.zeros((N, 3))
        
        for j in range(3):
            dqj = dq[j]
            q[:, j] = config.q_start[j] + dqj * f
            qd[:, j] = dqj * fd
            qdd[:, j] = dqj * fdd

        traj_data = np.vstack([t, q.T, qd.T, qdd.T])
        labels = np.array([
            "t", "dp1", "dp2", "dp3", "dv1", "dv2", "dv3", "da1", "da2", "da3"
        ]).reshape(-1, 1)

        traj_str = []
        for i, row in enumerate(traj_data):
            if i == 0:
                formatted = np.char.mod('%.3f', row)
            else:
                formatted = np.char.mod('%.8f', row)
            traj_str.append(formatted)

        traj_str = np.array(traj_str)
        traj_with_labels = np.hstack((labels, traj_str))

        np.savetxt(os.path.join(traj_dir, f"path_{path_id_current:03d}_traj.csv"),
                   traj_with_labels, delimiter=",", fmt="%s")

        new_record = np.array([[path_id_current, q_end_deg[0], q_end_deg[1], q_end_deg[2], T_total]])
        path_record = np.vstack([path_record, new_record])

        if not os.path.exists(Tracking_file):
            with open(Tracking_file, "w", encoding="utf-8") as tracking_handle:
                tracking_handle.write("pathid,dp1,dp2,dp3,T_total\n")

        with open(Tracking_file, "a", encoding="utf-8") as tracking_handle:
            tracking_handle.write(
                f"{path_id_current},{int(q_end_deg[0])},{int(q_end_deg[1])},{int(q_end_deg[2])},{T_total:.6f}\n"
            )

    print("All trajectories generated successfully.")


def main():
    """Command-line interface"""
    parser = argparse.ArgumentParser(description="PUMA-560 trajectory dataset generator")

    parser.add_argument(
        "number_of_trajectories",
        type=int,
        help="Number of trajectories to generate",
    )

    parser.add_argument(
        "--start-id",
        type=int,
        default=None,
        help="Starting path ID (default: uses module start_id)",
    )
    
    args = parser.parse_args()
    
    # Create configuration
    config = TrajConfig()
    
    # Generate trajectories
    generate_trajectories(
        num_paths=args.number_of_trajectories,
        config=config,
        start_id=args.start_id,
    )


if __name__ == "__main__":
    main()
