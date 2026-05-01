#!/usr/bin/env python3
"""
PUMA-560 PATH + TRAJECTORY DATASET GENERATOR
Generates:
1) Joint Angles
2) End Effector XYZ
3) Full trajectory (q, qd, qdd)

Constraints:
- |velocity| ≤ 2 rad/s
- |acceleration| ≤ 7 rad/s²
"""

import argparse
import numpy as np
import os
from pathlib import Path
from typing import Tuple


# ===================== PUMA DH PARAMETERS =====================

ALPHA = np.array([0.0, -np.pi/2, 0.0, np.pi/2])
A_DH = np.array([0.0, 0.0, 0.4318, 0.0])
D_DH = np.array([0.0, 0.2435, -0.0934, 0.4331])

T_BASE = np.eye(4)
T_BASE[2, 3] = 0.6718


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
        
        self.q1_min_deg = -150.0
        self.q1_max_deg = 150.0
        
        self.q2_min_deg = -215.0
        self.q2_max_deg = 45.0
        
        self.q3_min_deg = -35.0
        self.q3_max_deg = 215.0
        
        self.q4_deg = 0.0
        self.q4 = np.deg2rad(self.q4_deg)


def dh_transform(alpha: float, a: float, d: float, theta: float) -> np.ndarray:
    """
    Compute DH transformation matrix.
    
    Parameters:
    -----------
    alpha : float
        Twist angle (radians)
    a : float
        Link length
    d : float
        Link offset
    theta : float
        Joint angle (radians)
        
    Returns:
    --------
    T : ndarray
        4x4 transformation matrix
    """
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    ct = np.cos(theta)
    st = np.sin(theta)
    
    T = np.array([
        [ct, -st, 0, a],
        [st*ca, ct*ca, -sa, -sa*d],
        [st*sa, ct*sa, ca, ca*d],
        [0, 0, 0, 1]
    ], dtype=float)
    
    return T


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


def forward_kinematics(q: np.ndarray, q4: float) -> np.ndarray:
    """
    Compute end-effector position using forward kinematics.
    
    Parameters:
    -----------
    q : ndarray
        Joint angles (q1, q2, q3) in radians
    q4 : float
        Fourth joint angle in radians
        
    Returns:
    --------
    xyz : ndarray
        End-effector position (x, y, z)
    """
    q1, q2, q3 = q
    
    T01 = T_BASE @ dh_transform(ALPHA[0], A_DH[0], D_DH[0], q1)
    T12 = dh_transform(ALPHA[1], A_DH[1], D_DH[1], q2)
    T23 = dh_transform(ALPHA[2], A_DH[2], D_DH[2], q3)
    T34 = dh_transform(ALPHA[3], A_DH[3], D_DH[3], q4)
    
    T04 = T01 @ T12 @ T23 @ T34
    
    return T04[:3, 3]


def load_path_record(record_file: str) -> np.ndarray:
    """
    Load existing path record to check for duplicates.
    
    Parameters:
    -----------
    record_file : str
        Path to record CSV file
        
    Returns:
    --------
    pathRecord : ndarray
        Existing path records (path_id, q1_deg, q2_deg, q3_deg, T_total)
    """
    if not os.path.exists(record_file):
        return np.empty((0, 5))
    
    try:
        data = np.loadtxt(record_file, delimiter=",")
    except (ValueError, OSError):
        return np.empty((0, 5))
    
    if data.size == 0:
        return np.empty((0, 5))
    
    if data.ndim == 1:
        data = data.reshape(1, -1)
    
    if data.shape[1] == 7:
        # Convert from 7-column format to 5-column
        path_ids = np.arange(1, data.shape[0] + 1)
        path_record = np.column_stack([
            path_ids,
            np.rad2deg(data[:, 3:6]),  # Convert radians to degrees
            data[:, 6]  # T_total
        ])
        return path_record
    
    return data[:, :5] if data.shape[1] >= 5 else np.empty((0, 5))


def is_unique_path(candidate: np.ndarray, path_record: np.ndarray, threshold: float = 1e-4) -> bool:
    """
    Check if candidate path is unique compared to existing paths.
    
    Parameters:
    -----------
    candidate : ndarray
        Candidate path (q1_deg, q2_deg, q3_deg, T_total)
    path_record : ndarray
        Existing path records
    threshold : float
        Tolerance for uniqueness check
        
    Returns:
    --------
    is_unique : bool
        True if candidate is unique
    """
    if path_record.shape[0] == 0:
        return True
    
    # Compare relevant columns: q_deg + T_total (columns 1:5 from path_record)
    existing = path_record[:, 1:5]
    diff = np.abs(existing - candidate)
    
    # Path is unique if it differs from all existing paths in at least one dimension
    return np.all(np.any(diff > threshold, axis=1))


def generate_trajectories(num_paths: int, config: TrajConfig, 
                         base_dir: str = "/home/priyankan/Desktop/FYP-Puma_560/Dataset", 
                         start_id: int = 601,
                         seed: int = None,
                         plot: bool = False,
                         q_end_deg: np.ndarray = None,
                         t_total: float = None,
                         path_id: int = None) -> None:
    """
    Generate trajectory dataset.
    
    Parameters:
    -----------
    num_paths : int
        Number of trajectories to generate
    config : TrajConfig
        Configuration object
    base_dir : str
        Base output directory (default: /home/priyankan/Desktop/FYP-Puma_560/Dataset)
    start_id : int
        Starting path ID
    seed : int
        Random seed for reproducibility
    plot : bool
        Whether to display live plots
    q_end_deg : ndarray
        If provided, generate single trajectory to this endpoint (overrides num_paths)
    t_total : float
        If provided with q_end_deg, use this duration instead of auto-computing
    path_id : int
        If provided with q_end_deg, use this specific path ID instead of start_id
    """
    
    # Set random seed
    if seed is not None:
        np.random.seed(seed)
    
    # Create output directories
    angle_dir = os.path.join(base_dir, "Angles")
    xyz_dir = os.path.join(base_dir, "XYZ")
    traj_dir = os.path.join(base_dir, "Trajectories")
    record_dir = os.path.join(base_dir, "JointStatesAll")
    
    for dir_path in [angle_dir, xyz_dir, traj_dir, record_dir]:
        os.makedirs(dir_path, exist_ok=True)
    
    record_file = os.path.join(record_dir, "all_paths_0.csv")
    
    # Load existing path record
    path_record = load_path_record(record_file)
    
    # Determine number of trajectories and whether to use specified endpoint
    if q_end_deg is not None:
        q_end_deg = np.array(q_end_deg, dtype=float)
        num_traj = 1
        use_specified_endpoint = True
        # Use custom path_id if provided, otherwise use start_id
        actual_start_id = path_id if path_id is not None else start_id
    else:
        num_traj = num_paths
        use_specified_endpoint = False
        actual_start_id = start_id
    
    # Setup visualization if requested
    if plot:
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D
            
            fig_path = plt.figure(figsize=(8, 6))
            ax_path = fig_path.add_subplot(111, projection='3d')
            ax_path.set_xlabel('X')
            ax_path.set_ylabel('Y')
            ax_path.set_zlabel('Z')
            ax_path.set_title('Generated PUMA-560 Paths')
            ax_path.grid(True)
            
            fig_end = plt.figure(figsize=(8, 6))
            ax_end = fig_end.add_subplot(111, projection='3d')
            ax_end.set_xlabel('X')
            ax_end.set_ylabel('Y')
            ax_end.set_zlabel('Z')
            ax_end.set_title('Generated PUMA-560 End Points')
            ax_end.grid(True)
            
            plot_enabled = True
        except ImportError:
            print("Warning: matplotlib not available, skipping plots")
            plot_enabled = False
    else:
        plot_enabled = False
    
    # Generate trajectories
    for p in range(num_traj):
        path_id_current = actual_start_id + p
        print(f"Generating trajectory {p+1} / {num_traj} (path ID: {path_id_current})")
        
        # Generate random unique end configuration or use specified endpoint
        if use_specified_endpoint:
            # Use the specified end angles
            q_end = np.deg2rad(q_end_deg)
            dq = np.abs(q_end - config.q_start)
            
            # Compute minimum required duration from constraints
            T_vel = np.max(1.875 * dq / config.v_max)
            T_acc = np.max(np.sqrt(5.77 * dq / config.a_max))
            T_min = max(T_vel, T_acc)
            
            # Use specified duration or auto-computed minimum
            if t_total is not None:
                T_total = max(t_total, T_min)
                print(f"  Using specified duration T={T_total:.2f}s (min required: {T_min:.2f}s)")
            else:
                T_total = T_min
                print(f"  Auto-computed duration T={T_total:.2f}s")
        else:
            # Random generation with uniqueness check
            valid_path = False
            while not valid_path:
                q_end_deg = np.array([
                    config.q1_min_deg + (config.q1_max_deg - config.q1_min_deg) * np.random.rand(),
                    config.q2_min_deg + (config.q2_max_deg - config.q2_min_deg) * np.random.rand(),
                    config.q3_min_deg + (config.q3_max_deg - config.q3_min_deg) * np.random.rand()
                ])
                
                q_end = np.deg2rad(q_end_deg)
                dq = np.abs(q_end - config.q_start)
                
                T_vel = np.max(1.875 * dq / config.v_max)
                T_acc = np.max(np.sqrt(5.77 * dq / config.a_max))
                T_min = max(T_vel, T_acc)
                
                T_rand = config.possible_T[np.random.randint(len(config.possible_T))]
                T_total = max(T_min, T_rand)
                
                candidate = np.concatenate([q_end_deg, [T_total]])
                
                if is_unique_path(candidate, path_record):
                    valid_path = True
        
        # Generate time vector
        t = np.arange(0, T_total + config.dt, config.dt)
        N = len(t)
        
        # Compute minimum jerk profile
        f, fd, fdd = min_jerk_profile(t, T_total)
        
        # Compute joint trajectories
        dq = q_end - config.q_start
        
        q = np.zeros((N, 3))
        qd = np.zeros((N, 3))
        qdd = np.zeros((N, 3))
        
        for j in range(3):
            dqj = dq[j]
            q[:, j] = config.q_start[j] + dqj * f
            qd[:, j] = dqj * fd
            qdd[:, j] = dqj * fdd
        
        # Compute end-effector trajectory
        xyz = np.zeros((N, 3))
        for k in range(N):
            xyz[k] = forward_kinematics(q[k], config.q4)
        
        # Save files
        np.savetxt(os.path.join(angle_dir, f"path_{path_id_current:03d}_angles.csv"), 
                   q, delimiter=",", fmt="%.10f")
        
        np.savetxt(os.path.join(xyz_dir, f"path_{path_id_current:03d}_xyz.csv"), 
                   xyz, delimiter=",", fmt="%.10f")
        
        # Save trajectory with headers (row-wise format)
        traj_data = np.vstack([t, q.T, qd.T, qdd.T])
        
        labels = np.array([
            "t", "dp1", "dp2", "dp3", "dv1", "dv2", "dv3", "da1", "da2", "da3"
        ]).reshape(-1, 1)
        
        traj_str = []
        for i, row in enumerate(traj_data):
            if i == 0:
                # time row → 3 decimal places
                formatted = np.char.mod('%.3f', row)
            else:
                # all others → 8 decimal places
                formatted = np.char.mod('%.8f', row)
            traj_str.append(formatted)
        
        traj_str = np.array(traj_str)
        traj_with_labels = np.hstack((labels, traj_str))
        
        np.savetxt(os.path.join(traj_dir, f"path_{path_id_current:03d}_traj.csv"),
                   traj_with_labels, delimiter=",", fmt="%s")
        
        # Update path record
        new_record = np.array([[path_id_current, q_end_deg[0], q_end_deg[1], q_end_deg[2], T_total]])
        path_record = np.vstack([path_record, new_record])
        
        np.savetxt(record_file, path_record, delimiter=",", fmt="%.6f")
        
        # Plot if requested
        if plot_enabled:
            color = np.random.rand(3,)
            ax_path.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], color=color, linewidth=1.5)
            ax_end.scatter(xyz[-1, 0], xyz[-1, 1], xyz[-1, 2], 
                          color=color, edgecolors='k', s=20)
            plt.pause(0.001)
    
    if plot_enabled:
        plt.show()
    
    print("All trajectories generated successfully.")


def main():
    """Command-line interface"""
    parser = argparse.ArgumentParser(
        description="PUMA-560 Trajectory Dataset Generator"
    )
    
    parser.add_argument(
        "num_paths", 
        type=int, 
        help="Number of trajectories to generate"
    )
    
    parser.add_argument(
        "--start-id", 
        type=int, 
        default=601, 
        help="Starting path ID (default: 601)"
    )
    
    parser.add_argument(
        "--base-dir", 
        type=str, 
        default="/home/priyankan/Desktop/FYP-Puma_560/Dataset", 
        help="Base output directory (default: /home/priyankan/Desktop/FYP-Puma_560/Dataset)"
    )
    
    parser.add_argument(
        "--seed", 
        type=int, 
        default=None, 
        help="Random seed for reproducibility"
    )
    
    parser.add_argument(
        "--plot", 
        action="store_true", 
        help="Show live 3D plots during generation"
    )
    
    parser.add_argument(
        "--dt", 
        type=float, 
        default=0.01, 
        help="Sampling interval (default: 0.01)"
    )
    
    parser.add_argument(
        "--v-max", 
        type=float, 
        default=2.0, 
        help="Maximum joint velocity in rad/s (default: 2.0)"
    )
    
    parser.add_argument(
        "--a-max", 
        type=float, 
        default=7.0, 
        help="Maximum joint acceleration in rad/s^2 (default: 7.0)"
    )
    
    parser.add_argument(
        "--q-end-deg",
        type=float,
        nargs=3,
        default=None,
        metavar=("Q1", "Q2", "Q3"),
        help="Generate single trajectory to specified end joint angles in degrees (e.g., --q-end-deg 45 30 90)"
    )
    
    parser.add_argument(
        "--t-total",
        type=float,
        default=None,
        help="Total trajectory duration in seconds (only used with --q-end-deg). If not specified, auto-computed from constraints."
    )
    
    parser.add_argument(
        "--path-id",
        type=int,
        default=None,
        help="Specific path ID for single trajectory (only used with --q-end-deg). Overrides --start-id."
    )
    
    args = parser.parse_args()
    
    # Create configuration
    config = TrajConfig()
    config.dt = args.dt
    config.v_max = args.v_max
    config.a_max = args.a_max
    
    # Generate trajectories
    generate_trajectories(
        num_paths=args.num_paths,
        config=config,
        base_dir=args.base_dir,
        start_id=args.start_id,
        seed=args.seed,
        plot=args.plot,
        q_end_deg=args.q_end_deg,
        t_total=args.t_total,
        path_id=args.path_id
    )


if __name__ == "__main__":
    main()
    
    """
    Usage Examples:
    
    Generate 10 random trajectories with ID 601-610:
        python3 trajectory_generator.py 10
    
    Generate 20 random trajectories starting from ID 1000:
        python3 trajectory_generator.py 20 --start-id 1000
    
    Generate 5 random trajectories with custom output directory:
        python3 trajectory_generator.py 5 --base-dir ./my_dataset
    
    Generate 10 random trajectories with reproducible randomness:
        python3 trajectory_generator.py 10 --seed 42
    
    Generate 10 random trajectories with live visualization:
        python3 trajectory_generator.py 10 --plot
    
    Generate 10 random trajectories with custom constraints:
        python3 trajectory_generator.py 10 --v-max 1.5 --a-max 5.0
    
    Generate single trajectory to specified end joint angles (in degrees):
        python3 trajectory_generator.py 1 --q-end-deg 45.0 30.0 90.0
    
    Generate single trajectory to specified angles with custom duration:
        python3 trajectory_generator.py 1 --q-end-deg 155.0 -220.0 40.0 --t-total 24.0 --path-id 1122
    
    Generate single trajectory with specific path ID:
        python3 trajectory_generator.py 1 --q-end-deg 45.0 30.0 90.0 --path-id 999
    
    Generate single trajectory with custom output directory and path ID:
        python3 trajectory_generator.py 1 --q-end-deg 45.0 30.0 90.0 --path-id 1234 --base-dir ./my_dataset
    
    Generate single trajectory with visualization:
        python3 trajectory_generator.py 1 --q-end-deg 45.0 30.0 90.0 --plot
    """
