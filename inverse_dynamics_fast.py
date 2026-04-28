#!/usr/bin/env python3
"""
PUMA-560 INVERSE DYNAMICS CALCULATOR (Fast)
Computes torques from generated trajectories using efficient numerical methods.
"""

import numpy as np
import os
import sys
from pathlib import Path
from typing import Tuple, Dict, List
import argparse
import csv


# ==================== PUMA-560 PARAMETERS ====================

# DH Parameters
ALPHA = np.array([0.0, -np.pi/2, 0.0, np.pi/2])
A_DH = np.array([0.0, 0.0, 0.4318, 0.0])
D_DH = np.array([0.0, 0.2435, -0.0934, 0.4331])

# Base position
BASE_Z = 0.6718

# Link masses (kg)
M1 = 0.01
M2 = 17.4
M3 = 4.8

# Link centers of mass (m)
COG1 = np.array([0.0, 0.0, 0.0])
COG2 = np.array([0.068, 0.006, -0.016])
COG3 = np.array([0.000, -0.143, 0.014])

# Link inertia matrices
IM1 = 1.14
IM2 = 4.71
IM3 = 0.83

I1xx = 0.745
I1yy = 0.745
I1zz = 0.35 + IM1

I2xx = 2.6245
I2yy = 2.6245
I2zz = 0.539 + IM2

I3xx = 0.458
I3yy = 0.458
I3zz = 0.086 + IM3

# Gravity
G_CONST = 9.81


class InverseDynamicsCalculator:
    """
    Fast numerical inverse dynamics calculator for PUMA-560
    """
    
    def __init__(self):
        """Initialize calculator"""
        print("Initializing inverse dynamics calculator (fast numerical method)...")
        
        # Pre-compute inertia matrices
        self.I1 = np.diag([I1xx, I1yy, I1zz])
        self.I2 = np.diag([I2xx, I2yy, I2zz])
        self.I3 = np.diag([I3xx, I3yy, I3zz])
        
        self.eps = 1e-6  # Step size for numerical differentiation
    
    def _dh_transform(self, alpha: float, a: float, d: float, theta: float) -> np.ndarray:
        """Compute DH transformation matrix"""
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
    
    def _forward_kinematics_fast(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute forward kinematics frames efficiently
        """
        q1, q2, q3 = q
        
        # Base transformation
        T_base = np.eye(4)
        T_base[2, 3] = BASE_Z
        
        # DH transformations
        T01 = T_base @ self._dh_transform(ALPHA[0], A_DH[0], D_DH[0], q1)
        T12 = self._dh_transform(ALPHA[1], A_DH[1], D_DH[1], q2)
        T23 = self._dh_transform(ALPHA[2], A_DH[2], D_DH[2], q3)
        
        T02 = T01 @ T12
        T03 = T02 @ T23
        
        return T01, T02, T03
    
    def _cog_positions(self, T01: np.ndarray, T02: np.ndarray, T03: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Compute center of mass positions"""
        R01 = T01[:3, :3]
        R02 = T02[:3, :3]
        R03 = T03[:3, :3]
        
        rc1 = T01[:3, 3] + R01 @ COG1
        rc2 = T02[:3, 3] + R02 @ COG2
        rc3 = T03[:3, 3] + R03 @ COG3
        
        return rc1, rc2, rc3
    
    def _jacobian_numerical_fast(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute linear and angular Jacobians using numerical differentiation
        Returns: Jv1, Jv2, Jv3, Jw1, Jw2, Jw3
        """
        eps = self.eps
        T01, T02, T03 = self._forward_kinematics_fast(q)
        rc1, rc2, rc3 = self._cog_positions(T01, T02, T03)
        
        Jv1 = np.zeros((3, 3))
        Jv2 = np.zeros((3, 3))
        Jv3 = np.zeros((3, 3))
        
        # Numerical differentiation for linear Jacobians
        for i in range(3):
            q_pert = q.copy()
            q_pert[i] += eps
            
            T01_pert, T02_pert, T03_pert = self._forward_kinematics_fast(q_pert)
            R01_pert = T01_pert[:3, :3]
            R02_pert = T02_pert[:3, :3]
            R03_pert = T03_pert[:3, :3]
            
            rc1_pert = T01_pert[:3, 3] + R01_pert @ COG1
            rc2_pert = T02_pert[:3, 3] + R02_pert @ COG2
            rc3_pert = T03_pert[:3, 3] + R03_pert @ COG3
            
            Jv1[:, i] = (rc1_pert - rc1) / eps
            Jv2[:, i] = (rc2_pert - rc2) / eps
            Jv3[:, i] = (rc3_pert - rc3) / eps
        
        # Angular Jacobians (analytical for revolute joints)
        R01 = T01[:3, :3]
        R02 = T02[:3, :3]
        R03 = T03[:3, :3]
        
        Jw1 = np.array([
            [R01[0, 2], 0, 0],
            [R01[1, 2], 0, 0],
            [R01[2, 2], 0, 0]
        ])
        
        Jw2 = np.array([
            [R01[0, 2], R02[0, 2], 0],
            [R01[1, 2], R02[1, 2], 0],
            [R01[2, 2], R02[2, 2], 0]
        ])
        
        Jw3 = np.array([
            [R01[0, 2], R02[0, 2], R03[0, 2]],
            [R01[1, 2], R02[1, 2], R03[1, 2]],
            [R01[2, 2], R02[2, 2], R03[2, 2]]
        ])
        
        return Jv1, Jv2, Jv3, Jw1, Jw2, Jw3
    
    def _inertia_matrix_fast(self, q: np.ndarray) -> np.ndarray:
        """
        Compute inertia matrix D(q) using Jacobians
        """
        T01, T02, T03 = self._forward_kinematics_fast(q)
        R01 = T01[:3, :3]
        R02 = T02[:3, :3]
        R03 = T03[:3, :3]
        
        Jv1, Jv2, Jv3, Jw1, Jw2, Jw3 = self._jacobian_numerical_fast(q)
        
        D = (M1 * Jv1.T @ Jv1 + Jw1.T @ R01 @ self.I1 @ R01.T @ Jw1 +
             M2 * Jv2.T @ Jv2 + Jw2.T @ R02 @ self.I2 @ R02.T @ Jw2 +
             M3 * Jv3.T @ Jv3 + Jw3.T @ R03 @ self.I3 @ R03.T @ Jw3)
        
        return D
    
    def _christoffel_fast(self, q: np.ndarray) -> np.ndarray:
        """
        Compute Christoffel symbols using numerical differentiation of D matrix
        Only compute the unique elements needed for the Coriolis computation
        """
        eps = self.eps * 100  # Larger step for second derivatives
        C = np.zeros((3, 3, 3))
        
        D_nominal = self._inertia_matrix_fast(q)
        
        for k in range(3):
            q_plus = q.copy()
            q_plus[k] += eps
            q_minus = q.copy()
            q_minus[k] -= eps
            
            D_plus = self._inertia_matrix_fast(q_plus)
            D_minus = self._inertia_matrix_fast(q_minus)
            
            dD_dqk = (D_plus - D_minus) / (2 * eps)
            
            for i in range(3):
                for j in range(3):
                    # Get other required derivatives
                    q_p = q.copy()
                    q_p[j] += eps
                    q_m = q.copy()
                    q_m[j] -= eps
                    D_p = self._inertia_matrix_fast(q_p)
                    D_m = self._inertia_matrix_fast(q_m)
                    dD_dqj = (D_p - D_m) / (2 * eps)
                    
                    q_p = q.copy()
                    q_p[i] += eps
                    q_m = q.copy()
                    q_m[i] -= eps
                    D_p = self._inertia_matrix_fast(q_p)
                    D_m = self._inertia_matrix_fast(q_m)
                    dD_dqi = (D_p - D_m) / (2 * eps)
                    
                    C[i, j, k] = 0.5 * (dD_dqk[i, j] + dD_dqj[i, k] - dD_dqi[j, k])
        
        return C
    
    def _gravity_fast(self, q: np.ndarray) -> np.ndarray:
        """
        Compute gravity vector using numerical differentiation
        """
        eps = self.eps
        G_vec = np.zeros(3)
        
        for i in range(3):
            q_plus = q.copy()
            q_plus[i] += eps
            q_minus = q.copy()
            q_minus[i] -= eps
            
            T01_plus, T02_plus, T03_plus = self._forward_kinematics_fast(q_plus)
            T01_minus, T02_minus, T03_minus = self._forward_kinematics_fast(q_minus)
            
            R01_plus = T01_plus[:3, :3]
            R02_plus = T02_plus[:3, :3]
            R03_plus = T03_plus[:3, :3]
            
            R01_minus = T01_minus[:3, :3]
            R02_minus = T02_minus[:3, :3]
            R03_minus = T03_minus[:3, :3]
            
            rc1_plus = T01_plus[:3, 3] + R01_plus @ COG1
            rc2_plus = T02_plus[:3, 3] + R02_plus @ COG2
            rc3_plus = T03_plus[:3, 3] + R03_plus @ COG3
            
            rc1_minus = T01_minus[:3, 3] + R01_minus @ COG1
            rc2_minus = T02_minus[:3, 3] + R02_minus @ COG2
            rc3_minus = T03_minus[:3, 3] + R03_minus @ COG3
            
            PE_plus = G_CONST * (M1 * rc1_plus[2] + M2 * rc2_plus[2] + M3 * rc3_plus[2])
            PE_minus = G_CONST * (M1 * rc1_minus[2] + M2 * rc2_minus[2] + M3 * rc3_minus[2])
            
            G_vec[i] = (PE_plus - PE_minus) / (2 * eps)
        
        return G_vec
    
    def compute_torques(self, q: np.ndarray, dq: np.ndarray, ddq: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute torques using inverse dynamics formula:
        tau = D(q)*ddq + C(q,dq)*dq + G(q)
        """
        # Inertia matrix
        D = self._inertia_matrix_fast(q)
        
        # Gravity vector
        G_vec = self._gravity_fast(q)
        
        # Christoffel symbols
        C = self._christoffel_fast(q)
        
        # Coriolis/centripetal term
        C_vec = np.zeros(3)
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    C_vec[i] += C[i, j, k] * dq[j] * dq[k]
        
        # Inertia term
        M = D @ ddq
        
        # Total torque
        tau = M + C_vec + G_vec
        
        return tau, M, C_vec, G_vec
    
    def process_trajectory_file(self, traj_file: str, output_dir: str) -> None:
        """Process a single trajectory file"""
        print(f"Loading trajectory from {traj_file}...")
        data = self._load_trajectory_csv(traj_file)
        
        t = data['t']
        q = data['q']
        dq = data['dq']
        ddq = data['ddq']
        
        N = len(t)
        
        # Initialize output arrays
        tau_out = np.zeros((N, 3))
        M_out = np.zeros((N, 3))
        C_out = np.zeros((N, 3))
        G_out = np.zeros((N, 3))
        
        # Compute torques
        print(f"Computing torques for {N} time steps...")
        for k in range(N):
            if (k + 1) % max(1, N // 10) == 0:
                print(f"  Progress: {k+1}/{N}")
            
            tau, M, C_vec, G_vec = self.compute_torques(q[k], dq[k], ddq[k])
            tau_out[k] = tau
            M_out[k] = M
            C_out[k] = C_vec
            G_out[k] = G_vec
        
        # Save results
        self._save_results_csv(
            t, q, dq, ddq, tau_out, M_out, C_out, G_out,
            traj_file, output_dir
        )
    
    def _load_trajectory_csv(self, filename: str) -> Dict[str, np.ndarray]:
        """Load trajectory data from CSV"""
        data = []
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                values = [float(x) for x in row[1:]]
                data.append(values)
        
        data = np.array(data)
        
        result = {
            't': data[0],
            'q': data[1:4].T,
            'dq': data[4:7].T,
            'ddq': data[7:10].T
        }
        
        return result
    
    def _save_results_csv(self, t: np.ndarray, q: np.ndarray, dq: np.ndarray, 
                          ddq: np.ndarray, tau: np.ndarray, M: np.ndarray, 
                          C: np.ndarray, G: np.ndarray, 
                          traj_file: str, output_dir: str) -> None:
        """Save results to CSV"""
        os.makedirs(output_dir, exist_ok=True)
        
        base_name = Path(traj_file).stem
        output_name = base_name.replace('_traj', '_joint_states') + '.csv'
        output_file = os.path.join(output_dir, output_name)
        
        rows = [
            ('t', t),
            ('dp1', q[:, 0]),
            ('dp2', q[:, 1]),
            ('dp3', q[:, 2]),
            ('dv1', dq[:, 0]),
            ('dv2', dq[:, 1]),
            ('dv3', dq[:, 2]),
            ('da1', ddq[:, 0]),
            ('da2', ddq[:, 1]),
            ('da3', ddq[:, 2]),
            ('tau1', tau[:, 0]),
            ('tau2', tau[:, 1]),
            ('tau3', tau[:, 2]),
            ('m1', M[:, 0]),
            ('m2', M[:, 1]),
            ('m3', M[:, 2]),
            ('c1', C[:, 0]),
            ('c2', C[:, 1]),
            ('c3', C[:, 2]),
            ('g1', G[:, 0]),
            ('g2', G[:, 1]),
            ('g3', G[:, 2])
        ]
        
        with open(output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            for label, values in rows:
                fmt = '%.8f' if label == 't' else '%.10f'
                row = [label] + [fmt % v for v in values]
                writer.writerow(row)
        
        print(f"Saved results to {output_file}")
    
    def process_trajectory_directory(self, traj_dir: str, output_dir: str) -> None:
        """Process all trajectories in a directory"""
        traj_files = sorted(Path(traj_dir).glob('*_traj.csv'))
        
        if not traj_files:
            print(f"No trajectory files found in {traj_dir}")
            return
        
        print(f"Found {len(traj_files)} trajectory files")
        
        for idx, traj_file in enumerate(traj_files):
            print(f"\n[{idx+1}/{len(traj_files)}] Processing {traj_file.name}...")
            try:
                self.process_trajectory_file(str(traj_file), output_dir)
            except Exception as e:
                print(f"Error processing {traj_file.name}: {e}")
                import traceback
                traceback.print_exc()


def main():
    """Command-line interface"""
    parser = argparse.ArgumentParser(
        description="PUMA-560 Inverse Dynamics Calculator"
    )
    
    parser.add_argument(
        "input",
        type=str,
        help="Input trajectory file or directory"
    )
    
    parser.add_argument(
        "-o", "--output",
        type=str,
        default=None,
        help="Output directory"
    )
    
    parser.add_argument(
        "--output-dir-name",
        type=str,
        default="Joint_states",
        help="Output subdirectory name (default: Joint_states)"
    )
    
    args = parser.parse_args()
    
    calc = InverseDynamicsCalculator()
    
    input_path = Path(args.input)
    
    if input_path.is_file():
        if args.output:
            output_dir = args.output
        else:
            output_dir = str(input_path.parent.parent / args.output_dir_name)
        
        calc.process_trajectory_file(str(input_path), str(output_dir))
    
    elif input_path.is_dir():
        if args.output:
            output_dir = args.output
        else:
            output_dir = str(input_path.parent / args.output_dir_name)
        
        calc.process_trajectory_directory(str(input_path), str(output_dir))
    
    else:
        print(f"Error: {args.input} does not exist")
        sys.exit(1)


if __name__ == "__main__":
    main()
