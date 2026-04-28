#!/usr/bin/env python3
"""
PUMA-560 INVERSE DYNAMICS CALCULATOR (Optimized)
Computes torques from generated trajectories using numerical inverse dynamics.

This version uses pre-derived analytical expressions for better performance.
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

# Link centers of mass (m, in link frame)
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
G = 9.81


class InverseDynamicsCalculator:
    """
    Numerical inverse dynamics calculator for PUMA-560
    Uses pre-derived analytical expressions for efficiency
    """
    
    def __init__(self):
        """Initialize calculator"""
        print("Initializing inverse dynamics calculator (numerical method)...")
        self.eps = 1e-8  # For numerical differentiation
    
    def _dh_transform(self, alpha: float, a: float, d: float, theta: float) -> np.ndarray:
        """
        Compute DH transformation matrix.
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
    
    def _forward_kinematics(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute forward kinematics and frames.
        
        Returns:
        --------
        T01, T02, T03, T04 : transformation matrices
        R01, R02, R03, R04 : rotation matrices
        p01, p02, p03, p04 : positions
        """
        q1, q2, q3 = q
        
        # Base transformation
        T_base = np.eye(4)
        T_base[2, 3] = BASE_Z
        
        # DH transformations
        T01 = T_base @ self._dh_transform(ALPHA[0], A_DH[0], D_DH[0], q1)
        T12 = self._dh_transform(ALPHA[1], A_DH[1], D_DH[1], q2)
        T23 = self._dh_transform(ALPHA[2], A_DH[2], D_DH[2], q3)
        T34 = self._dh_transform(ALPHA[3], A_DH[3], D_DH[3], 0.0)
        
        T02 = T01 @ T12
        T03 = T02 @ T23
        T04 = T03 @ T34
        
        return T01, T02, T03, T04
    
    def _jacobian_linear(self, q: np.ndarray, link: int) -> np.ndarray:
        """
        Compute linear Jacobian for a link using numerical differentiation.
        
        Parameters:
        -----------
        q : ndarray
            Joint angles
        link : int
            Link number (1, 2, or 3)
            
        Returns:
        --------
        Jv : ndarray
            3x3 linear Jacobian
        """
        T01, T02, T03, T04 = self._forward_kinematics(q)
        
        # Get current position
        if link == 1:
            R01 = T01[:3, :3]
            p_link = T01[:3, 3] + R01 @ COG1
        elif link == 2:
            R02 = T02[:3, :3]
            p_link = T02[:3, 3] + R02 @ COG2
        elif link == 3:
            R03 = T03[:3, :3]
            p_link = T03[:3, 3] + R03 @ COG3
        else:
            raise ValueError("link must be 1, 2, or 3")
        
        # Numerical differentiation for Jacobian
        Jv = np.zeros((3, 3))
        for i in range(3):
            q_plus = q.copy()
            q_plus[i] += self.eps
            q_minus = q.copy()
            q_minus[i] -= self.eps
            
            T01_plus, T02_plus, T03_plus, _ = self._forward_kinematics(q_plus)
            T01_minus, T02_minus, T03_minus, _ = self._forward_kinematics(q_minus)
            
            if link == 1:
                R01_plus = T01_plus[:3, :3]
                p_plus = T01_plus[:3, 3] + R01_plus @ COG1
                R01_minus = T01_minus[:3, :3]
                p_minus = T01_minus[:3, 3] + R01_minus @ COG1
            elif link == 2:
                R02_plus = T02_plus[:3, :3]
                p_plus = T02_plus[:3, 3] + R02_plus @ COG2
                R02_minus = T02_minus[:3, :3]
                p_minus = T02_minus[:3, 3] + R02_minus @ COG2
            elif link == 3:
                R03_plus = T03_plus[:3, :3]
                p_plus = T03_plus[:3, 3] + R03_plus @ COG3
                R03_minus = T03_minus[:3, :3]
                p_minus = T03_minus[:3, 3] + R03_minus @ COG3
            
            Jv[:, i] = (p_plus - p_minus) / (2 * self.eps)
        
        return Jv
    
    def _jacobian_angular(self, q: np.ndarray, link: int) -> np.ndarray:
        """
        Compute angular Jacobian for a link.
        
        For revolute joints rotating about z-axis (PUMA-560):
        """
        T01, T02, T03, T04 = self._forward_kinematics(q)
        
        if link == 1:
            R01 = T01[:3, :3]
            Jw = np.zeros((3, 3))
            Jw[:, 0] = R01[:, 2]  # z-axis of frame 1
        elif link == 2:
            R01 = T01[:3, :3]
            R02 = T02[:3, :3]
            Jw = np.zeros((3, 3))
            Jw[:, 0] = R01[:, 2]  # z-axis of frame 1
            Jw[:, 1] = R02[:, 2]  # z-axis of frame 2
        elif link == 3:
            R01 = T01[:3, :3]
            R02 = T02[:3, :3]
            R03 = T03[:3, :3]
            Jw = np.zeros((3, 3))
            Jw[:, 0] = R01[:, 2]  # z-axis of frame 1
            Jw[:, 1] = R02[:, 2]  # z-axis of frame 2
            Jw[:, 2] = R03[:, 2]  # z-axis of frame 3
        else:
            raise ValueError("link must be 1, 2, or 3")
        
        return Jw
    
    def _inertia_matrix(self, q: np.ndarray) -> np.ndarray:
        """
        Compute inertia matrix D(q) numerically.
        
        D(q) = sum_i [ m_i * Jv_i^T * Jv_i + Jw_i^T * R_i * I_i * R_i^T * Jw_i ]
        """
        T01, T02, T03, T04 = self._forward_kinematics(q)
        
        D = np.zeros((3, 3))
        
        # Inertia matrices in link frames
        I1 = np.diag([I1xx, I1yy, I1zz])
        I2 = np.diag([I2xx, I2yy, I2zz])
        I3 = np.diag([I3xx, I3yy, I3zz])
        
        # Link 1
        Jv1 = self._jacobian_linear(q, 1)
        Jw1 = self._jacobian_angular(q, 1)
        R01 = T01[:3, :3]
        D += M1 * Jv1.T @ Jv1 + Jw1.T @ R01 @ I1 @ R01.T @ Jw1
        
        # Link 2
        Jv2 = self._jacobian_linear(q, 2)
        Jw2 = self._jacobian_angular(q, 2)
        R02 = T02[:3, :3]
        D += M2 * Jv2.T @ Jv2 + Jw2.T @ R02 @ I2 @ R02.T @ Jw2
        
        # Link 3
        Jv3 = self._jacobian_linear(q, 3)
        Jw3 = self._jacobian_angular(q, 3)
        R03 = T03[:3, :3]
        D += M3 * Jv3.T @ Jv3 + Jw3.T @ R03 @ I3 @ R03.T @ Jw3
        
        return D
    
    def _coriolis_matrix_numerical(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        """
        Compute Coriolis/centripetal term numerically using finite differences of D matrix.
        """
        eps = 1e-7
        
        # C[i,j,k] = 0.5 * (dD[i,j]/dq_k + dD[i,k]/dq_j - dD[j,k]/dq_i)
        
        D_nominal = self._inertia_matrix(q)
        
        C_tensor = np.zeros((3, 3, 3))
        
        for k in range(3):
            q_plus = q.copy()
            q_plus[k] += eps
            q_minus = q.copy()
            q_minus[k] -= eps
            
            D_plus = self._inertia_matrix(q_plus)
            D_minus = self._inertia_matrix(q_minus)
            
            dD_dqk = (D_plus - D_minus) / (2 * eps)
            
            for i in range(3):
                for j in range(3):
                    # Only compute the unique derivatives needed
                    if k >= j:  # Use symmetry
                        for m in range(k+1):
                            q_p2 = q.copy()
                            q_p2[m] += eps
                            q_m2 = q.copy()
                            q_m2[m] -= eps
                            
                            D_p2 = self._inertia_matrix(q_p2)
                            D_m2 = self._inertia_matrix(q_m2)
                            dD_dqm = (D_p2 - D_m2) / (2 * eps)
                            
                            if m == k:
                                dD_i_j_k = dD_dqk[i, j]
                            if m == j:
                                dD_i_k_j = dD_dqm[i, k]
                            if m == i:
                                dD_j_k_i = dD_dqm[j, k]
                        
                        if k == j:
                            C_tensor[i, j, k] = 0.5 * (dD_i_j_k + dD_i_j_k - dD_j_k_i)
        
        return C_tensor
    
    def _gravity_vector(self, q: np.ndarray) -> np.ndarray:
        """
        Compute gravity vector G(q) using potential energy method.
        
        G = grad_q(PE) where PE = g * (m1*z_c1 + m2*z_c2 + m3*z_c3)
        """
        eps = 1e-8
        
        G_grad = np.zeros(3)
        
        # Numerical differentiation
        for i in range(3):
            q_plus = q.copy()
            q_plus[i] += eps
            q_minus = q.copy()
            q_minus[i] -= eps
            
            T01_plus, T02_plus, T03_plus, _ = self._forward_kinematics(q_plus)
            T01_minus, T02_minus, T03_minus, _ = self._forward_kinematics(q_minus)
            
            # Positions of centers of mass
            R01_plus = T01_plus[:3, :3]
            rc1_plus = T01_plus[:3, 3] + R01_plus @ COG1
            R02_plus = T02_plus[:3, :3]
            rc2_plus = T02_plus[:3, 3] + R02_plus @ COG2
            R03_plus = T03_plus[:3, :3]
            rc3_plus = T03_plus[:3, 3] + R03_plus @ COG3
            
            R01_minus = T01_minus[:3, :3]
            rc1_minus = T01_minus[:3, 3] + R01_minus @ COG1
            R02_minus = T02_minus[:3, :3]
            rc2_minus = T02_minus[:3, 3] + R02_minus @ COG2
            R03_minus = T03_minus[:3, :3]
            rc3_minus = T03_minus[:3, 3] + R03_minus @ COG3
            
            # Potential energy derivative
            PE_plus = G * (M1 * rc1_plus[2] + M2 * rc2_plus[2] + M3 * rc3_plus[2])
            PE_minus = G * (M1 * rc1_minus[2] + M2 * rc2_minus[2] + M3 * rc3_minus[2])
            
            G_grad[i] = (PE_plus - PE_minus) / (2 * eps)
        
        return G_grad
    
    def _coriolis_vector_fast(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        """
        Compute Coriolis vector using fast numerical method.
        """
        eps = 1e-7
        
        # C_vec[i] = sum_{j,k} C[i,j,k] * dq[j] * dq[k]
        # Approximate using finite differences of D matrix derivatives
        
        D = self._inertia_matrix(q)
        
        C_vec = np.zeros(3)
        
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    # Compute C[i,j,k] numerically
                    q_perturb = q.copy()
                    q_perturb[k] += eps
                    D_plus = self._inertia_matrix(q_perturb)
                    
                    q_perturb = q.copy()
                    q_perturb[k] -= eps
                    D_minus = self._inertia_matrix(q_perturb)
                    
                    dD_ij_dqk = (D_plus[i, j] - D_minus[i, j]) / (2 * eps)
                    
                    # Similar for other derivatives
                    q_perturb = q.copy()
                    q_perturb[j] += eps
                    D_plus = self._inertia_matrix(q_perturb)
                    q_perturb = q.copy()
                    q_perturb[j] -= eps
                    D_minus = self._inertia_matrix(q_perturb)
                    dD_ik_dqj = (D_plus[i, k] - D_minus[i, k]) / (2 * eps)
                    
                    q_perturb = q.copy()
                    q_perturb[i] += eps
                    D_plus = self._inertia_matrix(q_perturb)
                    q_perturb = q.copy()
                    q_perturb[i] -= eps
                    D_minus = self._inertia_matrix(q_perturb)
                    dD_jk_dqi = (D_plus[j, k] - D_minus[j, k]) / (2 * eps)
                    
                    c_ijk = 0.5 * (dD_ij_dqk + dD_ik_dqj - dD_jk_dqi)
                    C_vec[i] += c_ijk * dq[j] * dq[k]
        
        return C_vec
    
    def compute_torques(self, q: np.ndarray, dq: np.ndarray, ddq: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute joint torques using inverse dynamics.
        
        tau = D(q)*ddq + C(q,dq)*dq + G(q)
        """
        # Compute components
        D = self._inertia_matrix(q)
        G_vec = self._gravity_vector(q)
        C_vec = self._coriolis_vector_fast(q, dq)
        
        # Compute inertia term
        M = D @ ddq
        
        # Total torque
        tau = M + C_vec + G_vec
        
        return tau, M, C_vec, G_vec
    
    def process_trajectory_file(self, traj_file: str, output_dir: str) -> None:
        """
        Process a trajectory CSV file and compute inverse dynamics.
        """
        print(f"Loading trajectory from {traj_file}...")
        data = self._load_trajectory_csv(traj_file)
        
        t = data['t']
        q = data['q']  # N x 3
        dq = data['dq']  # N x 3
        ddq = data['ddq']  # N x 3
        
        N = len(t)
        
        # Initialize output arrays
        tau_out = np.zeros((N, 3))
        M_out = np.zeros((N, 3))
        C_out = np.zeros((N, 3))
        G_out = np.zeros((N, 3))
        
        # Compute torques at each time step
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
        """
        Load trajectory data from CSV file.
        """
        data = []
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                values = [float(x) for x in row[1:]]
                data.append(values)
        
        data = np.array(data)
        
        result = {
            't': data[0],
            'q': data[1:4].T,      # N x 3
            'dq': data[4:7].T,     # N x 3
            'ddq': data[7:10].T    # N x 3
        }
        
        return result
    
    def _save_results_csv(self, t: np.ndarray, q: np.ndarray, dq: np.ndarray, 
                          ddq: np.ndarray, tau: np.ndarray, M: np.ndarray, 
                          C: np.ndarray, G: np.ndarray, 
                          traj_file: str, output_dir: str) -> None:
        """
        Save inverse dynamics results to CSV file.
        """
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
        """
        Process all trajectory files in a directory.
        """
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
        description="PUMA-560 Inverse Dynamics Calculator (Numerical)"
    )
    
    parser.add_argument(
        "input",
        type=str,
        help="Input trajectory file or directory containing trajectory files"
    )
    
    parser.add_argument(
        "-o", "--output",
        type=str,
        default=None,
        help="Output directory for results (default: Dataset/Joint_states)"
    )
    
    parser.add_argument(
        "--output-dir-name",
        type=str,
        default="Joint_states",
        help="Name of output subdirectory (default: Joint_states)"
    )
    
    args = parser.parse_args()
    
    # Initialize calculator
    calc = InverseDynamicsCalculator()
    
    # Determine input/output paths
    input_path = Path(args.input)
    
    if input_path.is_file():
        if args.output:
            output_dir = args.output
        else:
            output_dir = input_path.parent.parent / args.output_dir_name
        
        calc.process_trajectory_file(str(input_path), str(output_dir))
    
    elif input_path.is_dir():
        if args.output:
            output_dir = args.output
        else:
            output_dir = input_path.parent / args.output_dir_name
        
        calc.process_trajectory_directory(str(input_path), str(output_dir))
    
    else:
        print(f"Error: {args.input} does not exist")
        sys.exit(1)


if __name__ == "__main__":
    main()
