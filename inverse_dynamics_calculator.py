#!/usr/bin/env python3
"""
PUMA-560 INVERSE DYNAMICS CALCULATOR
Computes torques from generated trajectories using symbolic inverse dynamics.

Generates output with:
- Joint positions (q1, q2, q3)
- Joint velocities (dq1, dq2, dq3)
- Joint accelerations (ddq1, ddq2, ddq3)
- Computed torques (tau1, tau2, tau3)
- Inertia effects (m1, m2, m3)
- Coriolis/centripetal effects (c1, c2, c3)
- Gravity effects (g1, g2, g3)
"""

import numpy as np
import os
import sys
import sympy as sp
from pathlib import Path
from typing import Tuple, Dict, List
import argparse
from sympy import symbols, cos, sin, diff, Matrix, simplify, lambdify
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
COG1 = np.array([0.0, 0.0, 0.0])  # Link-1 CoG at joint axis
COG2 = np.array([0.068, 0.006, -0.016])
COG3 = np.array([0.000, -0.143, 0.014])

# Link inertia matrices
# Motor inertias
IM1 = 1.14
IM2 = 4.71
IM3 = 0.83

# Link inertia components
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
    Symbolic and numeric inverse dynamics calculator for PUMA-560
    """
    
    def __init__(self):
        """Initialize symbolic variables and compute dynamics functions"""
        print("Initializing inverse dynamics calculator...")
        
        # Symbolic variables
        self.q1, self.q2, self.q3 = symbols('q1 q2 q3', real=True)
        self.q_sym = Matrix([self.q1, self.q2, self.q3])
        
        # Build symbolic dynamics
        self._build_symbolic_dynamics()
        
        print("Symbolic dynamics computation complete.")
        print("Converting to numeric functions...")
        
        # Convert to numeric functions
        self._convert_to_numeric_functions()
        
        print("Ready for trajectory processing.")
    
    def _dh_transform_symbolic(self, alpha: float, a: float, d: float, theta: sp.Basic) -> sp.Matrix:
        """
        Compute DH transformation matrix symbolically.
        
        Parameters:
        -----------
        alpha : float
            Twist angle (radians)
        a : float
            Link length
        d : float
            Link offset
        theta : sp.Basic
            Joint angle (symbolic)
            
        Returns:
        --------
        T : sp.Matrix
            4x4 transformation matrix
        """
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        ct = cos(theta)
        st = sin(theta)
        
        T = sp.Matrix([
            [ct, -st, 0, a],
            [st*ca, ct*ca, -sa, -sa*d],
            [st*sa, ct*sa, ca, ca*d],
            [0, 0, 0, 1]
        ])
        
        return T
    
    def _build_symbolic_dynamics(self):
        """Build symbolic inertia matrix, gravity vector, and Christoffel symbols"""
        
        # DH Transformations
        T01 = self._dh_transform_symbolic(ALPHA[0], A_DH[0], D_DH[0], self.q1)
        T12 = self._dh_transform_symbolic(ALPHA[1], A_DH[1], D_DH[1], self.q2)
        T23 = self._dh_transform_symbolic(ALPHA[2], A_DH[2], D_DH[2], self.q3)
        
        T02 = T01 * T12
        T03 = T02 * T23
        
        # Extract rotation matrices
        R01 = T01[:3, :3]
        R02 = T02[:3, :3]
        R03 = T03[:3, :3]
        
        # Angular velocity Jacobians
        Jw1 = sp.Matrix([
            [R01[:3, 2].T],
            [sp.Matrix([0, 0, 0]).T],
            [sp.Matrix([0, 0, 0]).T]
        ])
        Jw1 = sp.BlockMatrix([[R01[:3, 2].T], [sp.zeros(1, 3)], [sp.zeros(1, 3)]]).as_explicit()
        Jw1 = sp.Matrix([
            [R01[0, 2], 0, 0],
            [R01[1, 2], 0, 0],
            [R01[2, 2], 0, 0]
        ])
        
        Jw2 = sp.Matrix([
            [R01[0, 2], R02[0, 2], 0],
            [R01[1, 2], R02[1, 2], 0],
            [R01[2, 2], R02[2, 2], 0]
        ])
        
        Jw3 = sp.Matrix([
            [R01[0, 2], R02[0, 2], R03[0, 2]],
            [R01[1, 2], R02[1, 2], R03[1, 2]],
            [R01[2, 2], R02[2, 2], R03[2, 2]]
        ])
        
        # Center of mass positions
        rc1 = T01[:3, 3] + R01 * sp.Matrix(COG1)
        rc2 = T02[:3, 3] + R02 * sp.Matrix(COG2)
        rc3 = T03[:3, 3] + R03 * sp.Matrix(COG3)
        
        # Linear velocity Jacobians
        Jv1 = sp.Matrix([
            [diff(rc1[0], self.q1), diff(rc1[0], self.q2), diff(rc1[0], self.q3)],
            [diff(rc1[1], self.q1), diff(rc1[1], self.q2), diff(rc1[1], self.q3)],
            [diff(rc1[2], self.q1), diff(rc1[2], self.q2), diff(rc1[2], self.q3)]
        ])
        
        Jv2 = sp.Matrix([
            [diff(rc2[0], self.q1), diff(rc2[0], self.q2), diff(rc2[0], self.q3)],
            [diff(rc2[1], self.q1), diff(rc2[1], self.q2), diff(rc2[1], self.q3)],
            [diff(rc2[2], self.q1), diff(rc2[2], self.q2), diff(rc2[2], self.q3)]
        ])
        
        Jv3 = sp.Matrix([
            [diff(rc3[0], self.q1), diff(rc3[0], self.q2), diff(rc3[0], self.q3)],
            [diff(rc3[1], self.q1), diff(rc3[1], self.q2), diff(rc3[1], self.q3)],
            [diff(rc3[2], self.q1), diff(rc3[2], self.q2), diff(rc3[2], self.q3)]
        ])
        
        # Inertia matrices
        I1 = sp.Matrix([
            [I1xx, 0, 0],
            [0, I1yy, 0],
            [0, 0, I1zz]
        ])
        
        I2 = sp.Matrix([
            [I2xx, 0, 0],
            [0, I2yy, 0],
            [0, 0, I2zz]
        ])
        
        I3 = sp.Matrix([
            [I3xx, 0, 0],
            [0, I3yy, 0],
            [0, 0, I3zz]
        ])
        
        # Inertia matrix D(q)
        print("Computing inertia matrix D(q)...")
        self.D = (M1 * Jv1.T * Jv1 + 
                  Jw1.T * R01 * I1 * R01.T * Jw1 +
                  M2 * Jv2.T * Jv2 + 
                  Jw2.T * R02 * I2 * R02.T * Jw2 +
                  M3 * Jv3.T * Jv3 + 
                  Jw3.T * R03 * I3 * R03.T * Jw3)
        
        self.D = simplify(self.D)
        
        # Gravity vector G(q)
        print("Computing gravity vector G(q)...")
        P = G * (M1 * rc1[2] + M2 * rc2[2] + M3 * rc3[2])
        self.G = sp.Matrix([
            diff(P, self.q1),
            diff(P, self.q2),
            diff(P, self.q3)
        ])
        
        self.G = simplify(self.G)
        
        # Christoffel symbols C[i,j,k]
        print("Computing Christoffel symbols...")
        self.C = {}
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    c_ijk = sp.Rational(1, 2) * (
                        diff(self.D[i, j], self.q_sym[k]) + 
                        diff(self.D[i, k], self.q_sym[j]) - 
                        diff(self.D[j, k], self.q_sym[i])
                    )
                    self.C[(i, j, k)] = simplify(c_ijk)
    
    def _convert_to_numeric_functions(self):
        """Convert symbolic expressions to numeric functions"""
        
        print("Converting D(q) to numeric function...")
        self.D_func = lambdify((self.q1, self.q2, self.q3), self.D, 'numpy')
        
        print("Converting G(q) to numeric function...")
        self.G_func = lambdify((self.q1, self.q2, self.q3), self.G, 'numpy')
        
        print("Converting C(q) to numeric functions...")
        self.C_func = {}
        for (i, j, k), expr in self.C.items():
            self.C_func[(i, j, k)] = lambdify((self.q1, self.q2, self.q3), expr, 'numpy')
    
    def compute_torques(self, q: np.ndarray, dq: np.ndarray, ddq: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute joint torques using inverse dynamics.
        
        Parameters:
        -----------
        q : ndarray
            Joint positions (q1, q2, q3) in radians
        dq : ndarray
            Joint velocities (dq1, dq2, dq3)
        ddq : ndarray
            Joint accelerations (ddq1, ddq2, ddq3)
            
        Returns:
        --------
        tau : ndarray
            Torques (tau1, tau2, tau3)
        M : ndarray
            Inertia term M(q)*ddq
        C_vec : ndarray
            Coriolis/centripetal term
        G_vec : ndarray
            Gravity term
        """
        # Evaluate D matrix
        D = np.array(self.D_func(q[0], q[1], q[2]), dtype=float)
        
        # Evaluate gravity vector
        G_vec = np.array(self.G_func(q[0], q[1], q[2]), dtype=float).flatten()
        
        # Compute Coriolis/centripetal term
        C_vec = np.zeros(3)
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    c_ijk = float(self.C_func[(i, j, k)](q[0], q[1], q[2]))
                    C_vec[i] += c_ijk * dq[j] * dq[k]
        
        # Compute inertia term
        M = D @ ddq
        
        # Compute total torque
        tau = M + C_vec + G_vec
        
        return tau, M, C_vec, G_vec
    
    def process_trajectory_file(self, traj_file: str, output_dir: str) -> None:
        """
        Process a trajectory CSV file and compute inverse dynamics.
        
        Parameters:
        -----------
        traj_file : str
            Path to trajectory CSV file
        output_dir : str
            Directory to save output CSV file
        """
        
        # Load trajectory data
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
        
        Expected format (row-wise):
        Row 1: t | t[0] | t[1] | ... | t[N]
        Row 2: dp1 | q1[0] | q1[1] | ... | q1[N]
        Row 3: dp2 | q2[0] | q2[1] | ... | q2[N]
        Row 4: dp3 | q3[0] | q3[1] | ... | q3[N]
        Row 5: dv1 | dq1[0] | dq1[1] | ... | dq1[N]
        Row 6: dv2 | dq2[0] | dq2[1] | ... | dq2[N]
        Row 7: dv3 | dq3[0] | dq3[1] | ... | dq3[N]
        Row 8: da1 | ddq1[0] | ddq1[1] | ... | ddq1[N]
        Row 9: da2 | ddq2[0] | ddq2[1] | ... | ddq2[N]
        Row 10: da3 | ddq3[0] | ddq3[1] | ... | ddq3[N]
        """
        data = []
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                # Skip label, convert rest to floats
                values = [float(x) for x in row[1:]]
                data.append(values)
        
        data = np.array(data)
        
        # Extract components
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
        
        Format (row-wise):
        Row 1: t | time values
        Row 2-4: dp1,dp2,dp3 | position values
        Row 5-7: dv1,dv2,dv3 | velocity values
        Row 8-10: da1,da2,da3 | acceleration values
        Row 11-13: tau1,tau2,tau3 | torque values
        Row 14-16: m1,m2,m3 | inertia term values
        Row 17-19: c1,c2,c3 | Coriolis term values
        Row 20-22: g1,g2,g3 | gravity term values
        """
        
        os.makedirs(output_dir, exist_ok=True)
        
        # Generate output filename
        base_name = Path(traj_file).stem  # e.g., "path_5600_traj"
        output_name = base_name.replace('_traj', '_joint_states') + '.csv'
        output_file = os.path.join(output_dir, output_name)
        
        # Row labels and data
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
        
        # Write CSV file
        with open(output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            for label, values in rows:
                row = [label] + [f'{v:.8f}' if label == 't' else f'{v:.10f}' for v in values]
                writer.writerow(row)
        
        print(f"Saved results to {output_file}")
    
    def process_trajectory_directory(self, traj_dir: str, output_dir: str) -> None:
        """
        Process all trajectory files in a directory.
        
        Parameters:
        -----------
        traj_dir : str
            Directory containing trajectory CSV files
        output_dir : str
            Directory to save output CSV files
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
        description="PUMA-560 Inverse Dynamics Calculator"
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
        help="Output directory for results (default: same as input directory)"
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
        # Single file processing
        if args.output:
            output_dir = args.output
        else:
            output_dir = input_path.parent / args.output_dir_name
        
        calc.process_trajectory_file(str(input_path), str(output_dir))
    
    elif input_path.is_dir():
        # Directory processing
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
