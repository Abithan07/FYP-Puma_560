import numpy as np
import os
import argparse
import sympy as sp

# ===================== SYMBOLIC VARIABLES =====================

q1, q2, q3 = sp.symbols('q1 q2 q3')
qd1, qd2, qd3 = sp.symbols('qd1 qd2 qd3')

q  = sp.Matrix([q1, q2, q3])
qd = sp.Matrix([qd1, qd2, qd3])

g = 9.81

# ===================== DH PARAMETERS =====================

a1, a2, a3 = 0, -sp.pi/2, 0
L1, L2, L3 = 0, 0, 0.4318
d1, d2, d3 = 0, 0.2435, -0.0934

# ===================== COG =====================

CoG1 = sp.Matrix([0, 0, 0])
CoG2 = sp.Matrix([0.068, 0.006, -0.016])
CoG3 = sp.Matrix([0, -0.143, 0.014])

# ===================== MASSES =====================

m1, m2, m3 = 0.01, 17.4, 4.8

# ===================== TRANSFORM =====================

def T(alpha, a, d, th):
    return sp.Matrix([
        [sp.cos(th), -sp.sin(th), 0, a],
        [sp.sin(th)*sp.cos(alpha), sp.cos(th)*sp.cos(alpha), -sp.sin(alpha), -sp.sin(alpha)*d],
        [sp.sin(th)*sp.sin(alpha), sp.cos(th)*sp.sin(alpha),  sp.cos(alpha),  sp.cos(alpha)*d],
        [0, 0, 0, 1]
    ])

T01 = T(a1, L1, d1, q1)
T12 = T(a2, L2, d2, q2)
T23 = T(a3, L3, d3, q3)

T02 = T01 * T12
T03 = T02 * T23

R01 = T01[:3, :3]
R02 = T02[:3, :3]
R03 = T03[:3, :3]

# ===================== CENTERS OF MASS =====================

rc1 = T01[:3, 3] + R01 * CoG1
rc2 = T02[:3, 3] + R02 * CoG2
rc3 = T03[:3, 3] + R03 * CoG3

# ===================== JACOBIANS =====================

Jv1 = rc1.jacobian(q)
Jv2 = rc2.jacobian(q)
Jv3 = rc3.jacobian(q)

# ===================== INERTIA MATRIX =====================

D = m1*(Jv1.T*Jv1) + m2*(Jv2.T*Jv2) + m3*(Jv3.T*Jv3)

# ===================== GRAVITY =====================

P = g*(m1*rc1[2] + m2*rc2[2] + m3*rc3[2])
G = sp.Matrix([sp.diff(P, qi) for qi in q])

# ===================== CHRISTOFFEL TENSOR =====================

C = sp.MutableDenseNDimArray.zeros(3,3,3)

for i in range(3):
    for j in range(3):
        for k in range(3):
            C[i,j,k] = 0.5 * (
                sp.diff(D[i,j], q[k]) +
                sp.diff(D[i,k], q[j]) -
                sp.diff(D[j,k], q[i])
            )

# ===================== CORIOLIS VECTOR =====================

Cvec = sp.Matrix([0, 0, 0])

for i in range(3):
    for j in range(3):
        for k in range(3):
            Cvec[i] += C[i,j,k] * qd[j] * qd[k]

# ===================== NUMERICAL FUNCTIONS =====================

D_func = sp.lambdify((q1,q2,q3), D, 'numpy')
G_func = sp.lambdify((q1,q2,q3), G, 'numpy')
C_func = sp.lambdify((q1,q2,q3,qd1,qd2,qd3), Cvec, 'numpy')

# ===================== PROCESS FUNCTION =====================

def process_file(in_file, out_file):

    data = np.loadtxt(in_file, delimiter=",", dtype=str)

    # remove labels
    data = data[:,1:].astype(float)

    t  = data[0]
    dp = data[1:4].T
    dv = data[4:7].T
    da = data[7:10].T

    N = len(t)

    tau = np.zeros((N,3))
    Mv  = np.zeros((N,3))
    Cv  = np.zeros((N,3))
    Gv  = np.zeros((N,3))

    for i in range(N):

        qk  = dp[i]
        dqk = dv[i]
        ddq = da[i]

        Dk = np.array(D_func(*qk), dtype=float)
        Gk = np.array(G_func(*qk), dtype=float).flatten()
        Ck = np.array(C_func(*qk, *dqk), dtype=float).flatten()

        Mvec = Dk @ ddq

        tau[i] = Mvec + Ck + Gk
        Mv[i] = Mvec
        Cv[i] = Ck
        Gv[i] = Gk

    out = np.vstack([
        t,
        dp[:,0], dp[:,1], dp[:,2],
        dv[:,0], dv[:,1], dv[:,2],
        da[:,0], da[:,1], da[:,2],
        tau[:,0], tau[:,1], tau[:,2],
        Mv[:,0], Mv[:,1], Mv[:,2],
        Cv[:,0], Cv[:,1], Cv[:,2],
        Gv[:,0], Gv[:,1], Gv[:,2],
    ])

    labels = np.array([
        "t",
        "dp1","dp2","dp3",
        "dv1","dv2","dv3",
        "da1","da2","da3",
        "tau1","tau2","tau3",
        "m1","m2","m3",
        "c1","c2","c3",
        "g1","g2","g3"
    ]).reshape(-1,1)

    out_str = np.zeros_like(out, dtype=object)

    for i in range(out.shape[0]):
        if i == 0:
            # time row → 3 decimals
            out_str[i] = np.char.mod('%.3f', out[i])
        else:
            # everything else → 8 decimals
            out_str[i] = np.char.mod('%.8f', out[i])

    final = np.hstack((labels, out_str))

    np.savetxt(out_file, final, delimiter=",", fmt="%s")

# ===================== MAIN =====================

def main(start_id, num_paths):

    traj_dir = "Dataset/Trajectories"
    out_dir  = "Dataset/JointStates"
    os.makedirs(out_dir, exist_ok=True)

    for i in range(num_paths):

        pid = start_id + i

        in_file  = os.path.join(traj_dir, f"path_{pid:03d}_trajectory.csv")
        out_file = os.path.join(out_dir, f"path_{pid:03d}_joint_states.csv")

        print(f"Processing {in_file}")

        process_file(in_file, out_file)

    print("All trajectories processed successfully.")

# ===================== RUN =====================

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("start_id", type=int)
    parser.add_argument("num_paths", type=int)

    args = parser.parse_args()

    main(args.start_id, args.num_paths)

"""
Usage:
    python inv_dyn.py <start_id> <num_paths>
    Example: python3 inv_dyn.py 1 10
This will process the trajectories with IDs from 1 to 10, and save the joint states in the Dataset/JointStates directory.
"""