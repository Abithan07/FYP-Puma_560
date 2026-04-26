#!/usr/bin/env python3
"""
Exact inverse dynamics model for the 3-DOF PUMA arm.

This module mirrors the symbolic dynamics generation used in inv_dyn.py so the
controller uses the same D(q), C(q, qd), and G(q) terms as the offline dataset
generation pipeline.
"""

import numpy as np
import sympy as sp

# ===================== SYMBOLIC MODEL =====================

q1, q2, q3 = sp.symbols('q1 q2 q3')
qd1, qd2, qd3 = sp.symbols('qd1 qd2 qd3')

q = sp.Matrix([q1, q2, q3])
qd = sp.Matrix([qd1, qd2, qd3])

g = 9.81

a1, a2, a3 = 0, -sp.pi / 2, 0
L1, L2, L3 = 0, 0, 0.4318
d1, d2, d3 = 0, 0.2435, -0.0934

CoG1 = sp.Matrix([0, 0, 0])
CoG2 = sp.Matrix([0.068, 0.006, -0.016])
CoG3 = sp.Matrix([0, -0.143, 0.014])

m1, m2, m3 = 0.01, 17.4, 4.8


def T(alpha, a, d, th):
    return sp.Matrix([
        [sp.cos(th), -sp.sin(th), 0, a],
        [sp.sin(th) * sp.cos(alpha), sp.cos(th) * sp.cos(alpha), -sp.sin(alpha), -sp.sin(alpha) * d],
        [sp.sin(th) * sp.sin(alpha), sp.cos(th) * sp.sin(alpha), sp.cos(alpha), sp.cos(alpha) * d],
        [0, 0, 0, 1],
    ])


T01 = T(a1, L1, d1, q1)
T12 = T(a2, L2, d2, q2)
T23 = T(a3, L3, d3, q3)

T02 = T01 * T12
T03 = T02 * T23

R01 = T01[:3, :3]
R02 = T02[:3, :3]
R03 = T03[:3, :3]

rc1 = T01[:3, 3] + R01 * CoG1
rc2 = T02[:3, 3] + R02 * CoG2
rc3 = T03[:3, 3] + R03 * CoG3

Jv1 = rc1.jacobian(q)
Jv2 = rc2.jacobian(q)
Jv3 = rc3.jacobian(q)

D_sym = m1 * (Jv1.T * Jv1) + m2 * (Jv2.T * Jv2) + m3 * (Jv3.T * Jv3)

P = g * (m1 * rc1[2] + m2 * rc2[2] + m3 * rc3[2])
G_sym = sp.Matrix([sp.diff(P, qi) for qi in q])

C = sp.MutableDenseNDimArray.zeros(3, 3, 3)
for i in range(3):
    for j in range(3):
        for k in range(3):
            C[i, j, k] = 0.5 * (
                sp.diff(D_sym[i, j], q[k])
                + sp.diff(D_sym[i, k], q[j])
                - sp.diff(D_sym[j, k], q[i])
            )

Cvec_sym = sp.Matrix([0, 0, 0])
for i in range(3):
    for j in range(3):
        for k in range(3):
            Cvec_sym[i] += C[i, j, k] * qd[j] * qd[k]

# ===================== NUMERICAL FUNCTIONS =====================

D_func = sp.lambdify((q1, q2, q3), D_sym, 'numpy')
G_func = sp.lambdify((q1, q2, q3), G_sym, 'numpy')
C_func = sp.lambdify((q1, q2, q3, qd1, qd2, qd3), Cvec_sym, 'numpy')


def compute_D_matrix(q_in):
    q_in = np.asarray(q_in, dtype=float)
    return np.array(D_func(*q_in), dtype=float)


def compute_gravity(q_in):
    q_in = np.asarray(q_in, dtype=float)
    return np.array(G_func(*q_in), dtype=float).flatten()


def compute_coriolis(q_in, qd_in):
    q_in = np.asarray(q_in, dtype=float)
    qd_in = np.asarray(qd_in, dtype=float)
    return np.array(C_func(*q_in, *qd_in), dtype=float).flatten()


def compute_inverse_dynamics(q_in, qd_in, qdd_in):
    q_in = np.asarray(q_in, dtype=float)
    qd_in = np.asarray(qd_in, dtype=float)
    qdd_in = np.asarray(qdd_in, dtype=float)

    D = compute_D_matrix(q_in)
    C = compute_coriolis(q_in, qd_in)
    G = compute_gravity(q_in)
    tau = D @ qdd_in + C + G
    return tau, D, C, G


def compute_gravity_analytical(q_in):
    return compute_gravity(q_in)


def compute_christoffel_symbols(q_in):
    q_in = np.asarray(q_in, dtype=float)
    dq = 1e-7
    gamma = np.zeros((3, 3, 3), dtype=float)
    D0 = compute_D_matrix(q_in)

    for k in range(3):
        q_plus = q_in.copy()
        q_plus[k] += dq
        D_plus = compute_D_matrix(q_plus)
        dD_dqk = (D_plus - D0) / dq
        for i in range(3):
            for j in range(3):
                gamma[i, j, k] = 0.5 * (
                    dD_dqk[i, j] + dD_dqk[i, j] - dD_dqk[j, i]
                )

    return gamma
