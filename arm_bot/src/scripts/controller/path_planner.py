# ============================================================
# PUMA-560 PATH + TRAJECTORY DATASET GENERATOR (PYTHON)
# ------------------------------------------------------------
# Generates:
# 1) Joint Angles
# 2) End Effector XYZ
# 3) Full trajectory (q, qd, qdd)
#
# Constraints:
# |velocity| <= 2 rad/s
# |acceleration| <= 7 rad/s²
#
# ============================================================

import os
import numpy as np

# ============================================================
# PUMA DH PARAMETERS
# ============================================================

alpha = np.array([0, -np.pi/2, 0, np.pi/2])
a_dh  = np.array([0, 0, 0.4318, 0])
d_dh  = np.array([0, 0.2435, -0.0934, 0.4331])

T_base = np.eye(4)
T_base[2, 3] = 0.6718

q4_deg = 0
q4 = np.deg2rad(q4_deg)



# ============================================================
# Utility: generate a min-jerk joint trajectory between two configs
# ============================================================
def generate_min_jerk_trajectory(q_start, q_end, T_total, dt=0.01):
    """Return t, q, qd, qdd, xyz for a min-jerk joint-space trajectory.

    q_start, q_end are in radians (3,)
    """
    t = np.arange(0, T_total + dt, dt)
    N = len(t)
    tau = t / T_total
    f   = 10*tau**3 - 15*tau**4 + 6*tau**5
    fd  = (30*tau**2 - 60*tau**3 + 30*tau**4)/T_total
    fdd = (60*tau - 180*tau**2 + 120*tau**3)/(T_total**2)

    q   = np.zeros((N,3))
    qd  = np.zeros((N,3))
    qdd = np.zeros((N,3))
    for j in range(3):
        dqj = q_end[j] - q_start[j]
        q[:,j]   = q_start[j] + dqj*f
        qd[:,j]  = dqj*fd
        qdd[:,j] = dqj*fdd

    # forward kinematics for tip
    xyz = np.zeros((N,3))
    for k in range(N):
        q1, q2, q3 = q[k]
        T01 = T_base @ dh(alpha[0], a_dh[0], d_dh[0], q1)
        T12 = dh(alpha[1], a_dh[1], d_dh[1], q2)
        T23 = dh(alpha[2], a_dh[2], d_dh[2], q3)
        T34 = dh(alpha[3], a_dh[3], d_dh[3], q4)
        T04 = T01 @ T12 @ T23 @ T34
        xyz[k,:] = T04[:3,3]

    return t, q, qd, qdd, xyz


# ============================================================
# DH FUNCTION
# ============================================================

def dh(alpha, a, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta)*np.cos(alpha),
         np.cos(theta)*np.cos(alpha),
         -np.sin(alpha),
         -np.sin(alpha)*d],

        [np.sin(theta)*np.sin(alpha),
         np.cos(theta)*np.sin(alpha),
         np.cos(alpha),
         np.cos(alpha)*d],

        [0,0,0,1]
    ])