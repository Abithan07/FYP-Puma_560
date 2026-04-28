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
import pandas as pd
import matplotlib.pyplot as plt

# ============================================================
# USER INPUT
# ============================================================

num_paths = int(input("Enter number of trajectories: "))

# ============================================================
# SETTINGS
# ============================================================

dt = 0.01
possible_T = np.arange(12, 25, 4)   # [12,16,20,24]

path_start_id = 601

# ---------------- Joint Limits ----------------
q1_min_deg = -150
q1_max_deg = 150

q2_min_deg = -215
q2_max_deg = 45

q3_min_deg = -35
q3_max_deg = 215

# ----------------------------------------------

v_max = 2.0
a_max = 7.0

# ============================================================
# OUTPUT DIRECTORIES
# ============================================================

base_dir = r"C:\Users\Priyankan\Desktop\Trajectory Gen\Dataset_04_27_n"

angle_dir = os.path.join(base_dir, "Angles")
xyz_dir   = os.path.join(base_dir, "XYZ")
traj_dir  = os.path.join(base_dir, "Trajectories")

os.makedirs(angle_dir, exist_ok=True)
os.makedirs(xyz_dir, exist_ok=True)
os.makedirs(traj_dir, exist_ok=True)

# ============================================================
# PATH RECORD
# ============================================================

record_dir = r"C:\Users\Priyankan\Desktop\Trajectory Gen\Joint States All"
os.makedirs(record_dir, exist_ok=True)

record_file = os.path.join(record_dir, "all_paths_0.csv")

if os.path.exists(record_file):
    path_record = pd.read_csv(record_file, header=None).values
else:
    path_record = np.empty((0, 5))

# ============================================================
# PUMA DH PARAMETERS
# ============================================================

alpha = np.array([0, -np.pi/2, 0, np.pi/2])
a_dh  = np.array([0, 0, 0.4318, 0])
d_dh  = np.array([0, 0.2435, -0.0934, 0.4331])

T_base = np.eye(4)
T_base[2, 3] = 0.6718

# ============================================================
# START CONFIGURATION
# ============================================================

q_start_deg = np.array([0, 45, 135])
q_start = np.deg2rad(q_start_deg)

q4_deg = 0
q4 = np.deg2rad(q4_deg)

# ============================================================
# PLOTTING
# ============================================================

fig1 = plt.figure(figsize=(8,6))
ax1 = fig1.add_subplot(111, projection='3d')
ax1.set_title("Generated PUMA-560 Paths")
ax1.set_xlabel("X")
ax1.set_ylabel("Y")
ax1.set_zlabel("Z")

fig2 = plt.figure(figsize=(8,6))
ax2 = fig2.add_subplot(111, projection='3d')
ax2.set_title("Generated PUMA-560 End Points")
ax2.set_xlabel("X")
ax2.set_ylabel("Y")
ax2.set_zlabel("Z")

plt.ion()


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

# ============================================================
# TRAJECTORY GENERATION
# ============================================================

for p in range(num_paths):

    print(f"Generating trajectory {p+1}/{num_paths}")

    path_id = path_start_id + p

    valid_path = False

    while not valid_path:

        q_end_deg = np.array([
            np.random.uniform(q1_min_deg, q1_max_deg),
            np.random.uniform(q2_min_deg, q2_max_deg),
            np.random.uniform(q3_min_deg, q3_max_deg)
        ])

        q_end = np.deg2rad(q_end_deg)

        dq = np.abs(q_end - q_start)

        T_vel = np.max(1.875 * dq / v_max)
        T_acc = np.max(np.sqrt(5.77 * dq / a_max))

        T_min = max(T_vel, T_acc)

        T_rand = np.random.choice(possible_T)

        T_total = max(T_min, T_rand)

        candidate = np.array([path_id, *q_end_deg, T_total])

        if len(path_record) == 0:
            valid_path = True
        else:
            diff = np.abs(path_record[:,1:5] - candidate[1:5])

            if np.all(np.any(diff > 1e-4, axis=1)):
                valid_path = True

    # ========================================================
    # TIME VECTOR
    # ========================================================

    t = np.arange(0, T_total + dt, dt)
    N = len(t)

    tau = t / T_total

    # ========================================================
    # MINIMUM JERK PROFILE
    # ========================================================

    f   = 10*tau**3 - 15*tau**4 + 6*tau**5
    fd  = (30*tau**2 - 60*tau**3 + 30*tau**4)/T_total
    fdd = (60*tau - 180*tau**2 + 120*tau**3)/(T_total**2)

    # ========================================================
    # JOINT TRAJECTORY
    # ========================================================

    q   = np.zeros((N,3))
    qd  = np.zeros((N,3))
    qdd = np.zeros((N,3))

    for j in range(3):

        dqj = q_end[j] - q_start[j]

        q[:,j]   = q_start[j] + dqj*f
        qd[:,j]  = dqj*fd
        qdd[:,j] = dqj*fdd

    # ========================================================
    # FORWARD KINEMATICS
    # ========================================================

    xyz = np.zeros((N,3))

    for k in range(N):

        q1, q2, q3 = q[k]

        T01 = T_base @ dh(alpha[0], a_dh[0], d_dh[0], q1)
        T12 = dh(alpha[1], a_dh[1], d_dh[1], q2)
        T23 = dh(alpha[2], a_dh[2], d_dh[2], q3)
        T34 = dh(alpha[3], a_dh[3], d_dh[3], q4)

        T04 = T01 @ T12 @ T23 @ T34

        xyz[k,:] = T04[:3,3]

    # ========================================================
    # SAVE FILES
    # ========================================================

    pd.DataFrame(q).to_csv(
        os.path.join(angle_dir, f"path_{path_id:03d}_angles.csv"),
        header=False,
        index=False
    )

    pd.DataFrame(xyz).to_csv(
        os.path.join(xyz_dir, f"path_{path_id:03d}_xyz.csv"),
        header=False,
        index=False
    )

    traj = np.column_stack((t, q, qd, qdd))

    pd.DataFrame(traj).to_csv(
        os.path.join(traj_dir, f"path_{path_id:03d}_traj.csv"),
        header=False,
        index=False
    )

    # ========================================================
    # UPDATE RECORD FILE
    # ========================================================

    new_record = np.array([[path_id, *q_end_deg, T_total]])
    path_record = np.vstack((path_record, new_record))

    pd.DataFrame(path_record).to_csv(
        record_file,
        header=False,
        index=False
    )

    # ========================================================
    # LIVE PLOT
    # ========================================================

    color = np.random.rand(3,)

    ax1.plot(xyz[:,0], xyz[:,1], xyz[:,2], color=color)
    ax2.scatter(xyz[-1,0], xyz[-1,1], xyz[-1,2], color=color)

    plt.draw()
    plt.pause(0.01)

print("All trajectories generated successfully.")

plt.ioff()
plt.show()