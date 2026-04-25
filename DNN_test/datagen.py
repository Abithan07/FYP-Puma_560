import argparse
import numpy as np
import pandas as pd
import os

# ---------------- USER INPUT ----------------
dt = 0.01
possible_T = np.arange(12, 25, 4)
v_max = 2
a_max = 7

parser = argparse.ArgumentParser()
parser.add_argument("path_id", type=int, help="Trajectory file ID, e.g. 601")
parser.add_argument("--seed", type=int, default=None, help="Random seed (optional)")
args = parser.parse_args()

path_id = args.path_id

if args.seed is not None:
    np.random.seed(args.seed)

# ---------------- DIRECTORIES ----------------
base_dir = "/home/priyankan/Desktop/FYP-Puma_560/DNN_test/Data"
summary_dir = "/home/priyankan/Desktop/FYP-Puma_560/DNN_test"

angle_dir = os.path.join(base_dir, "Angles")
xyz_dir   = os.path.join(base_dir, "XYZ")
traj_dir  = os.path.join(base_dir, "Trajectory")

summary_file = os.path.join(summary_dir, "summary.csv")
summary_deg_file = os.path.join(summary_dir, "summary_deg.csv")

for d in [angle_dir, xyz_dir, traj_dir]:
    os.makedirs(d, exist_ok=True)

# ---------------- LOAD SUMMARY ----------------
if os.path.exists(summary_file):
    summary = pd.read_csv(summary_file, header=None, dtype=np.float64).values
else:
    summary = np.empty((0,4))

# ---------------- DH PARAMETERS ----------------
alpha = np.array([0, -np.pi/2, 0])
a_dh  = np.array([0, 0, 0.4318])
d_dh  = np.array([0, 0.2435, -0.0934])

# Fast NumPy DH
def dh_numpy(alpha, a, d, theta):
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)

    return np.array([
        [ct, -st, 0, a],
        [st*ca, ct*ca, -sa, -sa*d],
        [st*sa, ct*sa, ca, ca*d],
        [0, 0, 0, 1]
    ])

# ---------------- TRAJECTORY GENERATION ----------------
q_start = np.radians([0, 45, 135])
q_min = np.radians([-80, 25, 100])
q_max = np.radians([80, 45, 170])

valid = False
max_attempts = 1000
attempts = 0

while not valid and attempts < max_attempts:
    attempts += 1

    q_end = q_min + (q_max - q_min) * np.random.rand(3)
    delta_q = np.abs(q_end - q_start)

    T_vel = 1.875 * np.max(delta_q / v_max)
    T_acc = np.max(np.sqrt(5.77 * delta_q / a_max))
    T_min = max(T_vel, T_acc)

    T_rand = np.random.choice(possible_T)
    T_total = max(T_min, T_rand)

    candidate = np.append(q_end, T_total)

    if summary.shape[0] == 0 or not np.any(np.all(np.abs(summary - candidate) < 1e-4, axis=1)):
        valid = True

if not valid:
    raise RuntimeError("Could not generate a unique trajectory.")

# ---------------- TIME ----------------
t = np.arange(0, T_total + dt, dt)
tau = t / T_total

# ---------------- MINIMUM JERK ----------------
f   = 10*tau**3 - 15*tau**4 + 6*tau**5
fd  = (30*tau**2 - 60*tau**3 + 30*tau**4) / T_total
fdd = (60*tau - 180*tau**2 + 120*tau**3) / T_total**2

# ---------------- TRAJECTORY ----------------
q_traj = np.zeros((len(t), 3))
dq_traj = np.zeros_like(q_traj)
ddq_traj = np.zeros_like(q_traj)

for j in range(3):
    dqj = q_end[j] - q_start[j]
    q_traj[:, j] = q_start[j] + dqj * f
    dq_traj[:, j] = dqj * fd
    ddq_traj[:, j] = dqj * fdd

# ---------------- FORWARD KINEMATICS ----------------
xyz = np.zeros_like(q_traj)

for k in range(len(t)):
    T1 = dh_numpy(alpha[0], a_dh[0], d_dh[0], q_traj[k,0])
    T2 = dh_numpy(alpha[1], a_dh[1], d_dh[1], q_traj[k,1])
    T3 = dh_numpy(alpha[2], a_dh[2], d_dh[2], q_traj[k,2])

    T = T1 @ T2 @ T3
    xyz[k, :] = T[:3, 3]

# ---------------- SAVE FILES ----------------
np.savetxt(os.path.join(angle_dir, f'path_{path_id:03d}_angles.csv'), q_traj, delimiter=',')
np.savetxt(os.path.join(xyz_dir, f'path_{path_id:03d}_xyz.csv'), xyz, delimiter=',')

# Trajectory CSV (cleaner using pandas)
traj_matrix = np.hstack([t.reshape(-1,1), q_traj, dq_traj, ddq_traj])

row_titles = ['t','dp1','dp2','dp3','dv1','dv2','dv3','da1','da2','da3']
df = pd.DataFrame(traj_matrix.T)
df.insert(0, 'var', row_titles)

traj_file = os.path.join(traj_dir, f'path_{path_id:03d}_trajectories.csv')
df.to_csv(traj_file, index=False, header=False)

# ---------------- UPDATE SUMMARY ----------------
summary = np.vstack([summary, candidate])
pd.DataFrame(summary).to_csv(summary_file, index=False, header=False)

# Keep a derived file where columns 1-3 are converted to degrees.
# This is refreshed every time summary.csv is updated.
summary_deg = summary.copy()
summary_deg[:, :3] = np.degrees(summary_deg[:, :3])
pd.DataFrame(summary_deg).to_csv(summary_deg_file, index=False, header=False)

print(f"Trajectory {path_id} generated and saved.")
print(f"Start (deg): {np.degrees(q_start)}")
print(f"End   (deg): {np.degrees(q_end)}")
print(f"T_total (s): {T_total:.4f}")