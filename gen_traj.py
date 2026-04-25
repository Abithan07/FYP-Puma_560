import numpy as np
import os
import csv
import argparse

# ===================== SETTINGS =====================

dt = 0.001
possible_T = np.arange(12, 25, 4)

v_max = 2
a_max = 7

# ===================== JOINT LIMITS =====================

q_start = np.deg2rad([0, 45, 135])

q1_min, q1_max = np.deg2rad([-100, 100])
q2_min, q2_max = np.deg2rad([-15, 45])
q3_min, q3_max = np.deg2rad([65, 205])

# ===================== OUTPUT =====================

base_dir = "Dataset"
traj_dir = os.path.join(base_dir, "Trajectories")
os.makedirs(traj_dir, exist_ok=True)

record_file = os.path.join(base_dir, "trajectories.csv")

# ===================== LOAD EXISTING RECORD =====================

if os.path.exists(record_file):
    # path_records = np.loadtxt(record_file, delimiter=",")
    path_records = np.genfromtxt(record_file, delimiter=",", skip_header=1)
    if path_records.ndim == 1:
        path_records = path_records.reshape(1, -1)
else:
    path_records = np.empty((0, 8))
    


# ===================== MINIMUM JERK =====================

def min_jerk(t, T):
    tau = t / T

    f = 10*tau**3 - 15*tau**4 + 6*tau**5
    fd = (30*tau**2 - 60*tau**3 + 30*tau**4) / T
    fdd = (60*tau - 180*tau**2 + 120*tau**3) / T**2

    return f, fd, fdd


# ===================== GENERATOR =====================

def generate_trajectories(start_id, num_paths):
    global path_records

    for i in range(num_paths):

        path_id = start_id + i
        print(f"Generating trajectory {path_id}")

        # ---------- RANDOM UNIQUE END ----------
        while True:
            q_end = np.array([
                np.random.uniform(q1_min, q1_max),
                np.random.uniform(q2_min, q2_max),
                np.random.uniform(q3_min, q3_max)
            ])

            dq = np.abs(q_end - q_start)

            T_vel = np.max(1.875 * dq / v_max)
            T_acc = np.max(np.sqrt(5.77 * dq / a_max))

            T_min = max(T_vel, T_acc)
            T_rand = np.random.choice(possible_T)

            T_total = max(T_min, T_rand)

            candidate = np.concatenate((q_end, [T_total]))

            if path_records.shape[0] == 0:
                break

            # Compare only relevant columns: q_rad + T_total
            existing = path_records[:, [1, 2, 3, 7]]

            diff = np.abs(existing - candidate)

            if np.all(np.any(diff > 1e-4, axis=1)):
                break

        # ---------- TIME ----------
        t = np.arange(0, T_total + dt, dt)

        # ---------- PROFILE ----------
        f, fd, fdd = min_jerk(t, T_total)

        # ---------- TRAJECTORY ----------
        dq = q_end - q_start

        q = q_start + np.outer(f, dq)
        qd = np.outer(fd, dq)
        qdd = np.outer(fdd, dq)

        # ---------- STACK COLUMN-WISE (ROW FORMAT) ----------
        traj = np.vstack([
            t,
            q[:, 0], q[:, 1], q[:, 2],
            qd[:, 0], qd[:, 1], qd[:, 2],
            qdd[:, 0], qdd[:, 1], qdd[:, 2]
        ])

        q_end_deg = np.rad2deg(q_end)

        # ---------- SAVE ----------
        filename = os.path.join(traj_dir, f"path_{path_id:03d}_trajectory.csv")
        # np.savetxt(filename, traj, delimiter=",")
        labels = np.array([
            "t","dp1","dp2","dp3","dv1","dv2","dv3","da1","da2","da3"
        ]).reshape(-1,1)

        traj_str = []

        for i, row in enumerate(traj):
            if i == 0:
                # time row → 3 decimal places
                formatted = np.char.mod('%.3f', row)
            else:
                # all others → 8 decimal places
                formatted = np.char.mod('%.8f', row)
            
            traj_str.append(formatted)

        traj_str = np.array(traj_str)

        traj_with_labels = np.hstack((labels, traj_str))

        np.savetxt(
            filename,
            traj_with_labels,
            delimiter=",",
            fmt="%s"
        )

        # ---------- UPDATE RECORD ----------
        new_record = np.concatenate(([path_id], q_end, q_end_deg, [T_total]))
        path_records = np.vstack([path_records, new_record])

        header = "path_id,q1_rad,q2_rad,q3_rad,q1_deg,q2_deg,q3_deg,T_total"
        # np.savetxt(record_file, path_records, delimiter=",", header=header, comments='')
        fmt = ["%d", "%.8f", "%.8f", "%.8f", "%.6f", "%.6f", "%.6f", "%.6f"]

        np.savetxt(
            record_file,
            path_records,
            delimiter=",",
            fmt=fmt,
            header=header,
            comments=''
        )


# ===================== MAIN =====================

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("start_id", type=int, help="Starting trajectory ID")
    parser.add_argument("num_paths", type=int, help="Number of trajectories")

    args = parser.parse_args()

    generate_trajectories(args.start_id, args.num_paths)

    print("All trajectories generated successfully.")

"""
Usage:
    python gen_traj.py <start_id> <num_paths>
    Example: python3 gen_traj.py 1 10
This will generate 10 trajectories with IDs from 1 to 10, and save them in the Dataset/Trajectories directory. The metadata will be recorded in Dataset/trajectories.csv.
"""