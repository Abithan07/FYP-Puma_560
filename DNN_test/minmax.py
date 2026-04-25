import pandas as pd
import numpy as np

file_path = "/home/priyankan/Desktop/FYP-Puma_560/DNN_test/Data/path_461_joint_states.csv"

# CSV format: first column = variable name (t, dp1, dp2, dp3, ...)
df = pd.read_csv(file_path, header=None)
df = df.set_index(0)

for joint in ["dp1", "dp2", "dp3"]:
    # vals = pd.to_numeric(df.loc[joint, 1:], errors="coerce").dropna()
    # print(f"{joint}: min = {vals.min():.9f} rad, max = {vals.max():.9f} rad")

    vals_rad = pd.to_numeric(df.loc[joint, 1:], errors="coerce").dropna()
    vals_deg = np.degrees(vals_rad)
    print(f"{joint}: min = {vals_deg.min():.2f} deg, max = {vals_deg.max():.2f} deg")