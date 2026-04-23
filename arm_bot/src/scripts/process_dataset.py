import pandas as pd
import re
import os
import sys

# ================================
# Input command line
# ================================
if len(sys.argv) < 2:
    print("Usage: python3 process_dataset.py <path_number>")
    sys.exit(1)

TARGET_PATH = int(sys.argv[1])

# ================================
# Paths
# ================================
log_dir_path = os.path.expanduser('~/Desktop/arm_bot/src/scripts/logs')
dataset_dir_path = os.path.expanduser('~/Desktop/arm_bot/src/scripts/Joint_states')
new_dataset_dir_path = os.path.expanduser('~/Desktop/arm_bot/src/scripts/new_generatedDataset')

os.makedirs(new_dataset_dir_path, exist_ok=True)

print(f"\nProcessing only path: {TARGET_PATH}\n")

# ================================
# Processing loop
# ================================
for filename in os.listdir(log_dir_path):
    match = re.search(r'path_(\d+)_log_(\d+)\.csv$', filename)

    if match and int(match.group(1)) == TARGET_PATH:
        log_file_path = os.path.join(log_dir_path, filename)
        print(f'Processing log file: {filename}')

        df = pd.read_csv(log_file_path)

        # Acceleration
        df['da1'] = df['vel1'].diff().fillna(0).round(6)
        df['da2'] = df['vel2'].diff().fillna(0).round(6)
        df['da3'] = df['vel3'].diff().fillna(0).round(6)

        # Rename columns
        df = df.rename(columns={
            'time_elapsed': 't',
            'pos1': 'dp1',
            'pos2': 'dp2',
            'pos3': 'dp3',
            'vel1': 'dv1',
            'vel2': 'dv2',
            'vel3': 'dv3',
            'torque1': 'tau1',
            'torque2': 'tau2',
            'torque3': 'tau3'
        })

        df = df.T

        output_file_path = os.path.join(
            new_dataset_dir_path,
            f'path_{match.group(1)}_joint_states_{match.group(2)}.csv'
        )

        print(f'Saving processed data to: {output_file_path}')
        df.to_csv(output_file_path, header=False, index=True)

print("\n✅ Processing complete.\n")