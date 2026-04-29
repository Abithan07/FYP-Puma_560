#!/usr/bin/env python3
"""
Plot Comparison Script
Compares injected positions/velocities/torques with sensor readings from continuous logger.
Automatically filters outliers by comparing consecutive values.

Usage:
    python3 plot_comparison.py <dataset_file> <continuous_log_file> [pos_threshold] [tau_threshold] [--allinone]
    
Example:
    python3 plot_comparison.py src/scripts/move_q2_traj_joint_states.csv logs/continuous_log_1_20260104_173527.csv

With custom thresholds:
    python3 plot_comparison.py dataset.csv log.csv 0.5 50.0

All-in-one mode:
    python3 plot_comparison.py dataset.csv log.csv --allinone

Generates a single high-resolution PNG file:
  Grid mode (default, 2x3 layout):
    - trajectory_comparison.png
      Top row: Expected/Injected values (Position, Velocity, Torque)
      Bottom row: Sensed values (Position, Velocity, Torque)
  All-in-one mode (3 rows, overlaid expected vs sensed):
    - trajectory_comparison_allinone.png
      Each row overlays dataset (solid) and sensed (dash-dot) with unique color per joint per value

Outlier Filtering:
    - Position changes > 1.0 rad (default) are filtered
    - Torque changes > 100.0 Nm (default) are filtered
    - Outliers are replaced with previous valid value
"""

import os
os.environ['NUMPY_EXPERIMENTAL_ARRAY_FUNCTION'] = '0'

import sys
import csv
import numpy as np
import warnings
warnings.filterwarnings('ignore')

import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from pathlib import Path
import re

# Default outlier filtering thresholds
POS_THRESHOLD = 1.0  # radians
TAU_THRESHOLD = 25.0  # Nm

# Plot mode: 'grid' for 2x3 layout, 'allinone' for overlaid comparison
PLOT_MODE = 'grid'  # Change to 'allinone' for all-in-one plotting

def remove_outliers(time, pos1, pos2, pos3, vel1, vel2, vel3, tau1, tau2, tau3):
    """
    Remove outliers by comparing consecutive values.
    If the change exceeds threshold, use the previous value instead.
    
    Args:
        time: Time array
        pos1, pos2, pos3: Position arrays (radians)
        vel1, vel2, vel3: Velocity arrays (rad/s)
        tau1, tau2, tau3: Torque arrays (Nm)
    
    Returns:
        Filtered arrays with outliers removed
    """
    if len(time) == 0:
        return time, pos1, pos2, pos3, vel1, vel2, vel3, tau1, tau2, tau3
    
    # Convert to numpy arrays
    time = np.array(time)
    pos1 = np.array(pos1)
    pos2 = np.array(pos2)
    pos3 = np.array(pos3)
    vel1 = np.array(vel1)
    vel2 = np.array(vel2)
    vel3 = np.array(vel3)
    tau1 = np.array(tau1)
    tau2 = np.array(tau2)
    tau3 = np.array(tau3)
    
    outlier_count = 0
    
    # Process each array
    for i in range(1, len(time)):
        # Check positions
        if abs(pos1[i] - pos1[i-1]) > POS_THRESHOLD:
            pos1[i] = pos1[i-1]
            outlier_count += 1
        if abs(pos2[i] - pos2[i-1]) > POS_THRESHOLD:
            pos2[i] = pos2[i-1]
            outlier_count += 1
        if abs(pos3[i] - pos3[i-1]) > POS_THRESHOLD:
            pos3[i] = pos3[i-1]
            outlier_count += 1
        
        # Check torques
        if abs(tau1[i] - tau1[i-1]) > TAU_THRESHOLD:
            tau1[i] = tau1[i-1]
            outlier_count += 1
        if abs(tau2[i] - tau2[i-1]) > TAU_THRESHOLD:
            tau2[i] = tau2[i-1]
            outlier_count += 1
        if abs(tau3[i] - tau3[i-1]) > TAU_THRESHOLD:
            tau3[i] = tau3[i-1]
            outlier_count += 1
    
    if outlier_count > 0:
        print(f'  - Filtered {outlier_count} outlier values (pos_threshold={POS_THRESHOLD:.2f} rad, tau_threshold={TAU_THRESHOLD:.1f} Nm)')
    
    return time, pos1, pos2, pos3, vel1, vel2, vel3, tau1, tau2, tau3


def load_dataset_file(filepath):
    """
    Load the dataset CSV file (move_q2_traj format).
    Format: First column is label, rest are values at different time steps.
    """
    data = {}
    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if row:
                key = row[0]
                values = [float(val) for val in row[1:]]
                data[key] = np.array(values)
    
    # Extract relevant arrays
    time = data['t']
    pos1 = data['dp1']
    pos2 = data['dp2']
    pos3 = data['dp3']
    vel1 = data['dv1']
    vel2 = data['dv2']
    vel3 = data['dv3']
    tau1 = data['tau1']
    tau2 = data['tau2']
    tau3 = data['tau3']
    
    # return time, pos1, pos2, pos3, vel1, vel2, vel3, tau1, tau2, tau3
    return remove_outliers(time, pos1, pos2, pos3, vel1, vel2, vel3, tau1, tau2, tau3)



def load_continuous_log(filepath):
    """
    Load the continuous logger CSV file.
    Supports three formats:
    1. Latest format: time_elapsed, pos1, pos2, pos3, vel1, vel2, vel3, torque1, torque2, torque3
    2. Previous format: time_elapsed, pos1, pos2, pos3, torque1, torque2, torque3
    3. Old format: timestamp, time_elapsed, des_pos_1, des_pos_2, des_pos_3, act_pos_1, act_pos_2, act_pos_3, ...
    """
    time = []
    pos1, pos2, pos3 = [], [], []
    vel1, vel2, vel3 = [], [], []
    tau1, tau2, tau3 = [], [], []
    
    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)  # Read header
        
        # Detect format based on header
        if 'timestamp' in header[0].lower():
            # Old format with timestamp column
            time_col = 1
            pos_cols = [5, 6, 7]  # act_pos_1, act_pos_2, act_pos_3
            vel_cols = None  # No velocities in old format
            tau_cols = [16, 17, 18]  # act_tau_1, act_tau_2, act_tau_3
        elif 'vel1' in header or (len(header) >= 10 and 'vel' in header[4].lower()):
            # Latest format with velocities (10 columns)
            time_col = 0
            pos_cols = [1, 2, 3]
            vel_cols = [4, 5, 6]
            tau_cols = [7, 8, 9]
        else:
            # Previous format without velocities (7 columns)
            time_col = 0
            pos_cols = [1, 2, 3]
            vel_cols = None
            tau_cols = [4, 5, 6]
        
        for row in reader:
            if row and len(row) > max(tau_cols):
                try:
                    time.append(float(row[time_col]))
                    pos1.append(float(row[pos_cols[0]]))
                    pos2.append(float(row[pos_cols[1]]))
                    pos3.append(float(row[pos_cols[2]]))
                    
                    # Load velocities if available
                    if vel_cols:
                        vel1.append(float(row[vel_cols[0]]))
                        vel2.append(float(row[vel_cols[1]]))
                        vel3.append(float(row[vel_cols[2]]))
                    else:
                        vel1.append(0.0)
                        vel2.append(0.0)
                        vel3.append(0.0)
                    
                    tau1.append(float(row[tau_cols[0]]))
                    tau2.append(float(row[tau_cols[1]]))
                    tau3.append(float(row[tau_cols[2]]))
                except ValueError:
                    continue  # Skip malformed rows
    
    # Filter outliers before returning
    return remove_outliers(time, pos1, pos2, pos3, vel1, vel2, vel3, tau1, tau2, tau3)
    # return time, pos1, pos2, pos3, vel1, vel2, vel3, tau1, tau2, tau3


def plot_all_in_one(dataset_time, dataset_pos1, dataset_pos2, dataset_pos3,
                    dataset_vel1, dataset_vel2, dataset_vel3,
                    dataset_tau1, dataset_tau2, dataset_tau3,
                    sensor_time, sensor_pos1, sensor_pos2, sensor_pos3,
                    sensor_vel1, sensor_vel2, sensor_vel3,
                    sensor_tau1, sensor_tau2, sensor_tau3,
                    dataset_filename='', log_filename='',
                    output_file='trajectory_comparison_allinone.png'):
    """
    Plot ALL values (position, velocity, torque) on a single timeline.
    Top subplot: Torque (left axis) + Position (right axis) overlaid.
    Bottom subplot: Velocity plotted separately for clarity.
    Solid lines = expected/dataset, dash-dot = sensed/logged.
    Unique color per (joint, value-type) pair; same color for expected & sensed of
    the same pair.
    """
    # fig, (ax_tau, ax_vel) = plt.subplots(2, 1, figsize=(24, 26),
    #                                       height_ratios=[3, 2],
    #                                       gridspec_kw={'hspace': 0.25})
    fig, (ax_tau, ax_vel) = plt.subplots(2, 1, figsize=(24, 26),
                                      gridspec_kw={'height_ratios': [3, 2], 'hspace': 0.25})

    # Create twin axis for Position on the top subplot (right side)
    ax_pos = ax_tau.twinx()

    # --- Y-AXIS LABEL COLORS (one color family per value type) ---
    tau_axis_color = '#B71C1C'   # Deep Red    — for Torque axis (LEFT)
    pos_axis_color = '#0D47A1'   # Dark Blue   — for Position axis (RIGHT 1)
    vel_axis_color = '#1B5E20'   # Dark Green  — for Velocity axis (RIGHT 2)

    # --- 3 base colors, 3 shades each (light/medium/dark per joint) ---
    # Torque: Red family (LEFT axis)
    #   J1 = lightest, J2 = medium, J3 = darkest
    color_tau_j1 = '#EF5350'  # Light Red
    color_tau_j2 = '#C62828'  # Medium Red
    color_tau_j3 = '#7F0000'  # Dark Red
    # Position: Blue family (RIGHT 1)
    color_pos_j1 = '#42A5F5'  # Light Blue
    color_pos_j2 = '#1565C0'  # Medium Blue
    color_pos_j3 = '#0D47A1'  # Dark Blue
    # Velocity: Green family (RIGHT 2)
    color_vel_j1 = '#66BB6A'  # Light Green
    color_vel_j2 = '#2E7D32'  # Medium Green
    color_vel_j3 = "#144617"  # Dark Green

    # Line styles (increased contrast)
    solid_lw = 4       # linewidth for dataset (solid)
    dashdot_lw = 3.5     # linewidth for sensed (dashdot)
    solid_alpha = 0.90
    dashdot_alpha = 0.70

    # Convert positions to degrees
    dataset_pos1_deg = np.rad2deg(dataset_pos1)
    dataset_pos2_deg = np.rad2deg(dataset_pos2)
    dataset_pos3_deg = np.rad2deg(dataset_pos3)
    sensor_pos1_deg = np.rad2deg(sensor_pos1)
    sensor_pos2_deg = np.rad2deg(sensor_pos2)
    sensor_pos3_deg = np.rad2deg(sensor_pos3)

    # Convert velocities to degrees/s
    dataset_vel1_deg = np.rad2deg(dataset_vel1)
    dataset_vel2_deg = np.rad2deg(dataset_vel2)
    dataset_vel3_deg = np.rad2deg(dataset_vel3)
    sensor_vel1_deg = np.rad2deg(sensor_vel1)
    sensor_vel2_deg = np.rad2deg(sensor_vel2)
    sensor_vel3_deg = np.rad2deg(sensor_vel3)

    # Collect all handles/labels for a unified legend
    handles = []
    labels = []

    # --- TORQUE lines (on ax_tau, LEFT y-axis) ---
    for data_t, data_v, color, joint, style, alpha, lw, source in [
        (dataset_time, dataset_tau1, color_tau_j1, 'tau1', '-',  solid_alpha, solid_lw, 'Exp'),
        (dataset_time, dataset_tau2, color_tau_j2, 'tau2', '-',  solid_alpha, solid_lw, 'Exp'),
        (dataset_time, dataset_tau3, color_tau_j3, 'tau3', '-',  solid_alpha, solid_lw, 'Exp'),
        (sensor_time,  sensor_tau1,  color_tau_j1, 'tau1', '-.', dashdot_alpha, dashdot_lw, 'Sns'),
        (sensor_time,  sensor_tau2,  color_tau_j2, 'tau2', '-.', dashdot_alpha, dashdot_lw, 'Sns'),
        (sensor_time,  sensor_tau3,  color_tau_j3, 'tau3', '-.', dashdot_alpha, dashdot_lw, 'Sns'),
    ]:
        h, = ax_tau.plot(data_t, data_v, color=color, linestyle=style,
                         linewidth=lw, alpha=alpha)
        handles.append(h)
        labels.append(f'{joint} {source}')

    # --- POSITION lines (on ax_pos, first RIGHT y-axis) ---
    for data_t, data_v, color, joint, style, alpha, lw, source in [
        (dataset_time, dataset_pos1_deg, color_pos_j1, 'dp1', '-',  solid_alpha, solid_lw, 'Exp'),
        (dataset_time, dataset_pos2_deg, color_pos_j2, 'dp2', '-',  solid_alpha, solid_lw, 'Exp'),
        (dataset_time, dataset_pos3_deg, color_pos_j3, 'dp3', '-',  solid_alpha, solid_lw, 'Exp'),
        (sensor_time,  sensor_pos1_deg,  color_pos_j1, 'dp1', '-.', dashdot_alpha, dashdot_lw, 'Sns'),
        (sensor_time,  sensor_pos2_deg,  color_pos_j2, 'dp2', '-.', dashdot_alpha, dashdot_lw, 'Sns'),
        (sensor_time,  sensor_pos3_deg,  color_pos_j3, 'dp3', '-.', dashdot_alpha, dashdot_lw, 'Sns'),
    ]:
        h, = ax_pos.plot(data_t, data_v, color=color, linestyle=style,
                         linewidth=lw, alpha=alpha)
        handles.append(h)
        labels.append(f'{joint} {source}')

    # --- VELOCITY lines (on ax_vel, second RIGHT y-axis, outermost) ---
    for data_t, data_v, color, joint, style, alpha, lw, source in [
        (dataset_time, dataset_vel1_deg, color_vel_j1, 'dv1', '-',  solid_alpha, solid_lw, 'Exp'),
        (dataset_time, dataset_vel2_deg, color_vel_j2, 'dv2', '-',  solid_alpha, solid_lw, 'Exp'),
        (dataset_time, dataset_vel3_deg, color_vel_j3, 'dv3', '-',  solid_alpha, solid_lw, 'Exp'),
        (sensor_time,  sensor_vel1_deg,  color_vel_j1, 'dv1', '-.', dashdot_alpha, dashdot_lw, 'Sns'),
        (sensor_time,  sensor_vel2_deg,  color_vel_j2, 'dv2', '-.', dashdot_alpha, dashdot_lw, 'Sns'),
        (sensor_time,  sensor_vel3_deg,  color_vel_j3, 'dv3', '-.', dashdot_alpha, dashdot_lw, 'Sns'),
    ]:
        h, = ax_vel.plot(data_t, data_v, color=color, linestyle=style,
                         linewidth=lw, alpha=alpha)
        handles.append(h)
        labels.append(f'{joint} {source}')

    # --- Color-coded Y-axis labels and tick labels ---
    # TOP SUBPLOT — LEFT: Torque
    ax_tau.set_ylabel('Torque (Nm)', fontsize=20, fontweight='bold', color=tau_axis_color)
    ax_tau.tick_params(axis='y', labelcolor=tau_axis_color, labelsize=20)
    ax_tau.spines['left'].set_color(tau_axis_color)
    ax_tau.spines['left'].set_linewidth(2)

    # TOP SUBPLOT — RIGHT: Position
    ax_pos.set_ylabel('Position (degrees)', fontsize=20, fontweight='bold', color=pos_axis_color)
    ax_pos.tick_params(axis='y', labelcolor=pos_axis_color, labelsize=20)
    ax_pos.spines['right'].set_color(pos_axis_color)
    ax_pos.spines['right'].set_linewidth(2)

    # BOTTOM SUBPLOT — Velocity
    ax_vel.set_ylabel('Angular Velocity (degrees/s)', fontsize=20, fontweight='bold', color=vel_axis_color)
    ax_vel.tick_params(axis='y', labelcolor=vel_axis_color, labelsize=20)
    ax_vel.spines['left'].set_color(vel_axis_color)
    ax_vel.spines['left'].set_linewidth(2)

    # --- Compute symmetric limits ---
    all_pos = np.concatenate([dataset_pos1_deg, dataset_pos2_deg, dataset_pos3_deg,
                              sensor_pos1_deg, sensor_pos2_deg, sensor_pos3_deg])
    all_vel = np.concatenate([dataset_vel1_deg, dataset_vel2_deg, dataset_vel3_deg,
                              sensor_vel1_deg, sensor_vel2_deg, sensor_vel3_deg])
    all_tau = np.concatenate([dataset_tau1, dataset_tau2, dataset_tau3,
                              sensor_tau1, sensor_tau2, sensor_tau3])

    def symmetric_limit(data, step):
        """Return symmetric y-limits snapped to step increments, with 0 centered."""
        abs_max = max(abs(np.min(data)), abs(np.max(data)))
        lim = int(np.ceil(abs_max / step)) * step + step  # one extra step of padding
        return -lim, lim

    pos_lo, pos_hi = symmetric_limit(all_pos, 10)
    vel_lo, vel_hi = symmetric_limit(all_vel, 10)
    tau_lo, tau_hi = symmetric_limit(all_tau, 2)

    ax_tau.set_ylim(tau_lo, tau_hi)
    ax_pos.set_ylim(pos_lo, pos_hi)
    ax_vel.set_ylim(vel_lo, vel_hi)

    # Set y-axis ticks with requested step sizes
    from matplotlib.ticker import MultipleLocator
    ax_pos.yaxis.set_major_locator(MultipleLocator(20))   # step 20 for position
    ax_vel.yaxis.set_major_locator(MultipleLocator(20))   # step 20 for velocity
    ax_tau.yaxis.set_major_locator(MultipleLocator(5))    # step 5 for torque

    # --- X-axis: timestep ticks at 0.1s intervals ---
    all_times = np.concatenate([dataset_time, sensor_time])
    t_max = np.ceil(np.max(all_times) * 10) / 10  # round up to nearest 0.1

    # Top subplot x-axis
    ax_tau.set_xlim(0, t_max)
    ax_tau.set_xticks(np.arange(0, t_max + 0.05, 0.1))
    ax_tau.tick_params(axis='x', labelsize=16, rotation=45)

    # Bottom subplot x-axis
    ax_vel.set_xlim(0, t_max)
    ax_vel.set_xticks(np.arange(0, t_max + 0.05, 0.1))
    ax_vel.set_xlabel('Time (s)', fontsize=18, fontweight='bold')
    ax_vel.tick_params(axis='x', labelsize=16, rotation=45)

    # Draw zero-lines for reference
    ax_tau.axhline(y=0, color='black', linewidth=0.8, linestyle='-', alpha=0.4)
    ax_vel.axhline(y=0, color='black', linewidth=0.8, linestyle='-', alpha=0.4)

    # Light background tint for better contrast
    ax_tau.set_facecolor('#F0F0F0')
    ax_vel.set_facecolor('#F0F0F0')

    # Grid
    ax_tau.grid(True, alpha=0.35, linestyle='--', color="#202020")
    ax_vel.grid(True, alpha=0.35, linestyle='--', color="#202020")

    # --- Unified legend (combine handles from both subplots) ---
    ax_vel.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, -0.18),
                  fontsize=24, ncol=6, framealpha=0.95, edgecolor='gray',
                  title='Solid (\u2014) = Expected/Dataset    Dash-dot (\u2013\u00b7) = Sensed/Logged',
                  title_fontsize=24)

    # Subplot titles
    ax_tau.set_title('Torque (red) | Position (blue)', fontsize=20, fontweight='bold', pad=10)
    ax_vel.set_title('Angular Velocity (green)', fontsize=20, fontweight='bold', pad=10)

    # Main title
    fig.suptitle(f'All-In-One Timeline\n'
                 f'Dataset: {dataset_filename}    Log: {log_filename}',
                 fontsize=24, fontweight='bold', y=0.995)

    plt.tight_layout(rect=[0, 0.07, 1.0, 0.97])
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f'Saved: {output_file}')
    plt.close()


def plot_all_comparisons(dataset_time, dataset_pos1, dataset_pos2, dataset_pos3,
                         dataset_vel1, dataset_vel2, dataset_vel3,
                         dataset_tau1, dataset_tau2, dataset_tau3,
                         sensor_time, sensor_pos1, sensor_pos2, sensor_pos3,
                         sensor_vel1, sensor_vel2, sensor_vel3,
                         sensor_tau1, sensor_tau2, sensor_tau3,
                         dataset_filename='', log_filename='',
                         output_file='trajectory_comparison.png'):
    """
    Plot all comparisons in a single high-resolution image with 2 rows x 3 columns:
    Top row: Expected/Injected values (Position, Velocity, Torque)
    Bottom row: Sensed values (Position, Velocity, Torque)
    """
    # Create figure with 2 rows and 3 columns
    fig, axes = plt.subplots(2, 3, figsize=(24, 12))
    
    # Convert positions to degrees for better readability
    dataset_pos1_deg = np.rad2deg(dataset_pos1)
    dataset_pos2_deg = np.rad2deg(dataset_pos2)
    dataset_pos3_deg = np.rad2deg(dataset_pos3)
    sensor_pos1_deg = np.rad2deg(sensor_pos1)
    sensor_pos2_deg = np.rad2deg(sensor_pos2)
    sensor_pos3_deg = np.rad2deg(sensor_pos3)
    
    # Convert velocities to degrees/s for better readability
    dataset_vel1_deg = np.rad2deg(dataset_vel1)
    dataset_vel2_deg = np.rad2deg(dataset_vel2)
    dataset_vel3_deg = np.rad2deg(dataset_vel3)
    sensor_vel1_deg = np.rad2deg(sensor_vel1)
    sensor_vel2_deg = np.rad2deg(sensor_vel2)
    sensor_vel3_deg = np.rad2deg(sensor_vel3)
    
    # TOP ROW: EXPECTED/INJECTED VALUES
    
    # Top Left: Expected Positions
    axes[0, 0].plot(dataset_time, dataset_pos1_deg, 'r-', linewidth=1.5, label='Joint 1', alpha=0.8)
    axes[0, 0].plot(dataset_time, dataset_pos2_deg, 'g-', linewidth=1.5, label='Joint 2', alpha=0.8)
    axes[0, 0].plot(dataset_time, dataset_pos3_deg, 'b-', linewidth=1.5, label='Joint 3', alpha=0.8)
    axes[0, 0].set_ylabel('Position (degrees)', fontsize=13, fontweight='bold')
    axes[0, 0].set_title('Expected Positions (From Dataset)', fontsize=15, fontweight='bold')
    axes[0, 0].legend(loc='upper right', fontsize=11)
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].set_xlim(left=0)
    
    # Top Center: Expected Velocities
    axes[0, 1].plot(dataset_time, dataset_vel1_deg, 'r-', linewidth=1.5, label='Joint 1', alpha=0.8)
    axes[0, 1].plot(dataset_time, dataset_vel2_deg, 'g-', linewidth=1.5, label='Joint 2', alpha=0.8)
    axes[0, 1].plot(dataset_time, dataset_vel3_deg, 'b-', linewidth=1.5, label='Joint 3', alpha=0.8)
    axes[0, 1].set_ylabel('Velocity (degrees/s)', fontsize=13, fontweight='bold')
    axes[0, 1].set_title('Expected Velocities (From Dataset)', fontsize=15, fontweight='bold')
    axes[0, 1].legend(loc='upper right', fontsize=11)
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].set_xlim(left=0)
    
    # Top Right: Injected Torques
    axes[0, 2].plot(dataset_time, dataset_tau1, 'r-', linewidth=1.5, label='Joint 1', alpha=0.8)
    axes[0, 2].plot(dataset_time, dataset_tau2, 'g-', linewidth=1.5, label='Joint 2', alpha=0.8)
    axes[0, 2].plot(dataset_time, dataset_tau3, 'b-', linewidth=1.5, label='Joint 3', alpha=0.8)
    axes[0, 2].set_ylabel('Torque (Nm)', fontsize=13, fontweight='bold')
    axes[0, 2].set_title('Injected Torques (From Dataset)', fontsize=15, fontweight='bold')
    axes[0, 2].legend(loc='upper right', fontsize=11)
    axes[0, 2].grid(True, alpha=0.3)
    axes[0, 2].set_xlim(left=0)
    
    # BOTTOM ROW: SENSED VALUES
    
    # Bottom Left: Sensed Positions
    axes[1, 0].plot(sensor_time, sensor_pos1_deg, 'r-', linewidth=1.5, label='Joint 1', alpha=0.8)
    axes[1, 0].plot(sensor_time, sensor_pos2_deg, 'g-', linewidth=1.5, label='Joint 2', alpha=0.8)
    axes[1, 0].plot(sensor_time, sensor_pos3_deg, 'b-', linewidth=1.5, label='Joint 3', alpha=0.8)
    axes[1, 0].set_xlabel('Time (s)', fontsize=13, fontweight='bold')
    axes[1, 0].set_ylabel('Position (degrees)', fontsize=13, fontweight='bold')
    axes[1, 0].set_title('Sensed Positions (From Logger)', fontsize=15, fontweight='bold')
    axes[1, 0].legend(loc='upper right', fontsize=11)
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].set_xlim(left=0)
    
    # Bottom Center: Sensed Velocities
    axes[1, 1].plot(sensor_time, sensor_vel1_deg, 'r-', linewidth=1.5, label='Joint 1', alpha=0.8)
    axes[1, 1].plot(sensor_time, sensor_vel2_deg, 'g-', linewidth=1.5, label='Joint 2', alpha=0.8)
    axes[1, 1].plot(sensor_time, sensor_vel3_deg, 'b-', linewidth=1.5, label='Joint 3', alpha=0.8)
    axes[1, 1].set_xlabel('Time (s)', fontsize=13, fontweight='bold')
    axes[1, 1].set_ylabel('Velocity (degrees/s)', fontsize=13, fontweight='bold')
    axes[1, 1].set_title('Sensed Velocities (From Logger)', fontsize=15, fontweight='bold')
    axes[1, 1].legend(loc='upper right', fontsize=11)
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].set_xlim(left=0)
    
    # Bottom Right: Sensor Torques
    axes[1, 2].plot(sensor_time, sensor_tau1, 'r-', linewidth=1.5, label='Joint 1', alpha=0.8)
    axes[1, 2].plot(sensor_time, sensor_tau2, 'g-', linewidth=1.5, label='Joint 2', alpha=0.8)
    axes[1, 2].plot(sensor_time, sensor_tau3, 'b-', linewidth=1.5, label='Joint 3', alpha=0.8)
    axes[1, 2].set_xlabel('Time (s)', fontsize=13, fontweight='bold')
    axes[1, 2].set_ylabel('Torque (Nm)', fontsize=13, fontweight='bold')
    axes[1, 2].set_title('Sensor Torque Readings (From Logger)', fontsize=15, fontweight='bold')
    axes[1, 2].legend(loc='upper right', fontsize=11)
    axes[1, 2].grid(True, alpha=0.3)
    axes[1, 2].set_xlim(left=0)
    
    # Add main title with filenames
    fig.suptitle(f'Dataset: {dataset_filename}\nLog: {log_filename}', 
                 fontsize=12, fontweight='bold', y=0.995)
    
    plt.tight_layout(rect=[0, 0, 1, 0.985])  # Adjust layout to make room for suptitle
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f'Saved: {output_file}')
    plt.close()


def plot_torque_comparison(dataset_time, dataset_tau1, dataset_tau2, dataset_tau3,
                           sensor_time, sensor_tau1, sensor_tau2, sensor_tau3,
                           output_file='torque_comparison.png'):
    """
    Plot torque comparison: Injected (top) vs Sensor readings (bottom).
    """
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
    
    # Top plot: Injected torques
    ax1.plot(dataset_time, dataset_tau1, 'r-', linewidth=1.5, label='Joint 1', alpha=0.8)
    ax1.plot(dataset_time, dataset_tau2, 'g-', linewidth=1.5, label='Joint 2', alpha=0.8)
    ax1.plot(dataset_time, dataset_tau3, 'b-', linewidth=1.5, label='Joint 3', alpha=0.8)
    ax1.set_ylabel('Torque (Nm)', fontsize=12, fontweight='bold')
    ax1.set_title('Injected Torques (From Dataset)', fontsize=14, fontweight='bold')
    ax1.legend(loc='upper right', fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(left=0)
    
    # Bottom plot: Sensor torques
    ax2.plot(sensor_time, sensor_tau1, 'r-', linewidth=1.5, label='Joint 1', alpha=0.8)
    ax2.plot(sensor_time, sensor_tau2, 'g-', linewidth=1.5, label='Joint 2', alpha=0.8)
    ax2.plot(sensor_time, sensor_tau3, 'b-', linewidth=1.5, label='Joint 3', alpha=0.8)
    ax2.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Torque (Nm)', fontsize=12, fontweight='bold')
    ax2.set_title('Sensor Torque Readings (From Continuous Logger)', fontsize=14, fontweight='bold')
    ax2.legend(loc='upper right', fontsize=10)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(left=0)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f'Saved: {output_file}')
    plt.close()


def plot_position_comparison(dataset_time, dataset_pos1, dataset_pos2, dataset_pos3,
                             sensor_time, sensor_pos1, sensor_pos2, sensor_pos3,
                             output_file='position_comparison.png'):
    """
    Plot position comparison: Expected (top) vs Sensed readings (bottom).
    """
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
    
    # Convert radians to degrees for better readability
    dataset_pos1_deg = np.rad2deg(dataset_pos1)
    dataset_pos2_deg = np.rad2deg(dataset_pos2)
    dataset_pos3_deg = np.rad2deg(dataset_pos3)
    sensor_pos1_deg = np.rad2deg(sensor_pos1)
    sensor_pos2_deg = np.rad2deg(sensor_pos2)
    sensor_pos3_deg = np.rad2deg(sensor_pos3)
    
    # Top plot: Expected positions
    ax1.plot(dataset_time, dataset_pos1_deg, 'r-', linewidth=1.5, label='Joint 1', alpha=0.8)
    ax1.plot(dataset_time, dataset_pos2_deg, 'g-', linewidth=1.5, label='Joint 2', alpha=0.8)
    ax1.plot(dataset_time, dataset_pos3_deg, 'b-', linewidth=1.5, label='Joint 3', alpha=0.8)
    ax1.set_ylabel('Position (degrees)', fontsize=12, fontweight='bold')
    ax1.set_title('Expected Positions (From Dataset)', fontsize=14, fontweight='bold')
    ax1.legend(loc='upper right', fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(left=0)
    
    # Bottom plot: Sensed positions
    ax2.plot(sensor_time, sensor_pos1_deg, 'r-', linewidth=1.5, label='Joint 1', alpha=0.8)
    ax2.plot(sensor_time, sensor_pos2_deg, 'g-', linewidth=1.5, label='Joint 2', alpha=0.8)
    ax2.plot(sensor_time, sensor_pos3_deg, 'b-', linewidth=1.5, label='Joint 3', alpha=0.8)
    ax2.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Position (degrees)', fontsize=12, fontweight='bold')
    ax2.set_title('Sensed Positions (From Continuous Logger)', fontsize=14, fontweight='bold')
    ax2.legend(loc='upper right', fontsize=10)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(left=0)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f'Saved: {output_file}')
    plt.close()


def main():
    if len(sys.argv) < 3:
        print("Usage: python3 plot_comparison.py <dataset_file> <continuous_log_file> [pos_threshold] [tau_threshold] [--allinone]")
        print("\nExample:")
        print("  python3 plot_comparison.py src/scripts/move_q2_traj_joint_states.csv logs/continuous_log_1_20260104_173527.csv")
        print("\nOptional thresholds (for outlier filtering):")
        print("  pos_threshold: Max position change in radians (default: 1.0)")
        print("  tau_threshold: Max torque change in Nm (default: 100.0)")
        print("\nPlot mode flag:")
        print("  --allinone: Use all-in-one overlaid plot instead of 2x3 grid")
        print("\nExample with custom thresholds:")
        print("  python3 plot_comparison.py dataset.csv log.csv 0.5 50.0")
        print("\nExample with all-in-one mode:")
        print("  python3 plot_comparison.py dataset.csv log.csv --allinone")
        print("  python3 plot_comparison.py dataset.csv log.csv 0.5 50.0 --allinone")
        sys.exit(1)
    
    dataset_file = sys.argv[1]
    log_file = sys.argv[2]
    
    # Parse remaining arguments: detect --allinone flag and numeric thresholds
    global POS_THRESHOLD, TAU_THRESHOLD, PLOT_MODE
    numeric_args = []
    for arg in sys.argv[3:]:
        if arg == '--allinone':
            PLOT_MODE = 'allinone'
        else:
            try:
                numeric_args.append(float(arg))
            except ValueError:
                print(f"Warning: Unknown argument '{arg}', ignoring.")
    
    if len(numeric_args) >= 1:
        POS_THRESHOLD = numeric_args[0]
    if len(numeric_args) >= 2:
        TAU_THRESHOLD = numeric_args[1]
    
    # Check if files exist
    if not Path(dataset_file).exists():
        print(f"Error: Dataset file not found: {dataset_file}")
        sys.exit(1)
    
    if not Path(log_file).exists():
        print(f"Error: Log file not found: {log_file}")
        sys.exit(1)
    
    print('='*70)
    print('TRAJECTORY COMPARISON PLOTTER')
    print('='*70)
    print(f'Dataset file: {dataset_file}')
    print(f'Log file: {log_file}')
    print(f'Plot mode: {PLOT_MODE}')
    print()
    
    # Load data
    print('Loading dataset file...')
    ds_time, ds_pos1, ds_pos2, ds_pos3, ds_vel1, ds_vel2, ds_vel3, ds_tau1, ds_tau2, ds_tau3 = load_dataset_file(dataset_file)
    print(f'  - Loaded {len(ds_time)} trajectory points')
    print(f'  - Duration: {ds_time[-1]:.2f}s')
    
    print('Loading continuous log file...')
    log_time, log_pos1, log_pos2, log_pos3, log_vel1, log_vel2, log_vel3, log_tau1, log_tau2, log_tau3 = load_continuous_log(log_file)
    print(f'  - Loaded {len(log_time)} samples')
    print(f'  - Duration: {log_time[-1]:.2f}s')
    print()
    
    # Extract filenames and determine output path
    dataset_basename = Path(dataset_file).name
    log_basename = Path(log_file).name
    
    # Extract trajectory name from log filename (e.g., trajectory_path_005_log_2 -> path_005_2)
    match = re.search(r'path_(\w+)', log_basename)
    if match:
        traj_log_name = match.group(1)
        if PLOT_MODE == 'allinone':
            output_file = f'src/scripts/plots/trajectory_comparison_{traj_log_name}.png'
        else:
            output_file = f'src/scripts/plots/trajectory_comparison_{traj_log_name}_individual.png'
    else:
        if PLOT_MODE == 'allinone':
            output_file = f'src/scripts/plots/trajectory_comparison_{log_basename}.png'
        else:
            output_file = f'src/scripts/plots/trajectory_comparison_{log_basename}_individual.png'
    
    # Create plots directory if it doesn't exist
    Path('src/scripts/plots').mkdir(exist_ok=True)
    
    # Generate plots based on mode
    if PLOT_MODE == 'allinone':
        print('Generating all-in-one comparison plot...')
        plot_all_in_one(
            ds_time, ds_pos1, ds_pos2, ds_pos3, ds_vel1, ds_vel2, ds_vel3, ds_tau1, ds_tau2, ds_tau3,
            log_time, log_pos1, log_pos2, log_pos3, log_vel1, log_vel2, log_vel3, log_tau1, log_tau2, log_tau3,
            dataset_filename=dataset_basename,
            log_filename=log_basename,
            output_file=output_file
        )
    else:
        print('Generating comprehensive comparison plot...')
        plot_all_comparisons(
            ds_time, ds_pos1, ds_pos2, ds_pos3, ds_vel1, ds_vel2, ds_vel3, ds_tau1, ds_tau2, ds_tau3,
            log_time, log_pos1, log_pos2, log_pos3, log_vel1, log_vel2, log_vel3, log_tau1, log_tau2, log_tau3,
            dataset_filename=dataset_basename,
            log_filename=log_basename,
            output_file=output_file
        )
    
    print()
    print('='*70)
    print('PLOTTING COMPLETE')
    print('='*70)
    print('Generated file:')
    if PLOT_MODE == 'allinone':
        print(f'  - {output_file} (3 rows: position/velocity/torque, solid=expected, dash-dot=sensed)')
    else:
        print(f'  - {output_file} (2x3 grid: top=expected, bottom=sensed)')
    print('='*70)


if __name__ == '__main__':
    main()
