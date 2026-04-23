#!/usr/bin/env python3
"""
Plot Comparison Script
Compares injected positions/velocities/torques with sensor readings from continuous logger.
Automatically filters outliers by comparing consecutive values.

Usage:
    python3 plot_comparison.py <dataset_file> <continuous_log_file> [pos_threshold] [tau_threshold] [--interactive]
    
Example:
    python3 plot_comparison.py src/scripts/move_q2_traj_joint_states.csv logs/continuous_log_1_20260104_173527.csv

With custom thresholds:
    python3 plot_comparison.py dataset.csv log.csv 0.5 50.0

With interactive plots:
    python3 plot_comparison.py dataset.csv log.csv 0.5 50.0 --interactive

Generates output files:
    - trajectory_comparison.png (static 2x3 grid)
    OR
    - trajectory_comparison.html (interactive plot with hover tooltips)

Outlier Filtering:
    - Position changes > 1.0 rad (default) are filtered
    - Torque changes > 25.0 Nm (default) are filtered
    - Outliers are replaced with previous valid value
"""

import os
os.environ['NUMPY_EXPERIMENTAL_ARRAY_FUNCTION'] = '0'

import sys
import csv
import numpy as np
import warnings
warnings.filterwarnings('ignore')

# Try to import Plotly BEFORE matplotlib configuration
PLOTLY_AVAILABLE = False
try:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
    PLOTLY_AVAILABLE = True
except Exception as e:
    PLOTLY_AVAILABLE = False
    _plotly_error = str(e)

# Configure matplotlib for non-interactive use (only if not in interactive mode)
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from pathlib import Path


# Default outlier filtering thresholds
POS_THRESHOLD = 1.0  # radians
TAU_THRESHOLD = 25.0  # Nm
INTERACTIVE_MODE = False


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
    
    return time, pos1, pos2, pos3, vel1, vel2, vel3, tau1, tau2, tau3


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


def plot_all_comparisons_plotly(dataset_time, dataset_pos1, dataset_pos2, dataset_pos3,
                                dataset_vel1, dataset_vel2, dataset_vel3,
                                dataset_tau1, dataset_tau2, dataset_tau3,
                                sensor_time, sensor_pos1, sensor_pos2, sensor_pos3,
                                sensor_vel1, sensor_vel2, sensor_vel3,
                                sensor_tau1, sensor_tau2, sensor_tau3,
                                output_file='trajectory_comparison.html'):
    """
    Plot all comparisons using Plotly with interactive features (hover tooltips, zoom, etc).
    Creates a 2x3 subplot grid:
    Top row: Expected/Injected values (Position, Velocity, Torque)
    Bottom row: Sensed values (Position, Velocity, Torque)
    """
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
    
    # Create subplots
    fig = make_subplots(
        rows=2, cols=3,
        subplot_titles=(
            'Expected Positions (From Dataset)',
            'Expected Velocities (From Dataset)',
            'Injected Torques (From Dataset)',
            'Sensed Positions (From Logger)',
            'Sensed Velocities (From Logger)',
            'Sensor Torque Readings (From Logger)'
        )
    )
    
    # Define colors for each joint
    colors = {'Joint 1': '#FF0000', 'Joint 2': '#00AA00', 'Joint 3': '#0000FF'}
    
    # TOP ROW: EXPECTED/INJECTED VALUES
    
    # Top Left: Expected Positions
    fig.add_trace(
        go.Scatter(x=dataset_time, y=dataset_pos1_deg, mode='lines', name='Joint 1',
                   line=dict(color=colors['Joint 1'], width=2),
                   hovertemplate='<b>Joint 1</b><br>Time: %{x:.3f}s<br>Position: %{y:.2f}°<extra></extra>'),
        row=1, col=1
    )
    fig.add_trace(
        go.Scatter(x=dataset_time, y=dataset_pos2_deg, mode='lines', name='Joint 2',
                   line=dict(color=colors['Joint 2'], width=2),
                   hovertemplate='<b>Joint 2</b><br>Time: %{x:.3f}s<br>Position: %{y:.2f}°<extra></extra>'),
        row=1, col=1
    )
    fig.add_trace(
        go.Scatter(x=dataset_time, y=dataset_pos3_deg, mode='lines', name='Joint 3',
                   line=dict(color=colors['Joint 3'], width=2),
                   hovertemplate='<b>Joint 3</b><br>Time: %{x:.3f}s<br>Position: %{y:.2f}°<extra></extra>'),
        row=1, col=1
    )
    
    # Top Center: Expected Velocities
    fig.add_trace(
        go.Scatter(x=dataset_time, y=dataset_vel1_deg, mode='lines', name='Joint 1', showlegend=False,
                   line=dict(color=colors['Joint 1'], width=2),
                   hovertemplate='<b>Joint 1</b><br>Time: %{x:.3f}s<br>Velocity: %{y:.2f}°/s<extra></extra>'),
        row=1, col=2
    )
    fig.add_trace(
        go.Scatter(x=dataset_time, y=dataset_vel2_deg, mode='lines', name='Joint 2', showlegend=False,
                   line=dict(color=colors['Joint 2'], width=2),
                   hovertemplate='<b>Joint 2</b><br>Time: %{x:.3f}s<br>Velocity: %{y:.2f}°/s<extra></extra>'),
        row=1, col=2
    )
    fig.add_trace(
        go.Scatter(x=dataset_time, y=dataset_vel3_deg, mode='lines', name='Joint 3', showlegend=False,
                   line=dict(color=colors['Joint 3'], width=2),
                   hovertemplate='<b>Joint 3</b><br>Time: %{x:.3f}s<br>Velocity: %{y:.2f}°/s<extra></extra>'),
        row=1, col=2
    )
    
    # Top Right: Injected Torques
    fig.add_trace(
        go.Scatter(x=dataset_time, y=dataset_tau1, mode='lines', name='Joint 1', showlegend=False,
                   line=dict(color=colors['Joint 1'], width=2),
                   hovertemplate='<b>Joint 1</b><br>Time: %{x:.3f}s<br>Torque: %{y:.2f} Nm<extra></extra>'),
        row=1, col=3
    )
    fig.add_trace(
        go.Scatter(x=dataset_time, y=dataset_tau2, mode='lines', name='Joint 2', showlegend=False,
                   line=dict(color=colors['Joint 2'], width=2),
                   hovertemplate='<b>Joint 2</b><br>Time: %{x:.3f}s<br>Torque: %{y:.2f} Nm<extra></extra>'),
        row=1, col=3
    )
    fig.add_trace(
        go.Scatter(x=dataset_time, y=dataset_tau3, mode='lines', name='Joint 3', showlegend=False,
                   line=dict(color=colors['Joint 3'], width=2),
                   hovertemplate='<b>Joint 3</b><br>Time: %{x:.3f}s<br>Torque: %{y:.2f} Nm<extra></extra>'),
        row=1, col=3
    )
    
    # BOTTOM ROW: SENSED VALUES
    
    # Bottom Left: Sensed Positions
    fig.add_trace(
        go.Scatter(x=sensor_time, y=sensor_pos1_deg, mode='lines', name='Joint 1', showlegend=False,
                   line=dict(color=colors['Joint 1'], width=2),
                   hovertemplate='<b>Joint 1</b><br>Time: %{x:.3f}s<br>Position: %{y:.2f}°<extra></extra>'),
        row=2, col=1
    )
    fig.add_trace(
        go.Scatter(x=sensor_time, y=sensor_pos2_deg, mode='lines', name='Joint 2', showlegend=False,
                   line=dict(color=colors['Joint 2'], width=2),
                   hovertemplate='<b>Joint 2</b><br>Time: %{x:.3f}s<br>Position: %{y:.2f}°<extra></extra>'),
        row=2, col=1
    )
    fig.add_trace(
        go.Scatter(x=sensor_time, y=sensor_pos3_deg, mode='lines', name='Joint 3', showlegend=False,
                   line=dict(color=colors['Joint 3'], width=2),
                   hovertemplate='<b>Joint 3</b><br>Time: %{x:.3f}s<br>Position: %{y:.2f}°<extra></extra>'),
        row=2, col=1
    )
    
    # Bottom Center: Sensed Velocities
    fig.add_trace(
        go.Scatter(x=sensor_time, y=sensor_vel1_deg, mode='lines', name='Joint 1', showlegend=False,
                   line=dict(color=colors['Joint 1'], width=2),
                   hovertemplate='<b>Joint 1</b><br>Time: %{x:.3f}s<br>Velocity: %{y:.2f}°/s<extra></extra>'),
        row=2, col=2
    )
    fig.add_trace(
        go.Scatter(x=sensor_time, y=sensor_vel2_deg, mode='lines', name='Joint 2', showlegend=False,
                   line=dict(color=colors['Joint 2'], width=2),
                   hovertemplate='<b>Joint 2</b><br>Time: %{x:.3f}s<br>Velocity: %{y:.2f}°/s<extra></extra>'),
        row=2, col=2
    )
    fig.add_trace(
        go.Scatter(x=sensor_time, y=sensor_vel3_deg, mode='lines', name='Joint 3', showlegend=False,
                   line=dict(color=colors['Joint 3'], width=2),
                   hovertemplate='<b>Joint 3</b><br>Time: %{x:.3f}s<br>Velocity: %{y:.2f}°/s<extra></extra>'),
        row=2, col=2
    )
    
    # Bottom Right: Sensor Torques
    fig.add_trace(
        go.Scatter(x=sensor_time, y=sensor_tau1, mode='lines', name='Joint 1', showlegend=False,
                   line=dict(color=colors['Joint 1'], width=2),
                   hovertemplate='<b>Joint 1</b><br>Time: %{x:.3f}s<br>Torque: %{y:.2f} Nm<extra></extra>'),
        row=2, col=3
    )
    fig.add_trace(
        go.Scatter(x=sensor_time, y=sensor_tau2, mode='lines', name='Joint 2', showlegend=False,
                   line=dict(color=colors['Joint 2'], width=2),
                   hovertemplate='<b>Joint 2</b><br>Time: %{x:.3f}s<br>Torque: %{y:.2f} Nm<extra></extra>'),
        row=2, col=3
    )
    fig.add_trace(
        go.Scatter(x=sensor_time, y=sensor_tau3, mode='lines', name='Joint 3', showlegend=False,
                   line=dict(color=colors['Joint 3'], width=2),
                   hovertemplate='<b>Joint 3</b><br>Time: %{x:.3f}s<br>Torque: %{y:.2f} Nm<extra></extra>'),
        row=2, col=3
    )
    
    # Update axes labels
    fig.update_xaxes(title_text='Time (s)', row=2, col=1)
    fig.update_xaxes(title_text='Time (s)', row=2, col=2)
    fig.update_xaxes(title_text='Time (s)', row=2, col=3)
    
    fig.update_yaxes(title_text='Position (°)', row=1, col=1)
    fig.update_yaxes(title_text='Position (°)', row=2, col=1)
    fig.update_yaxes(title_text='Velocity (°/s)', row=1, col=2)
    fig.update_yaxes(title_text='Velocity (°/s)', row=2, col=2)
    fig.update_yaxes(title_text='Torque (Nm)', row=1, col=3)
    fig.update_yaxes(title_text='Torque (Nm)', row=2, col=3)
    
    # Update layout
    fig.update_layout(
        title_text='Trajectory Comparison: Expected vs Sensed Values (Interactive)',
        height=900,
        width=1800,
        showlegend=True,
        hovermode='x unified',
        font=dict(size=11)
    )
    
    fig.write_html(output_file)
    print(f'Saved: {output_file}')


def plot_all_comparisons(dataset_time, dataset_pos1, dataset_pos2, dataset_pos3,
                         dataset_vel1, dataset_vel2, dataset_vel3,
                         dataset_tau1, dataset_tau2, dataset_tau3,
                         sensor_time, sensor_pos1, sensor_pos2, sensor_pos3,
                         sensor_vel1, sensor_vel2, sensor_vel3,
                         sensor_tau1, sensor_tau2, sensor_tau3,
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
    
    plt.tight_layout()
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
    global INTERACTIVE_MODE
    
    if len(sys.argv) < 3 or len(sys.argv) > 6:
        print("Usage: python3 plot_comparison.py <dataset_file> <continuous_log_file> [pos_threshold] [tau_threshold] [--interactive]")
        print("\nExample (static PNG):")
        print("  python3 plot_comparison.py src/scripts/move_q2_traj_joint_states.csv logs/continuous_log_1_20260104_173527.csv")
        print("\nExample (interactive HTML with hover tooltips):")
        print("  python3 plot_comparison.py dataset.csv log.csv --interactive")
        print("\nWith custom thresholds:")
        print("  python3 plot_comparison.py dataset.csv log.csv 0.5 50.0")
        print("\nWith custom thresholds and interactive mode:")
        print("  python3 plot_comparison.py dataset.csv log.csv 0.5 50.0 --interactive")
        print("\nOptional thresholds (for outlier filtering):")
        print("  pos_threshold: Max position change in radians (default: 1.0)")
        print("  tau_threshold: Max torque change in Nm (default: 25.0)")
        sys.exit(1)
    
    dataset_file = sys.argv[1]
    log_file = sys.argv[2]
    
    # Optional threshold parameters and interactive flag
    global POS_THRESHOLD, TAU_THRESHOLD
    for i in range(3, len(sys.argv)):
        if sys.argv[i] == '--interactive':
            INTERACTIVE_MODE = True
        else:
            try:
                if i == 3:
                    POS_THRESHOLD = float(sys.argv[i])
                elif i == 4:
                    TAU_THRESHOLD = float(sys.argv[i])
            except ValueError:
                print(f"Invalid threshold value: {sys.argv[i]}")
                sys.exit(1)
    
    # Check if files exist
    if not Path(dataset_file).exists():
        print(f"Error: Dataset file not found: {dataset_file}")
        sys.exit(1)
    
    if not Path(log_file).exists():
        print(f"Error: Log file not found: {log_file}")
        sys.exit(1)
    
    # Check if Plotly is available in interactive mode
    if INTERACTIVE_MODE and not PLOTLY_AVAILABLE:
        print("Error: Plotly is required for interactive mode.")
        print("Install it with: pip install plotly")
        if '_plotly_error' in globals():
            print(f"Import error details: {_plotly_error}")
        sys.exit(1)
    
    print('='*70)
    print('TRAJECTORY COMPARISON PLOTTER')
    print('='*70)
    print(f'Dataset file: {dataset_file}')
    print(f'Log file: {log_file}')
    print(f'Output mode: {"INTERACTIVE (HTML)" if INTERACTIVE_MODE else "STATIC (PNG)"}')
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
    
    # Generate plots
    if INTERACTIVE_MODE:
        print('Generating interactive comparison plot (HTML with hover tooltips)...')
        plot_all_comparisons_plotly(
            ds_time, ds_pos1, ds_pos2, ds_pos3, ds_vel1, ds_vel2, ds_vel3, ds_tau1, ds_tau2, ds_tau3,
            log_time, log_pos1, log_pos2, log_pos3, log_vel1, log_vel2, log_vel3, log_tau1, log_tau2, log_tau3
        )
    else:
        print('Generating static comparison plot (PNG)...')
        plot_all_comparisons(
            ds_time, ds_pos1, ds_pos2, ds_pos3, ds_vel1, ds_vel2, ds_vel3, ds_tau1, ds_tau2, ds_tau3,
            log_time, log_pos1, log_pos2, log_pos3, log_vel1, log_vel2, log_vel3, log_tau1, log_tau2, log_tau3
        )
    
    print()
    print('='*70)
    print('PLOTTING COMPLETE')
    print('='*70)
    if INTERACTIVE_MODE:
        print('Generated file:')
        print('  - trajectory_comparison.html (interactive plot with hover tooltips)')
    else:
        print('Generated file:')
        print('  - trajectory_comparison.png (2x3 grid: top=expected, bottom=sensed)')
    print('='*70)


if __name__ == '__main__':
    main()