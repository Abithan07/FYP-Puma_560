#!/usr/bin/env python3
"""
validate.py
Compares DNN-predicted desired trajectory against the actual Gazebo-logged
joint states and reports per-joint RMSE + generates comparison plots.

Usage:
  python3 validate.py \\
      --desired /path/to/path_XXX_joint_states.csv \\
      --actual  ~/puma560_logs/path_XXX_log_1.csv  \\
      --out     ./results

The 'desired' CSV is row-wise (produced by DNN inference):
  row key → t, dp1, dp2, dp3, ...

The 'actual' CSV is column-wise (produced by triggered_logger.py):
  time_elapsed, pos1, pos2, pos3, vel1, vel2, vel3, torque1, torque2, torque3

Time alignment:
  The logger records starting from t=0 at the moment the first torque command
  is sent. The desired CSV t-axis starts at some offset (GRU warmup trim).
  Both are re-based to start at 0 for comparison.
"""

import argparse
import os
import sys

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import csv


# ── Loaders ─────────────────────────────────────────────────────────────────

def load_desired(path: str) -> dict:
    """Load the row-wise DNN output CSV. Returns dict of key→np.array."""
    if not os.path.isfile(path):
        sys.exit(f'ERROR: Desired CSV not found: {path}')
    data = {}
    with open(path) as f:
        for row in csv.reader(f):
            if row:
                data[row[0]] = np.array([float(v) for v in row[1:]])
    required = ['t', 'dp1', 'dp2', 'dp3']
    missing  = [k for k in required if k not in data]
    if missing:
        sys.exit(f'ERROR: Desired CSV missing rows: {missing}')
    return data


def load_actual(path: str) -> dict:
    """Load the column-wise logger CSV. Returns dict of header→np.array."""
    if not os.path.isfile(path):
        sys.exit(f'ERROR: Actual log not found: {path}')
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    if not rows:
        sys.exit('ERROR: Actual log CSV is empty.')
    keys = list(rows[0].keys())
    required = ['time_elapsed', 'pos1', 'pos2', 'pos3']
    missing  = [k for k in required if k not in keys]
    if missing:
        sys.exit(f'ERROR: Actual log CSV missing columns: {missing}')
    return {k: np.array([float(r[k]) for r in rows]) for k in keys}


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Trajectory tracking validator')
    parser.add_argument('--desired', required=True,
                        help='DNN joint_states CSV (row-wise)')
    parser.add_argument('--actual',  required=True,
                        help='Logger output CSV (column-wise)')
    parser.add_argument('--out', default='.',
                        help='Output directory for plots (default: .)')
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)

    # ── Load ─────────────────────────────────────────────────────────────
    des = load_desired(args.desired)
    act = load_actual(args.actual)

    t_des = des['t'] - des['t'][0]          # re-base to 0
    t_act = act['time_elapsed']              # already 0-based

    dp  = [des['dp1'], des['dp2'], des['dp3']]
    pos = [act['pos1'], act['pos2'], act['pos3']]

    # ── Common time grid (interpolate both onto it) ───────────────────────
    t_start = max(t_des[0],  t_act[0])
    t_end   = min(t_des[-1], t_act[-1])

    if t_end <= t_start:
        sys.exit('ERROR: Desired and actual trajectories do not overlap in time.')

    n_pts   = max(int((t_end - t_start) / 0.01), 2)
    t_grid  = np.linspace(t_start, t_end, n_pts)

    des_interp = [np.interp(t_grid, t_des, dp[j])  for j in range(3)]
    act_interp = [np.interp(t_grid, t_act, pos[j]) for j in range(3)]

    # ── Metrics ──────────────────────────────────────────────────────────
    rmse     = [np.sqrt(np.mean((des_interp[j] - act_interp[j])**2)) for j in range(3)]
    max_err  = [np.max(np.abs(des_interp[j] - act_interp[j]))        for j in range(3)]
    rmse_deg = [r * 180 / np.pi for r in rmse]
    max_deg  = [m * 180 / np.pi for m in max_err]

    print('\n' + '='*60)
    print('Trajectory Tracking Validation Report')
    print('='*60)
    print(f'  Overlap duration : {t_end-t_start:.2f} s  ({n_pts} samples)')
    print(f'  Desired start    : {des["t"][0]:.3f} s  →  re-based to 0')
    print()
    print(f'  {"Joint":<8}  {"RMSE (rad)":>12}  {"RMSE (°)":>10}  {"Max |e| (°)":>12}')
    print(f'  {"-"*8}  {"-"*12}  {"-"*10}  {"-"*12}')
    for j in range(3):
        print(f'  joint_{j+1}   {rmse[j]:>12.6f}  {rmse_deg[j]:>10.4f}  {max_deg[j]:>12.4f}')
    print('='*60 + '\n')

    # ── Plots ─────────────────────────────────────────────────────────────
    joint_labels = ['Joint 1', 'Joint 2', 'Joint 3']
    colors_des   = ['#1f77b4', '#ff7f0e', '#2ca02c']
    colors_act   = ['#aec7e8', '#ffbb78', '#98df8a']

    # -- Plot 1: Comparison (all joints stacked) --
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.suptitle('Trajectory Tracking: Desired vs Actual', fontsize=14, fontweight='bold')

    for j in range(3):
        ax = axes[j]
        ax.plot(t_grid, np.degrees(des_interp[j]), color=colors_des[j],
                lw=2, label='Desired (DNN)')
        ax.plot(t_grid, np.degrees(act_interp[j]), color=colors_act[j],
                lw=1.5, ls='--', label='Actual (Gazebo)')
        ax.set_ylabel(f'{joint_labels[j]}\nPosition (°)', fontsize=10)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_title(f'RMSE = {rmse_deg[j]:.4f}°   Max error = {max_deg[j]:.4f}°',
                     fontsize=9, loc='left')

    axes[-1].set_xlabel('Time (s)', fontsize=10)
    fig.tight_layout()
    out1 = os.path.join(args.out, 'tracking_comparison.png')
    fig.savefig(out1, dpi=150)
    print(f'Saved: {out1}')
    plt.close(fig)

    # -- Plot 2: Tracking error per joint --
    fig2, axes2 = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig2.suptitle('Tracking Error (Desired − Actual)', fontsize=14, fontweight='bold')

    for j in range(3):
        err_deg = np.degrees(des_interp[j] - act_interp[j])
        ax = axes2[j]
        ax.plot(t_grid, err_deg, color=colors_des[j], lw=1.5)
        ax.axhline(0, color='k', lw=0.8, ls=':')
        ax.fill_between(t_grid, err_deg, alpha=0.2, color=colors_des[j])
        ax.set_ylabel(f'{joint_labels[j]}\nError (°)', fontsize=10)
        ax.set_title(f'RMSE = {rmse_deg[j]:.4f}°', fontsize=9, loc='left')
        ax.grid(True, alpha=0.3)

    axes2[-1].set_xlabel('Time (s)', fontsize=10)
    fig2.tight_layout()
    out2 = os.path.join(args.out, 'tracking_error.png')
    fig2.savefig(out2, dpi=150)
    print(f'Saved: {out2}')
    plt.close(fig2)

    # -- Plot 3: Phase portrait (actual velocity vs position per joint) --
    if 'vel1' in act and 'vel2' in act and 'vel3' in act:
        vel = [act['vel1'], act['vel2'], act['vel3']]
        vel_interp = [np.interp(t_grid, t_act, vel[j]) for j in range(3)]

        fig3, axes3 = plt.subplots(1, 3, figsize=(14, 5))
        fig3.suptitle('Phase Portrait — Actual Trajectory', fontsize=13, fontweight='bold')
        for j in range(3):
            ax = axes3[j]
            ax.plot(np.degrees(act_interp[j]), np.degrees(vel_interp[j]),
                    color=colors_des[j], lw=1.5)
            ax.set_xlabel(f'{joint_labels[j]} Position (°)', fontsize=9)
            ax.set_ylabel('Velocity (°/s)', fontsize=9)
            ax.set_title(joint_labels[j], fontsize=10)
            ax.grid(True, alpha=0.3)
        fig3.tight_layout()
        out3 = os.path.join(args.out, 'phase_portrait.png')
        fig3.savefig(out3, dpi=150)
        print(f'Saved: {out3}')
        plt.close(fig3)

    print('\nValidation complete.')


if __name__ == '__main__':
    main()
