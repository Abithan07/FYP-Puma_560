#!/usr/bin/env python3
"""
DNN Torque Controller Analysis Tool
Analyses logged data from torque_publisher_dnn.py:
  - Tracking performance (position / velocity errors)
  - DeLaN physics baseline vs GRU residual contribution
  - DNN feedforward vs PD+I feedback split
  - GRU warmup effect (pre/post step 128)
  - Optional side-by-side comparison with a CTC log
"""

import sys
import re
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
import matplotlib.patches as mpatches
from pathlib import Path
import argparse
from typing import Dict, Optional


PLOTS_DIR = Path('/home/priyankan/Desktop/FYP-Puma_560/arm_bot/src/scripts/DNN_Analysis')
GRU_WARMUP = 128   # steps before GRU activates


# ======================================================================== #
class DNNAnalyzer:
    """Analyse a DNN torque controller log produced by torque_publisher_dnn.py."""

    def __init__(self, log_file: str):
        self.path = Path(log_file)
        if not self.path.exists():
            raise FileNotFoundError(f'Log file not found: {log_file}')
        self.data = pd.read_csv(log_file)
        self._validate()

        # Extract path ID from filename, e.g. "path_461_..." → "Path 461"
        m = re.search(r'path[_\-](\d+)', self.path.stem, re.IGNORECASE)
        self.path_id     = m.group(1) if m else None
        self.title_prefix = f'[Path {self.path_id}]  ' if self.path_id else ''

        self.t          = self.data['t'].to_numpy()
        self.has_gru_col = 'gru_active' in self.data.columns
        # Split index: first step where gru_active == 1
        if self.has_gru_col:
            active = self.data['gru_active'].to_numpy()
            idx    = np.where(active == 1)[0]
            self.warmup_end = int(idx[0]) if len(idx) else len(self.t)
        else:
            self.warmup_end = GRU_WARMUP
        print(f'✓ Loaded: {log_file}')
        print(f'  Timesteps : {len(self.data)}')
        print(f'  Duration  : {self.t[-1]:.2f} s')
        print(f'  GRU active from step {self.warmup_end} '
              f'(t={self.t[self.warmup_end]:.2f}s)')

    def _validate(self):
        required = ['t', 'q_des_1', 'q_act_1', 'tau_dnn_1', 'tau_fb_1', 'tau_total_1',
                    'e_pos_1', 'e_vel_1']
        missing  = [c for c in required if c not in self.data.columns]
        if missing:
            raise ValueError(f'Missing columns: {missing}')
    
    def has_sensed_torques(self) -> bool:
        """Check if log file contains sensed torques from Gazebo"""
        return all(f'tau_sensed_{j}' in self.data.columns for j in [1, 2, 3])

    def sensed_torques_look_empty(self) -> bool:
        """Check whether sensed torque columns exist but are all near zero."""
        if not self.has_sensed_torques():
            return False
        sensed = np.concatenate([self._col('tau_sensed', j) for j in [1, 2, 3]])
        return np.all(np.abs(sensed) < 1e-9)
    
    def compute_sensed_vs_commanded_error(self) -> Dict:
        """Compute error between commanded and sensed torques from Gazebo"""
        errors = {}
        
        if not self.has_sensed_torques():
            return errors
        
        for joint in [1, 2, 3]:
            tau_cmd_col = f'tau_total_{joint}'
            tau_sen_col = f'tau_sensed_{joint}'
            
            if tau_cmd_col in self.data.columns and tau_sen_col in self.data.columns:
                tau_cmd = self.data[tau_cmd_col].values
                tau_sen = self.data[tau_sen_col].values
                tau_error = tau_cmd - tau_sen
                
                errors[f'j{joint}_rms'] = np.sqrt(np.mean(tau_error**2))
                errors[f'j{joint}_max'] = np.max(np.abs(tau_error))
                errors[f'j{joint}_mean'] = np.mean(np.abs(tau_error))
                errors[f'j{joint}_std'] = np.std(tau_error)
                
                # Peak torques
                errors[f'j{joint}_tau_cmd_peak'] = np.max(np.abs(tau_cmd))
                errors[f'j{joint}_tau_sen_peak'] = np.max(np.abs(tau_sen))
        
        return errors

    # ------------------------------------------------------------------ #
    #  Metric helpers                                                      #
    # ------------------------------------------------------------------ #
    def _col(self, name, joint):
        return self.data[f'{name}_{joint}'].to_numpy()

    def compute_tracking_errors(self) -> Dict:
        out = {}
        for j in [1, 2, 3]:
            e  = self._col('e_pos', j)
            ew = self.warmup_end
            out[f'j{j}_rms_deg']      = np.sqrt(np.mean(e**2))       * 180/np.pi
            out[f'j{j}_max_deg']      = np.max(np.abs(e))             * 180/np.pi
            out[f'j{j}_mean_deg']     = np.mean(np.abs(e))            * 180/np.pi
            out[f'j{j}_rms_warmup']   = np.sqrt(np.mean(e[:ew]**2))  * 180/np.pi
            out[f'j{j}_rms_postgru']  = np.sqrt(np.mean(e[ew:]**2))  * 180/np.pi if ew < len(e) else 0.0
        return out

    def compute_torque_split(self) -> Dict:
        out = {}
        for j in [1, 2, 3]:
            tau_dnn   = np.abs(self._col('tau_dnn',   j))
            tau_fb    = np.abs(self._col('tau_fb',    j))
            tau_total = np.abs(self._col('tau_total', j))
            mask = tau_total > 1e-6
            out[f'j{j}_dnn_pct']    = 100 * np.mean(tau_dnn[mask]  / tau_total[mask])
            out[f'j{j}_fb_pct']     = 100 * np.mean(tau_fb[mask]   / tau_total[mask])
            out[f'j{j}_peak_dnn']   = np.max(tau_dnn)
            out[f'j{j}_peak_fb']    = np.max(tau_fb)
            out[f'j{j}_peak_total'] = np.max(tau_total)
        return out

    def compute_gru_contribution(self) -> Dict:
        """How much does the GRU residual add on top of DeLaN (post-warmup)?"""
        out = {}
        if 'tau_delan_1' not in self.data.columns:
            return out
        ew = self.warmup_end
        for j in [1, 2, 3]:
            tau_delan = self._col('tau_delan', j)[ew:]
            tau_dnn   = self._col('tau_dnn',   j)[ew:]
            residual  = tau_dnn - tau_delan
            if len(residual):
                out[f'j{j}_gru_rms']  = np.sqrt(np.mean(residual**2))
                out[f'j{j}_gru_max']  = np.max(np.abs(residual))
                out[f'j{j}_gru_mean'] = np.mean(np.abs(residual))
        return out

    # ------------------------------------------------------------------ #
    #  Summary print                                                       #
    # ------------------------------------------------------------------ #
    def print_summary(self):
        err   = self.compute_tracking_errors()
        split = self.compute_torque_split()
        gru   = self.compute_gru_contribution()

        print('\n' + '='*80)
        print('DNN TORQUE CONTROLLER — PERFORMANCE SUMMARY')
        print('='*80)

        print('\n── TRACKING ERRORS ─────────────────────────────────────')
        for j in [1, 2, 3]:
            print(f'\n  Joint {j}:')
            print(f'    RMS (full)      : {err[f"j{j}_rms_deg"]:.4f}°')
            print(f'    RMS (warmup)    : {err[f"j{j}_rms_warmup"]:.4f}°  '
                  f'← DeLaN-only (steps 0–{self.warmup_end})')
            print(f'    RMS (post-GRU)  : {err[f"j{j}_rms_postgru"]:.4f}°  '
                  f'← DeLaN+GRU (step {self.warmup_end}+)')
            print(f'    Max             : {err[f"j{j}_max_deg"]:.4f}°')
            print(f'    Mean            : {err[f"j{j}_mean_deg"]:.4f}°')

        print('\n── TORQUE SPLIT ─────────────────────────────────────────')
        for j in [1, 2, 3]:
            print(f'\n  Joint {j}:')
            print(f'    DNN feedforward : {split[f"j{j}_dnn_pct"]:.1f}%  '
                  f'(peak {split[f"j{j}_peak_dnn"]:.2f} Nm)')
            print(f'    PD+I correction : {split[f"j{j}_fb_pct"]:.1f}%  '
                  f'(peak {split[f"j{j}_peak_fb"]:.2f} Nm)')
            print(f'    Peak total      : {split[f"j{j}_peak_total"]:.2f} Nm')

        if gru:
            print('\n── GRU RESIDUAL (post-warmup) ───────────────────────────')
            for j in [1, 2, 3]:
                if f'j{j}_gru_rms' in gru:
                    print(f'\n  Joint {j}:')
                    print(f'    RMS correction  : {gru[f"j{j}_gru_rms"]:.4f} Nm')
                    print(f'    Max correction  : {gru[f"j{j}_gru_max"]:.4f} Nm')
                    print(f'    Mean correction : {gru[f"j{j}_gru_mean"]:.4f} Nm')

        print('\n── OVERALL ──────────────────────────────────────────────')
        avg_rms   = np.mean([err[f'j{j}_rms_deg'] for j in [1,2,3]])
        avg_fb    = np.mean([split[f'j{j}_fb_pct'] for j in [1,2,3]])
        avg_post  = np.mean([err[f'j{j}_rms_postgru'] for j in [1,2,3]])
        avg_warm  = np.mean([err[f'j{j}_rms_warmup'] for j in [1,2,3]])
        print(f'  Average RMS error         : {avg_rms:.4f}°')
        print(f'  Avg RMS (warmup)          : {avg_warm:.4f}°')
        print(f'  Avg RMS (post-GRU)        : {avg_post:.4f}°')
        print(f'  Avg feedback contribution : {avg_fb:.1f}%')
        if avg_post < avg_warm:
            print(f'  ✓ GRU improves tracking  ({avg_warm-avg_post:.4f}° RMS reduction)')
        else:
            print(f'  ⚠ GRU did not reduce tracking error post-warmup')
        if avg_rms < 0.5:   print('  ✓ Excellent tracking')
        elif avg_rms < 1.0: print('  ✓ Good tracking')
        elif avg_rms < 2.0: print('  ◐ Acceptable tracking')
        else:               print('  ✗ Poor tracking — consider tuning gains')
        if avg_fb > 40:     print('  ⚠ High feedback fraction — DNN may not fit this trajectory well')
        elif avg_fb > 15:   print('  ◐ Moderate feedback — normal for residual correction')
        else:               print('  ✓ Low feedback — DNN feedforward is dominant')
        
        # Sensed vs Commanded Torque Analysis
        if self.has_sensed_torques():
            print('\n── SENSED vs COMMANDED TORQUES (from Gazebo) ────────────')
            if self.sensed_torques_look_empty():
                print('  ⚠ Sensed torque columns are present but contain only zeros.')
                print('    This usually means the controller did not populate JointState.effort.')
            sensed_errors = self.compute_sensed_vs_commanded_error()
            for joint in [1, 2, 3]:
                print(f'\n  Joint {joint}:')
                print(f'    RMS Error       : {sensed_errors.get(f"j{joint}_rms", 0):.4f} N⋅m')
                print(f'    Max Error       : {sensed_errors.get(f"j{joint}_max", 0):.4f} N⋅m')
                print(f'    Mean Error      : {sensed_errors.get(f"j{joint}_mean", 0):.4f} N⋅m')
                print(f'    Commanded Peak  : {sensed_errors.get(f"j{joint}_tau_cmd_peak", 0):.2f} N⋅m')
                print(f'    Sensed Peak     : {sensed_errors.get(f"j{joint}_tau_sen_peak", 0):.2f} N⋅m')
            
            avg_sensed_error = np.mean([sensed_errors.get(f'j{j}_rms', 0) for j in [1,2,3]])
            print(f'\n  Average Sensed-Commanded Error: {avg_sensed_error:.4f} N⋅m')
            if avg_sensed_error < 1.0:
                print('  ✓ Excellent torque tracking')
            elif avg_sensed_error < 5.0:
                print('  ✓ Good torque tracking')
            elif avg_sensed_error < 10.0:
                print('  ◐ Acceptable torque tracking')
            else:
                print('  ✗ Poor torque tracking — Check controller/actuator')

    # ------------------------------------------------------------------ #
    #  Plots                                                               #
    # ------------------------------------------------------------------ #
    def _warmup_shade(self, ax):
        """Shade the GRU warmup period on an axis."""
        ax.axvspan(self.t[0], self.t[self.warmup_end],
                   alpha=0.08, color='grey', label='GRU warmup')
        ax.axvline(self.t[self.warmup_end], color='grey',
                   linestyle=':', linewidth=1.2, alpha=0.7)

    def plot_trajectory_tracking(self, output_file: str = None):
        fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
        fig.suptitle(f'{self.title_prefix}DNN Controller — Position Tracking', fontsize=14, fontweight='bold')
        for j, ax in enumerate(axes, 1):
            q_des = np.degrees(self._col('q_des', j))
            q_act = np.degrees(self._col('q_act', j))
            ax.plot(self.t, q_des, 'b-',  linewidth=2,   label='Desired')
            ax.plot(self.t, q_act, 'r--', linewidth=1.5, label='Actual', alpha=0.8)
            self._warmup_shade(ax)
            ax.set_ylabel(f'Joint {j} (°)')
            ax.legend(loc='upper right', fontsize=9)
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel('Time (s)')
        plt.tight_layout()
        self._save_or_show(fig, output_file)

    def plot_tracking_errors(self, output_file: str = None):
        fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
        fig.suptitle(f'{self.title_prefix}DNN Controller — Position Tracking Errors', fontsize=14, fontweight='bold')
        err = self.compute_tracking_errors()
        for j, ax in enumerate(axes, 1):
            e = np.degrees(self._col('e_pos', j))
            ax.plot(self.t, e, 'r-', linewidth=1.5)
            ax.axhline(0, color='k', linestyle='--', alpha=0.3)
            self._warmup_shade(ax)
            rms = err[f'j{j}_rms_deg']
            ax.set_ylabel(f'Joint {j} error (°)')
            ax.set_title(f'Joint {j}  RMS={rms:.4f}°', fontsize=10)
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel('Time (s)')
        plt.tight_layout()
        self._save_or_show(fig, output_file)

    def plot_torque_breakdown(self, output_file: str = None):
        fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
        fig.suptitle(f'{self.title_prefix}DNN Controller — Torque Breakdown', fontsize=14, fontweight='bold')
        for j, ax in enumerate(axes, 1):
            tau_dnn   = self._col('tau_dnn',   j)
            tau_fb    = self._col('tau_fb',    j)
            tau_total = self._col('tau_total', j)
            ax.plot(self.t, tau_dnn,   'b-',  linewidth=2,   label='DNN (DeLaN+GRU)')
            ax.plot(self.t, tau_fb,    'g-',  linewidth=1.5, label='PD+I feedback', alpha=0.8)
            ax.plot(self.t, tau_total, 'r--', linewidth=1.5, label='Total commanded', alpha=0.8)
            
            # Add sensed torques if available
            if self.has_sensed_torques():
                tau_sensed = self._col('tau_sensed', j)
                ax.plot(self.t, tau_sensed, 'purple', linestyle=':', linewidth=2, 
                        label='Sensed (Gazebo)', alpha=0.75)
            
            self._warmup_shade(ax)
            ax.set_ylabel(f'Joint {j} torque (Nm)')
            ax.legend(loc='upper right', fontsize=9)
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel('Time (s)')
        plt.tight_layout()
        self._save_or_show(fig, output_file)

    def plot_all_joint_torques_same_scale(self, output_file: str = None):
        """Plot total commanded torques for joints 1-3 on one axis with shared scale."""
        fig, ax = plt.subplots(figsize=(13, 6))
        fig.suptitle(f'{self.title_prefix}Torque Breakdown — All Joints, Same Scale',
                     fontsize=14, fontweight='bold')

        colors = ['tab:blue', 'tab:orange', 'tab:green']
        styles = { 'dnn': '-', 'total': ':' }

        for j, color in enumerate(colors, start=1):
            tau_dnn   = self._col('tau_dnn',   j)
            tau_total = self._col('tau_total', j)

            ax.plot(self.t, tau_dnn,   color=color, linestyle=styles['dnn'],  linewidth=1.8, label=f'J{j} DNN')
            ax.plot(self.t, tau_total, color=color, linestyle=styles['total'],linewidth=1.6, alpha=0.8, label=f'J{j} Total')

        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Torque (Nm)')
        # x-axis grid every 0.1s
        ax.xaxis.set_major_locator(MultipleLocator(0.1))
        ax.grid(True, which='major', axis='x', linestyle='--', alpha=0.35)
        ax.grid(True, which='major', axis='y', alpha=0.3)
        # rotate x tick labels vertically
        plt.setp(ax.get_xticklabels(), rotation=90, ha='center')

        # Reduce legend duplicates by ordering: for readability keep all entries
        ax.legend(loc='upper right', fontsize=9, ncol=1)

        plt.tight_layout()
        self._save_or_show(fig, output_file)

    def plot_delan_vs_dnn(self, output_file: str = None):
        """DeLaN baseline vs full DNN (shows GRU residual contribution)."""
        if 'tau_delan_1' not in self.data.columns:
            print('⚠ tau_delan columns not present — skipping DeLaN vs DNN plot')
            return
        fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
        fig.suptitle(f'{self.title_prefix}DeLaN Baseline vs DNN (DeLaN + GRU residual)',
                     fontsize=14, fontweight='bold')
        for j, ax in enumerate(axes, 1):
            tau_delan = self._col('tau_delan', j)
            tau_dnn   = self._col('tau_dnn',   j)
            gru_res   = tau_dnn - tau_delan
            ax.plot(self.t, tau_delan, 'b-',  linewidth=2,   label='DeLaN (physics)', alpha=0.9)
            ax.plot(self.t, tau_dnn,   'r--', linewidth=1.5, label='DNN (DeLaN+GRU)', alpha=0.85)
            ax.fill_between(self.t, tau_delan, tau_dnn,
                            alpha=0.15, color='green', label='GRU residual')
            self._warmup_shade(ax)
            ax.set_ylabel(f'Joint {j} torque (Nm)')
            ax.legend(loc='upper right', fontsize=9)
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel('Time (s)')
        plt.tight_layout()
        self._save_or_show(fig, output_file)

    def plot_gru_residual(self, output_file: str = None):
        """GRU correction magnitude over time (zero during warmup)."""
        if 'tau_delan_1' not in self.data.columns:
            print('⚠ tau_delan columns not present — skipping GRU residual plot')
            return
        fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
        fig.suptitle(f'{self.title_prefix}GRU Residual Correction per Joint', fontsize=14, fontweight='bold')
        for j, ax in enumerate(axes, 1):
            residual = self._col('tau_dnn', j) - self._col('tau_delan', j)
            ax.plot(self.t, residual, color='darkorange', linewidth=1.5)
            ax.axhline(0, color='k', linestyle='--', alpha=0.3)
            self._warmup_shade(ax)
            rms_post = np.sqrt(np.mean(residual[self.warmup_end:]**2)) if self.warmup_end < len(residual) else 0
            ax.set_ylabel(f'Joint {j} (Nm)')
            ax.set_title(f'Joint {j}  RMS residual (post-warmup) = {rms_post:.4f} Nm', fontsize=10)
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel('Time (s)')
        plt.tight_layout()
        self._save_or_show(fig, output_file)

    def plot_velocity_tracking(self, output_file: str = None):
        fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
        fig.suptitle(f'{self.title_prefix}DNN Controller — Velocity Tracking', fontsize=14, fontweight='bold')
        for j, ax in enumerate(axes, 1):
            qd_des = self._col('qd_des', j)
            qd_act = self._col('qd_act', j)
            ax.plot(self.t, qd_des, 'b-',  linewidth=2,   label='Desired')
            ax.plot(self.t, qd_act, 'r--', linewidth=1.5, label='Actual', alpha=0.8)
            self._warmup_shade(ax)
            ax.set_ylabel(f'Joint {j} (rad/s)')
            ax.legend(loc='upper right', fontsize=9)
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel('Time (s)')
        plt.tight_layout()
        self._save_or_show(fig, output_file)

    def plot_warmup_effect(self, output_file: str = None):
        """Bar chart: RMS error during warmup vs post-GRU for each joint."""
        err  = self.compute_tracking_errors()
        x    = np.arange(3)
        warm = [err[f'j{j}_rms_warmup']  for j in [1,2,3]]
        post = [err[f'j{j}_rms_postgru'] for j in [1,2,3]]
        w    = 0.35
        fig, ax = plt.subplots(figsize=(8, 5))
        b1 = ax.bar(x - w/2, warm, w, label=f'DeLaN-only (steps 0–{self.warmup_end})',
                    color='steelblue', alpha=0.85)
        b2 = ax.bar(x + w/2, post, w, label=f'DeLaN+GRU  (step {self.warmup_end}+)',
                    color='darkorange', alpha=0.85)
        ax.set_xlabel('Joint')
        ax.set_ylabel('RMS position error (°)')
        ax.set_title(f'{self.title_prefix}Warmup vs Post-GRU Tracking Error', fontsize=13, fontweight='bold')
        ax.set_xticks(x); ax.set_xticklabels(['Joint 1', 'Joint 2', 'Joint 3'])
        ax.legend(); ax.grid(axis='y', alpha=0.3)
        for bar in list(b1) + list(b2):
            ax.text(bar.get_x() + bar.get_width()/2,
                    bar.get_height() + 0.002,
                    f'{bar.get_height():.3f}°', ha='center', va='bottom', fontsize=9)
        plt.tight_layout()
        self._save_or_show(fig, output_file)
    
    def plot_sensed_vs_commanded_torques(self, output_file: str = None):
        """Plot commanded vs sensed torques from Gazebo"""
        if not self.has_sensed_torques():
            print('⚠ Sensed torques not available in log file')
            return
        
        fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
        fig.suptitle(f'{self.title_prefix}Commanded vs Sensed Torques (Gazebo)',
                     fontsize=14, fontweight='bold')
        for j, ax in enumerate(axes, 1):
            tau_cmd = self._col('tau_total', j)
            tau_sen = self._col('tau_sensed', j)
            ax.plot(self.t, tau_cmd, 'b-', linewidth=2, label='Commanded')
            ax.plot(self.t, tau_sen, 'r--', linewidth=1.5, label='Sensed (Gazebo)', alpha=0.8)
            ax.axhline(y=0, color='k', linestyle=':', alpha=0.3)
            self._warmup_shade(ax)
            ax.set_ylabel(f'Joint {j} torque (N⋅m)')
            ax.legend(loc='upper right', fontsize=9)
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel('Time (s)')
        plt.tight_layout()
        self._save_or_show(fig, output_file)
    
    def plot_torque_tracking_error(self, output_file: str = None):
        """Plot error between commanded and sensed torques"""
        if not self.has_sensed_torques():
            print('⚠ Sensed torques not available in log file')
            return
        
        fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
        fig.suptitle(f'{self.title_prefix}Torque Tracking Error (Commanded - Sensed)',
                     fontsize=14, fontweight='bold')
        for j, ax in enumerate(axes, 1):
            tau_cmd = self._col('tau_total', j)
            tau_sen = self._col('tau_sensed', j)
            tau_error = tau_cmd - tau_sen
            ax.plot(self.t, tau_error, color='purple', linewidth=2)
            ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
            ax.fill_between(self.t, tau_error, 0, alpha=0.2, color='purple')
            self._warmup_shade(ax)
            rms = np.sqrt(np.mean(tau_error**2))
            ax.set_ylabel(f'Joint {j} error (N⋅m)')
            ax.set_title(f'Joint {j}  RMS Error: {rms:.4f} N⋅m', fontsize=10)
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel('Time (s)')
        plt.tight_layout()
        self._save_or_show(fig, output_file)

    # ------------------------------------------------------------------ #
    #  CTC comparison                                                      #
    # ------------------------------------------------------------------ #
    def plot_compare_ctc(self, ctc_log: str, output_file: str = None):
        """Overlay tracking errors from a CTC log for direct comparison."""
        ctc_path = Path(ctc_log)
        if not ctc_path.exists():
            print(f'⚠ CTC log not found: {ctc_log}')
            return
        ctc = pd.read_csv(ctc_log)
        fig, axes = plt.subplots(3, 1, figsize=(14, 11), sharex=True)
        fig.suptitle(f'{self.title_prefix}DNN vs CTC — Position Tracking Error Comparison',
                     fontsize=14, fontweight='bold')
        t_ctc = ctc['t'].to_numpy()
        for j, ax in enumerate(axes, 1):
            e_dnn = np.degrees(self.data[f'e_pos_{j}'].to_numpy())
            e_ctc = np.degrees(ctc[f'e_pos_{j}'].to_numpy()) if f'e_pos_{j}' in ctc.columns else None
            ax.plot(self.t, e_dnn, 'r-',  linewidth=1.5,
                    label=f'DNN  RMS={np.sqrt(np.mean(e_dnn**2)):.3f}°')
            if e_ctc is not None:
                ax.plot(t_ctc, e_ctc, 'b--', linewidth=1.5, alpha=0.8,
                        label=f'CTC  RMS={np.sqrt(np.mean(e_ctc**2)):.3f}°')
            ax.axhline(0, color='k', linestyle='--', alpha=0.3)
            self._warmup_shade(ax)
            ax.set_ylabel(f'Joint {j} error (°)')
            ax.legend(loc='upper right', fontsize=9)
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel('Time (s)')
        plt.tight_layout()
        self._save_or_show(fig, output_file)

    # ------------------------------------------------------------------ #
    @staticmethod
    def _save_or_show(fig, output_file, show_after_save: bool = False):
        """Save figure to `output_file` or discard without showing.

        This function never calls `plt.show()` to avoid GUI/backend issues.
        If `output_file` is provided the figure is saved. If `output_file` is
        None the figure is simply closed (no display).
        """
        if output_file:
            Path(output_file).parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(output_file, dpi=150, bbox_inches='tight')
            print(f'  ✓ Saved: {output_file}')
        plt.close(fig)


# ======================================================================== #
#  Entry point                                                              #
# ======================================================================== #
def main():
    parser = argparse.ArgumentParser(
        description='Analyse DNN torque controller performance from log CSV')
    parser.add_argument('log_file',
                        help='Path to DNN log CSV (from torque_publisher_dnn.py)')
    parser.add_argument('--summary',   action='store_true',
                        help='Print performance summary to terminal')
    parser.add_argument('--plots',     action='store_true',
                        help='Generate all plots')
    parser.add_argument('--output-dir', type=str, default=None,
                        help='Directory to save plots (default: src/scripts/plots/)')
    parser.add_argument('--compare-ctc', type=str, default=None, metavar='CTC_LOG',
                        help='Path to a CTC log CSV for side-by-side error comparison')
    # Individual plot overrides
    parser.add_argument('--plot-trajectory',  type=str, metavar='FILE')
    parser.add_argument('--plot-errors',      type=str, metavar='FILE')
    parser.add_argument('--plot-torques',     type=str, metavar='FILE')
    parser.add_argument('--plot-delan-vs-dnn',type=str, metavar='FILE')
    parser.add_argument('--plot-gru-residual',type=str, metavar='FILE')
    parser.add_argument('--plot-velocity',    type=str, metavar='FILE')
    parser.add_argument('--plot-warmup',      type=str, metavar='FILE')
    parser.add_argument('--plot-all-joint-torques', type=str, metavar='FILE',
                        help='Save single-axis plot with all joint total torques')
    parser.add_argument('--plot-sensed-torques', type=str, metavar='FILE',
                        help='Save sensed vs commanded torques plot')
    parser.add_argument('--plot-torque-error', type=str, metavar='FILE',
                        help='Save torque tracking error plot')
    args = parser.parse_args()

    try:
        analyzer = DNNAnalyzer(args.log_file)
    except Exception as e:
        print(f'✗ {e}', file=sys.stderr)
        return 1

    # Resolve output directory and filename prefix
    if args.output_dir:
        out_dir = Path(args.output_dir)
    else:
        out_dir = PLOTS_DIR
    out_dir.mkdir(parents=True, exist_ok=True)
    stem   = Path(args.log_file).stem
    prefix = out_dir / stem

    any_individual = any([
        args.plot_trajectory, args.plot_errors, args.plot_torques,
        args.plot_delan_vs_dnn, args.plot_gru_residual,
        args.plot_velocity, args.plot_warmup,
        args.plot_all_joint_torques,
        args.plot_sensed_torques, args.plot_torque_error,
    ])

    # Always print summary unless suppressed by individual plot-only flags
    if args.summary or not (any_individual or args.plots or args.compare_ctc):
        analyzer.print_summary()

    # If user ran the script without plot flags, save the combined torque breakdown plot
    if not (any_individual or args.plots or args.compare_ctc):
        analyzer.plot_all_joint_torques_same_scale(f'{prefix}_all_joint_torques.png')

    if args.plots or args.output_dir:
        print(f'\nGenerating all plots → {out_dir}/')
        analyzer.plot_trajectory_tracking(f'{prefix}_trajectory.png')
        analyzer.plot_tracking_errors(    f'{prefix}_errors.png')
        analyzer.plot_torque_breakdown(   f'{prefix}_torques.png')
        analyzer.plot_delan_vs_dnn(       f'{prefix}_delan_vs_dnn.png')
        analyzer.plot_gru_residual(       f'{prefix}_gru_residual.png')
        analyzer.plot_velocity_tracking(  f'{prefix}_velocity.png')
        analyzer.plot_warmup_effect(      f'{prefix}_warmup_effect.png')
        analyzer.plot_all_joint_torques_same_scale(f'{prefix}_all_joint_torques.png')
        if analyzer.has_sensed_torques():
            analyzer.plot_sensed_vs_commanded_torques(f'{prefix}_sensed_torques.png')
            analyzer.plot_torque_tracking_error(f'{prefix}_torque_error.png')
        if args.compare_ctc:
            analyzer.plot_compare_ctc(args.compare_ctc,
                                      f'{prefix}_vs_ctc.png')
        # Do not show the saved plots; everything saved above
    else:
        if args.plot_trajectory:   analyzer.plot_trajectory_tracking(args.plot_trajectory)
        if args.plot_errors:       analyzer.plot_tracking_errors(args.plot_errors)
        if args.plot_torques:      analyzer.plot_torque_breakdown(args.plot_torques)
        if args.plot_delan_vs_dnn: analyzer.plot_delan_vs_dnn(args.plot_delan_vs_dnn)
        if args.plot_gru_residual: analyzer.plot_gru_residual(args.plot_gru_residual)
        if args.plot_velocity:     analyzer.plot_velocity_tracking(args.plot_velocity)
        if args.plot_warmup:       analyzer.plot_warmup_effect(args.plot_warmup)
        if args.plot_all_joint_torques: analyzer.plot_all_joint_torques_same_scale(args.plot_all_joint_torques)
        if args.plot_sensed_torques: analyzer.plot_sensed_vs_commanded_torques(args.plot_sensed_torques)
        if args.plot_torque_error: analyzer.plot_torque_tracking_error(args.plot_torque_error)
        if args.compare_ctc:
            out = f'{prefix}_vs_ctc.png' if not any_individual else None
            analyzer.plot_compare_ctc(args.compare_ctc, out)

    return 0


if __name__ == '__main__':
    sys.exit(main())
 