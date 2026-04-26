#!/usr/bin/env python3
"""
Computed Torque Control Analysis Tool
Analyzes logged control data to evaluate tracking performance and model accuracy
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import argparse
from typing import Tuple, Dict
import sys

class CTCAnalyzer:
    """Analyze computed torque control performance from logged CSV data"""
    
    def __init__(self, log_file: str):
        """
        Load and parse CTC log file
        
        Args:
            log_file: Path to the CTC log CSV file
        """
        self.log_file = Path(log_file)
        if not self.log_file.exists():
            raise FileNotFoundError(f"Log file not found: {log_file}")
        
        try:
            self.data = pd.read_csv(log_file)
        except Exception as e:
            raise ValueError(f"Failed to parse log file: {e}")
        
        print(f"✓ Loaded log file: {log_file}")
        print(f"  Datapoints: {len(self.data)}")
        print(f"  Duration: {self.data['t'].iloc[-1]:.2f} s")
        print(f"  Columns: {len(self.data.columns)}")
    
    def compute_errors(self) -> Dict[str, np.ndarray]:
        """Compute tracking error metrics"""
        errors = {}
        
        for joint in [1, 2, 3]:
            col_name = f'e_pos_{joint}'
            if col_name in self.data.columns:
                e_pos = self.data[col_name].values
                
                errors[f'joint_{joint}_rms'] = np.sqrt(np.mean(e_pos**2)) * 180/np.pi  # degrees
                errors[f'joint_{joint}_max'] = np.max(np.abs(e_pos)) * 180/np.pi
                errors[f'joint_{joint}_mean'] = np.mean(np.abs(e_pos)) * 180/np.pi
                errors[f'joint_{joint}_std'] = np.std(e_pos) * 180/np.pi
        
        return errors
    
    def compute_feedback_contribution(self) -> Dict[str, float]:
        """Compute relative contribution of feedback vs model-based control"""
        contributions = {}
        
        for joint in [1, 2, 3]:
            tau_model_col = f'tau_model_{joint}'
            tau_fb_col = f'tau_fb_{joint}'
            tau_total_col = f'tau_total_{joint}'
            
            if all(col in self.data.columns for col in [tau_model_col, tau_fb_col, tau_total_col]):
                tau_model = np.abs(self.data[tau_model_col].values)
                tau_fb = np.abs(self.data[tau_fb_col].values)
                tau_total = np.abs(self.data[tau_total_col].values)
                
                # Avoid division by zero
                mask = tau_total > 1e-6
                feedback_pct = 100 * np.mean(tau_fb[mask] / (tau_total[mask] + 1e-10))
                model_pct = 100 * np.mean(tau_model[mask] / (tau_total[mask] + 1e-10))
                
                contributions[f'joint_{joint}_feedback_%'] = feedback_pct
                contributions[f'joint_{joint}_model_%'] = model_pct
                
                # Peak torques
                contributions[f'joint_{joint}_tau_model_peak'] = np.max(tau_model)
                contributions[f'joint_{joint}_tau_fb_peak'] = np.max(tau_fb)
                contributions[f'joint_{joint}_tau_total_peak'] = np.max(tau_total)
        
        return contributions
    
    def compute_model_error_evolution(self) -> Dict[str, np.ndarray]:
        """Analyze how model error changes during trajectory"""
        evolution = {}
        
        for joint in [1, 2, 3]:
            tau_fb_col = f'tau_fb_{joint}'
            if tau_fb_col in self.data.columns:
                # Large feedback indicates model/estimation error
                evolution[f'joint_{joint}_feedback'] = self.data[tau_fb_col].values
        
        return evolution
    
    def print_summary(self):
        """Print performance summary"""
        errors = self.compute_errors()
        contributions = self.compute_feedback_contribution()
        
        print("\n" + "="*80)
        print("COMPUTED TORQUE CONTROL PERFORMANCE SUMMARY")
        print("="*80)
        
        print("\n--- TRACKING ERROR ---")
        for joint in [1, 2, 3]:
            print(f"\nJoint {joint}:")
            print(f"  RMS Error: {errors.get(f'joint_{joint}_rms', 0):.4f}°")
            print(f"  Max Error: {errors.get(f'joint_{joint}_max', 0):.4f}°")
            print(f"  Mean Error: {errors.get(f'joint_{joint}_mean', 0):.4f}°")
            print(f"  Std Dev: {errors.get(f'joint_{joint}_std', 0):.4f}°")
        
        print("\n--- CONTROL TERM CONTRIBUTION ---")
        for joint in [1, 2, 3]:
            print(f"\nJoint {joint}:")
            print(f"  Model-based: {contributions.get(f'joint_{joint}_model_%', 0):.1f}%")
            print(f"  Feedback: {contributions.get(f'joint_{joint}_feedback_%', 0):.1f}%")
            print(f"  Peak tau_model: {contributions.get(f'joint_{joint}_tau_model_peak', 0):.2f} N⋅m")
            print(f"  Peak tau_fb: {contributions.get(f'joint_{joint}_tau_fb_peak', 0):.2f} N⋅m")
            print(f"  Peak tau_total: {contributions.get(f'joint_{joint}_tau_total_peak', 0):.2f} N⋅m")
        
        # Overall assessment
        print("\n--- OVERALL ASSESSMENT ---")
        avg_rms_error = np.mean([errors.get(f'joint_{j}_rms', 0) for j in [1,2,3]])
        avg_feedback = np.mean([contributions.get(f'joint_{j}_feedback_%', 0) for j in [1,2,3]])
        
        print(f"Average RMS Error: {avg_rms_error:.4f}°")
        print(f"Average Feedback Contribution: {avg_feedback:.1f}%")
        
        if avg_rms_error < 0.5:
            print("✓ Excellent tracking performance")
        elif avg_rms_error < 1.0:
            print("✓ Good tracking performance")
        elif avg_rms_error < 2.0:
            print("◐ Acceptable tracking performance")
        else:
            print("✗ Poor tracking performance - Consider tuning gains")
        
        if avg_feedback > 50:
            print("⚠ High feedback contribution - Model may need tuning")
        elif avg_feedback > 20:
            print("◐ Moderate feedback - Normal for real systems")
        else:
            print("✓ Low feedback contribution - Model is accurate")
    
    def plot_trajectory_tracking(self, output_file: str = None):
        """Plot desired vs actual trajectory"""
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        t = self.data['t'].to_numpy()
        
        for joint_idx, ax in enumerate(axes, 1):
            q_des = self.data[f'q_des_{joint_idx}'].to_numpy()
            q_act = self.data[f'q_act_{joint_idx}'].to_numpy()
            ax.plot(t, q_des, 'b-', label='Desired', linewidth=2)
            ax.plot(t, q_act, 'r--', label='Actual', alpha=0.7, linewidth=1.5)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(f'Position Joint {joint_idx} (rad)')
            ax.legend()
            ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        if output_file:
            plt.savefig(output_file, dpi=150)
            print(f"✓ Saved: {output_file}")
        else:
            plt.show()
    
    def plot_tracking_errors(self, output_file: str = None):
        """Plot tracking errors over time"""
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        t = self.data['t'].to_numpy()
        
        for joint_idx, ax in enumerate(axes, 1):
            e_pos = self.data[f'e_pos_{joint_idx}'].values * 180/np.pi  # Convert to degrees
            ax.plot(t, e_pos, 'r-', linewidth=2)
            ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(f'Position Error Joint {joint_idx} (°)')
            ax.grid(True, alpha=0.3)
            ax.set_title(f'Joint {joint_idx} - RMS Error: {np.sqrt(np.mean(e_pos**2)):.4f}°')
        
        plt.tight_layout()
        if output_file:
            plt.savefig(output_file, dpi=150)
            print(f"✓ Saved: {output_file}")
        else:
            plt.show()
    
    def plot_torque_breakdown(self, output_file: str = None):
        """Plot model-based, feedback, and total torques"""
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        t = self.data['t'].to_numpy()
        
        for joint_idx, ax in enumerate(axes, 1):
            tau_model = self.data[f'tau_model_{joint_idx}'].to_numpy()
            tau_fb = self.data[f'tau_fb_{joint_idx}'].to_numpy()
            tau_total = self.data[f'tau_total_{joint_idx}'].to_numpy()
            ax.plot(t, tau_model, 'b-', label='Model', linewidth=2)
            ax.plot(t, tau_fb, 'g-', label='Feedback', linewidth=1.5, alpha=0.7)
            ax.plot(t, tau_total, 'r-', label='Total', linewidth=2, alpha=0.8)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(f'Torque Joint {joint_idx} (N⋅m)')
            ax.legend()
            ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        if output_file:
            plt.savefig(output_file, dpi=150)
            print(f"✓ Saved: {output_file}")
        else:
            plt.show()
    
    def plot_velocity_tracking(self, output_file: str = None):
        """Plot desired vs actual velocity"""
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        t = self.data['t'].to_numpy()
        
        for joint_idx, ax in enumerate(axes, 1):
            qd_des = self.data[f'qd_des_{joint_idx}'].to_numpy()
            qd_act = self.data[f'qd_act_{joint_idx}'].to_numpy()
            ax.plot(t, qd_des, 'b-', label='Desired', linewidth=2)
            ax.plot(t, qd_act, 'r--', label='Actual', alpha=0.7, linewidth=1.5)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(f'Velocity Joint {joint_idx} (rad/s)')
            ax.legend()
            ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        if output_file:
            plt.savefig(output_file, dpi=150)
            print(f"✓ Saved: {output_file}")
        else:
            plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Analyze Computed Torque Control performance from logged data'
    )
    parser.add_argument('log_file', help='Path to CTC log CSV file')
    parser.add_argument('--summary', action='store_true', help='Print performance summary')
    parser.add_argument('--plots', action='store_true', help='Generate all plots')
    parser.add_argument('--plot-trajectory', type=str, metavar='FILE', help='Save trajectory plot')
    parser.add_argument('--plot-errors', type=str, metavar='FILE', help='Save error plot')
    parser.add_argument('--plot-torques', type=str, metavar='FILE', help='Save torque breakdown plot')
    parser.add_argument('--plot-velocity', type=str, metavar='FILE', help='Save velocity plot')
    parser.add_argument('--output-dir', type=str, default=None, help='Directory to save all plots')
    
    args = parser.parse_args()
    
    try:
        analyzer = CTCAnalyzer(args.log_file)
    except Exception as e:
        print(f"✗ Error: {e}", file=sys.stderr)
        return 1
    
    # Determine output directory
    if args.output_dir:
        Path(args.output_dir).mkdir(parents=True, exist_ok=True)
        prefix = Path(args.output_dir) / Path(args.log_file).stem
    else:
        prefix = Path(args.log_file).stem
    
    # Print summary
    if args.summary or not (args.plot_trajectory or args.plot_errors or args.plot_torques or args.plot_velocity or args.plots):
        analyzer.print_summary()
    
    # Generate plots
    if args.plots or args.output_dir:
        analyzer.plot_trajectory_tracking(f'{prefix}_trajectory.png')
        analyzer.plot_tracking_errors(f'{prefix}_errors.png')
        analyzer.plot_torque_breakdown(f'{prefix}_torques.png')
        analyzer.plot_velocity_tracking(f'{prefix}_velocity.png')
    else:
        if args.plot_trajectory:
            analyzer.plot_trajectory_tracking(args.plot_trajectory)
        if args.plot_errors:
            analyzer.plot_tracking_errors(args.plot_errors)
        if args.plot_torques:
            analyzer.plot_torque_breakdown(args.plot_torques)
        if args.plot_velocity:
            analyzer.plot_velocity_tracking(args.plot_velocity)
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
