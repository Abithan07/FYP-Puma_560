#!/usr/bin/env python3
"""
Batch processor for PUMA-560 inverse dynamics calculations
Integrates trajectory generation and dynamics computation in one workflow
"""

import argparse
import subprocess
import sys
from pathlib import Path
import shutil


def main():
    parser = argparse.ArgumentParser(
        description="Batch process PUMA-560 trajectories and compute inverse dynamics"
    )
    
    parser.add_argument(
        "mode",
        choices=["generate", "dynamics", "full"],
        help="Mode: 'generate' (trajectories only), 'dynamics' (inverse dynamics only), 'full' (both)"
    )
    
    parser.add_argument(
        "--num-paths",
        type=int,
        help="Number of trajectories to generate (for 'generate' or 'full' mode)"
    )
    
    parser.add_argument(
        "--start-id",
        type=int,
        default=601,
        help="Starting path ID (default: 601)"
    )
    
    parser.add_argument(
        "--base-dir",
        type=str,
        default="/home/priyankan/Desktop/FYP-Puma_560/Dataset",
        help="Base dataset directory"
    )
    
    parser.add_argument(
        "--traj-dir",
        type=str,
        default=None,
        help="Trajectory directory (default: {base-dir}/Trajectories)"
    )
    
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Output directory (default: {base-dir}/Joint_states)"
    )
    
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed for trajectory generation"
    )
    
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Show plots during trajectory generation"
    )
    
    parser.add_argument(
        "--v-max",
        type=float,
        default=2.0,
        help="Maximum joint velocity (rad/s)"
    )
    
    parser.add_argument(
        "--a-max",
        type=float,
        default=7.0,
        help="Maximum joint acceleration (rad/s²)"
    )
    
    args = parser.parse_args()
    
    # Set default directories
    if args.traj_dir is None:
        args.traj_dir = str(Path(args.base_dir) / "Trajectories")
    
    if args.output_dir is None:
        args.output_dir = str(Path(args.base_dir) / "Joint_states")
    
    # Determine working directory
    workspace_root = Path(__file__).parent
    
    # Generate trajectories if needed
    if args.mode in ["generate", "full"]:
        if args.num_paths is None:
            print("Error: --num-paths required for 'generate' or 'full' mode")
            sys.exit(1)
        
        print(f"\n{'='*60}")
        print(f"GENERATING {args.num_paths} TRAJECTORIES")
        print(f"{'='*60}")
        
        cmd = [
            "python3",
            "trajectory_generator.py",
            str(args.num_paths),
            "--start-id", str(args.start_id),
            "--base-dir", args.base_dir,
            "--v-max", str(args.v_max),
            "--a-max", str(args.a_max)
        ]
        
        if args.seed is not None:
            cmd.extend(["--seed", str(args.seed)])
        
        if args.plot:
            cmd.append("--plot")
        
        result = subprocess.run(cmd, cwd=workspace_root)
        if result.returncode != 0:
            print("Error: Trajectory generation failed")
            sys.exit(1)
    
    # Compute inverse dynamics if needed
    if args.mode in ["dynamics", "full"]:
        print(f"\n{'='*60}")
        print(f"COMPUTING INVERSE DYNAMICS")
        print(f"{'='*60}")
        
        # Check if trajectory directory exists and has files
        traj_path = Path(args.traj_dir)
        if not traj_path.exists():
            print(f"Error: Trajectory directory not found: {args.traj_dir}")
            sys.exit(1)
        
        traj_files = list(traj_path.glob("*_traj.csv"))
        if not traj_files:
            print(f"Error: No trajectory files found in {args.traj_dir}")
            sys.exit(1)
        
        print(f"Found {len(traj_files)} trajectory files to process")
        
        cmd = [
            "python3",
            "inverse_dynamics_fast.py",
            args.traj_dir,
            "-o", args.output_dir
        ]
        
        result = subprocess.run(cmd, cwd=workspace_root)
        if result.returncode != 0:
            print("Error: Inverse dynamics computation failed")
            sys.exit(1)
        
        # Summary statistics
        output_path = Path(args.output_dir)
        output_files = list(output_path.glob("*_joint_states.csv"))
        print(f"\n{'='*60}")
        print(f"PROCESSING COMPLETE")
        print(f"{'='*60}")
        print(f"Generated {len(output_files)} joint state files")
        print(f"Output directory: {args.output_dir}")


if __name__ == "__main__":
    main()
