#!/usr/bin/env python3
"""
trajectory_logger.py

Reusable CSV logger for logging joint positions, velocities, and torques during
robot trajectory execution. Automatically manages test numbering and creates
timestamped CSV files.

Features:
- Automatic test numbering (test_1, test_2, etc.)
- CSV format with headers
- Logs: timestamp, joint positions, joint velocities, commanded torques
- Thread-safe logging
"""
import csv
import os
from datetime import datetime
from typing import List, Optional
import threading


class TrajectoryLogger:
    def __init__(self, base_name: str = "trajectory_log", output_dir: str = ".", description: str = ""):
        """
        Initialize the trajectory logger.
        
        Args:
            base_name: Base name for the log files (e.g., "trajectory_log", "sweep_log")
            output_dir: Directory where CSV files will be saved
            description: Optional description to include in the filename
        """
        self.base_name = base_name
        self.output_dir = output_dir
        self.description = description
        self.csv_file = None
        self.csv_writer = None
        self.file_handle = None
        self.lock = threading.Lock()
        self.log_count = 0
        
        # Ensure output directory exists
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Find next available test number
        self.test_number = self._get_next_test_number()
        
    def _get_next_test_number(self) -> int:
        """
        Find the next available test number by checking existing files.
        
        Returns:
            Next available test number
        """
        test_num = 1
        while True:
            # Check if file with this test number exists
            pattern = f"{self.base_name}_test_{test_num}_"
            existing_files = [f for f in os.listdir(self.output_dir) if f.startswith(pattern)]
            if not existing_files:
                break
            test_num += 1
        return test_num
    
    def start_logging(self) -> str:
        """
        Start logging to a new CSV file.
        
        Returns:
            Path to the created CSV file
        """
        with self.lock:
            # Generate filename with test number and timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            if self.description:
                filename = f"{self.base_name}_test_{self.test_number}_{self.description}_{timestamp}.csv"
            else:
                filename = f"{self.base_name}_test_{self.test_number}_{timestamp}.csv"
            
            self.csv_file = os.path.join(self.output_dir, filename)
            
            # Open file and create CSV writer
            self.file_handle = open(self.csv_file, 'w', newline='')
            self.csv_writer = csv.writer(self.file_handle)
            
            # Write header
            header = [
                'timestamp',
                'time_elapsed',
                'joint_1_pos', 'joint_2_pos', 'joint_3_pos',
                'joint_1_vel', 'joint_2_vel', 'joint_3_vel',
                'joint_1_torque', 'joint_2_torque', 'joint_3_torque',
                'phase'
            ]
            self.csv_writer.writerow(header)
            self.file_handle.flush()
            
            self.start_time = datetime.now()
            self.log_count = 0
            
            return self.csv_file
    
    def log_data(self, 
                 positions: List[float],
                 velocities: List[float],
                 torques: List[float],
                 phase: str = ""):
        """
        Log a single data point to the CSV file.
        
        Args:
            positions: List of 3 joint positions [rad]
            velocities: List of 3 joint velocities [rad/s]
            torques: List of 3 commanded torques [Nm]
            phase: Optional phase/state description
        """
        if self.csv_writer is None:
            return
        
        with self.lock:
            current_time = datetime.now()
            timestamp = current_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # millisecond precision
            elapsed = (current_time - self.start_time).total_seconds()
            
            row = [
                timestamp,
                f"{elapsed:.3f}",
                f"{positions[0]:.6f}", f"{positions[1]:.6f}", f"{positions[2]:.6f}",
                f"{velocities[0]:.6f}", f"{velocities[1]:.6f}", f"{velocities[2]:.6f}",
                f"{torques[0]:.6f}", f"{torques[1]:.6f}", f"{torques[2]:.6f}",
                phase
            ]
            
            self.csv_writer.writerow(row)
            self.log_count += 1
            
            # Flush every 10 rows to ensure data is written
            if self.log_count % 10 == 0:
                self.file_handle.flush()
    
    def stop_logging(self) -> Optional[str]:
        """
        Stop logging and close the CSV file.
        
        Returns:
            Path to the completed CSV file, or None if not logging
        """
        with self.lock:
            if self.file_handle is not None:
                self.file_handle.flush()
                self.file_handle.close()
                file_path = self.csv_file
                
                self.file_handle = None
                self.csv_writer = None
                result_file = self.csv_file
                self.csv_file = None
                
                return result_file
            return None
    
    def get_test_number(self) -> int:
        """Get the current test number."""
        return self.test_number
    
    def get_log_count(self) -> int:
        """Get the number of logged data points."""
        return self.log_count
    
    def is_logging(self) -> bool:
        """Check if currently logging."""
        return self.csv_writer is not None


# Convenience function for one-shot logging setup
def create_logger(script_name: str, output_dir: str = ".", description: str = "") -> TrajectoryLogger:
    """
    Create and initialize a trajectory logger.
    
    Args:
        script_name: Name of the calling script (e.g., "move_trajectory", "sweep_joints")
        output_dir: Directory for output files
        description: Optional description
    
    Returns:
        Initialized TrajectoryLogger instance
    """
    logger = TrajectoryLogger(base_name=script_name, output_dir=output_dir, description=description)
    return logger


if __name__ == '__main__':
    # Example usage
    print("TrajectoryLogger Test")
    print("=" * 50)
    
    # Create logger
    logger = create_logger("test_log", output_dir="./logs")
    print(f"Test number: {logger.get_test_number()}")
    
    # Start logging
    csv_file = logger.start_logging()
    print(f"Created CSV file: {csv_file}")
    
    # Log some sample data
    import time
    for i in range(5):
        positions = [0.1 * i, 0.2 * i, 0.3 * i]
        velocities = [0.05, 0.10, 0.15]
        torques = [1.0 + i, 2.0 + i, 3.0 + i]
        logger.log_data(positions, velocities, torques, phase=f"PHASE_{i}")
        time.sleep(0.1)
    
    # Stop logging
    final_file = logger.stop_logging()
    print(f"Logging stopped. File: {final_file}")
    print(f"Total logged points: {logger.get_log_count()}")
    
    # Create another logger to demonstrate test numbering
    logger2 = create_logger("test_log", output_dir="./logs")
    print(f"\nNext test number: {logger2.get_test_number()}")
