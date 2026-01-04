#!/usr/bin/env python3
"""
trajectory_logger_enhanced.py

Enhanced CSV logger for logging commanded vs actual torques and positions.

Compact format:
- time_elapsed, exp_pos1, act_pos1, exp_pos2, act_pos2, exp_pos3, act_pos3
- cmd_tq1, sen_tq1, cmd_tq2, sen_tq2, cmd_tq3, sen_tq3
- err_tq1, err_tq2, err_tq3, phase

Features:
- Automatic test numbering
- Thread-safe logging
"""
import csv
import os
from datetime import datetime
from typing import List, Optional
import threading


class TrajectoryLoggerEnhanced:
    def __init__(self, base_name: str = "trajectory_log", output_dir: str = ".", description: str = ""):
        """
        Initialize the enhanced trajectory logger.
        
        Args:
            base_name: Base name for the log files
            output_dir: Directory where CSV files will be saved
            description: Optional description
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
        """Find the next available test number by checking existing files."""
        test_num = 1
        while True:
            pattern = f"{self.base_name}_test_{test_num}_"
            existing_files = [f for f in os.listdir(self.output_dir) if f.startswith(pattern)]
            if not existing_files:
                break
            test_num += 1
        return test_num
    
    def start_logging(self) -> str:
        """Start logging to a new CSV file."""
        with self.lock:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            if self.description:
                filename = f"{self.base_name}_test_{self.test_number}_{self.description}_{timestamp}.csv"
            else:
                filename = f"{self.base_name}_test_{self.test_number}_{timestamp}.csv"
            
            self.csv_file = os.path.join(self.output_dir, filename)
            
            # Open file and create CSV writer
            self.file_handle = open(self.csv_file, 'w', newline='')
            self.csv_writer = csv.writer(self.file_handle)
            
            # Compact header format as requested
            # Format: time_elapsed, exp_pos1, act_pos1, exp_pos2, act_pos2, exp_pos3, act_pos3,
            #         cmd_tq1, sen_tq1, cmd_tq2, sen_tq2, cmd_tq3, sen_tq3, err_tq1, err_tq2, err_tq3, phase
            header = [
                'time_elapsed',
                'exp_pos1', 'act_pos1', 
                'exp_pos2', 'act_pos2', 
                'exp_pos3', 'act_pos3',
                'cmd_tq1', 'sen_tq1', 
                'cmd_tq2', 'sen_tq2', 
                'cmd_tq3', 'sen_tq3',
                'err_tq1', 'err_tq2', 'err_tq3',
                'phase'
            ]
            self.csv_writer.writerow(header)
            self.file_handle.flush()
            
            self.start_time = datetime.now()
            self.log_count = 0
            
            return self.csv_file
    
    def log_data(self, 
                 desired_positions: List[float],
                 actual_positions: List[float],
                 velocities: List[float],
                 commanded_torques: List[float],
                 actual_torques: Optional[List[float]] = None,
                 phase: str = ""):
        """
        Log a single data point with commanded vs actual comparison.
        
        Args:
            desired_positions: Desired joint positions from trajectory [rad]
            actual_positions: Actual joint positions from sensors [rad]
            velocities: Joint velocities [rad/s]
            commanded_torques: Commanded torques sent to controller [Nm]
            actual_torques: Optional actual torques from effort sensors [Nm]
            phase: Phase/state description
        """
        if self.csv_writer is None:
            return
        
        with self.lock:
            current_time = datetime.now()
            elapsed = (current_time - self.start_time).total_seconds()
            
            # Calculate torque errors if actual torques provided
            if actual_torques is not None:
                tau_errors = [commanded_torques[i] - actual_torques[i] for i in range(3)]
                act_tau = actual_torques
            else:
                tau_errors = [0.0, 0.0, 0.0]
                act_tau = [0.0, 0.0, 0.0]
            
            # Compact format: time_elapsed, exp_pos1, act_pos1, exp_pos2, act_pos2, exp_pos3, act_pos3,
            #                 cmd_tq1, sen_tq1, cmd_tq2, sen_tq2, cmd_tq3, sen_tq3, err_tq1, err_tq2, err_tq3, phase
            row = [
                f"{elapsed:.3f}",
                f"{desired_positions[0]:.6f}", f"{actual_positions[0]:.6f}",
                f"{desired_positions[1]:.6f}", f"{actual_positions[1]:.6f}",
                f"{desired_positions[2]:.6f}", f"{actual_positions[2]:.6f}",
                f"{commanded_torques[0]:.6f}", f"{act_tau[0]:.6f}",
                f"{commanded_torques[1]:.6f}", f"{act_tau[1]:.6f}",
                f"{commanded_torques[2]:.6f}", f"{act_tau[2]:.6f}",
                f"{tau_errors[0]:.6f}", f"{tau_errors[1]:.6f}", f"{tau_errors[2]:.6f}",
                phase
            ]
            
            self.csv_writer.writerow(row)
            self.log_count += 1
            
            # Flush every 10 rows
            if self.log_count % 10 == 0:
                self.file_handle.flush()
    
    def stop_logging(self) -> Optional[str]:
        """Stop logging and close the CSV file."""
        with self.lock:
            if self.file_handle is not None:
                self.file_handle.flush()
                self.file_handle.close()
                
                result_file = self.csv_file
                self.file_handle = None
                self.csv_writer = None
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
