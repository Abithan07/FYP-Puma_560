#!/usr/bin/env python3
"""
Continuous Logger - Standalone Script
Continuously logs joint positions and torques at 100Hz to CSV file.

This script runs independently from the torque publisher and can be started
in a separate terminal before running the torque publisher to capture all data.

Usage:
    ros2 run arm_bot continuous_logger.py
    or
    python3 continuous_logger.py

The script will:
- Automatically create a timestamped log file in the logs/ directory
- Log position and effort (torque) values at 100Hz
- Continue running until interrupted with Ctrl+C
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import csv
import os
from datetime import datetime
import signal
import sys


class ContinuousLogger(Node):
    def __init__(self):
        super().__init__('continuous_logger')
        
        # Setup log directory
        self.log_dir = '/data/ros2/ros2_ws2/arm_bot/logs'
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Generate timestamped filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.test_number = self._get_next_test_number()
        self.csv_filename = os.path.join(
            self.log_dir, 
            f'continuous_log_{self.test_number}_{timestamp}.csv'
        )
        
        # Open CSV file and write header
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write header
        # Format: time_elapsed, pos1, pos2, pos3, vel1, vel2, vel3, torque1, torque2, torque3
        self.csv_writer.writerow([
            'time_elapsed',
            'pos1', 'pos2', 'pos3',
            'vel1', 'vel2', 'vel3',
            'torque1', 'torque2', 'torque3'
        ])
        self.csv_file.flush()
        
        # Start time for elapsed time calculation
        self.start_time = None
        self.sample_count = 0
        
        # Subscribe to joint states at high frequency
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10  # QoS depth
        )
        
        # Timer to ensure consistent 100Hz logging (even if messages arrive faster/slower)
        self.log_timer = self.create_timer(0.01, self.timer_callback)  # 100Hz = 0.01s
        
        # Store latest joint state data
        self.latest_positions = [0.0, 0.0, 0.0]
        self.latest_velocities = [0.0, 0.0, 0.0]
        self.latest_efforts = [0.0, 0.0, 0.0]
        self.data_received = False
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.get_logger().info('='*70)
        self.get_logger().info('CONTINUOUS LOGGER - 100Hz Data Acquisition')
        self.get_logger().info('='*70)
        self.get_logger().info(f'Log file: {self.csv_filename}')
        self.get_logger().info(f'Test number: {self.test_number}')
        self.get_logger().info('Logging format: time_elapsed, pos1, pos2, pos3, vel1, vel2, vel3, torque1, torque2, torque3')
        self.get_logger().info('Press Ctrl+C to stop logging')
        self.get_logger().info('='*70)
    
    def _get_next_test_number(self):
        """Find the next available test number by checking existing files."""
        test_num = 1
        while True:
            pattern = f"continuous_log_{test_num}_"
            existing_files = [f for f in os.listdir(self.log_dir) if f.startswith(pattern)]
            if not existing_files:
                break
            test_num += 1
        return test_num
    
    def joint_state_callback(self, msg):
        """Store the latest joint state data"""
        try:
            # Find indices for joints 1, 2, 3
            idx1 = msg.name.index('joint_1')
            idx2 = msg.name.index('joint_2')
            idx3 = msg.name.index('joint_3')
            
            # Extract positions
            self.latest_positions = [
                msg.position[idx1],
                msg.position[idx2],
                msg.position[idx3]
            ]
            
            # Extract velocities
            if len(msg.velocity) >= 3:
                self.latest_velocities = [
                    msg.velocity[idx1],
                    msg.velocity[idx2],
                    msg.velocity[idx3]
                ]
            
            # Extract efforts (torques)
            if len(msg.effort) >= 3:
                self.latest_efforts = [
                    msg.effort[idx1],
                    msg.effort[idx2],
                    msg.effort[idx3]
                ]
            
            self.data_received = True
            
        except (ValueError, IndexError) as e:
            self.get_logger().warning(f'Error parsing joint states: {e}')
    
    def timer_callback(self):
        """Log data at consistent 100Hz rate"""
        if not self.data_received:
            return
        
        # Initialize start time on first data
        if self.start_time is None:
            self.start_time = self.get_clock().now()
            self.get_logger().info('Started logging data...')
        
        # Calculate elapsed time
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9  # Convert to seconds
        
        # Write data row
        row = [
            f"{elapsed:.3f}",
            f"{self.latest_positions[0]:.6f}",
            f"{self.latest_positions[1]:.6f}",
            f"{self.latest_positions[2]:.6f}",
            f"{self.latest_velocities[0]:.6f}",
            f"{self.latest_velocities[1]:.6f}",
            f"{self.latest_velocities[2]:.6f}",
            f"{self.latest_efforts[0]:.6f}",
            f"{self.latest_efforts[1]:.6f}",
            f"{self.latest_efforts[2]:.6f}"
        ]
        
        self.csv_writer.writerow(row)
        self.sample_count += 1
        
        # Flush every 100 samples (1 second)
        if self.sample_count % 100 == 0:
            self.csv_file.flush()
            self.get_logger().info(
                f't={elapsed:.1f}s | Samples: {self.sample_count} | '
                f'Pos: [{self.latest_positions[0]:.3f}, {self.latest_positions[1]:.3f}, {self.latest_positions[2]:.3f}] rad | '
                f'Torque: [{self.latest_efforts[0]:.2f}, {self.latest_efforts[1]:.2f}, {self.latest_efforts[2]:.2f}] Nm'
            )
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        self.get_logger().info('\n' + '='*70)
        self.get_logger().info('Stopping logger...')
        self.cleanup()
        sys.exit(0)
    
    def cleanup(self):
        """Close CSV file and print summary"""
        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
            
            elapsed = 0.0
            if self.start_time:
                current_time = self.get_clock().now()
                elapsed = (current_time - self.start_time).nanoseconds / 1e9
            
            self.get_logger().info('='*70)
            self.get_logger().info('LOGGING COMPLETE')
            self.get_logger().info('='*70)
            self.get_logger().info(f'File saved: {self.csv_filename}')
            self.get_logger().info(f'Total samples: {self.sample_count}')
            self.get_logger().info(f'Duration: {elapsed:.2f}s')
            self.get_logger().info(f'Average rate: {self.sample_count/elapsed:.1f} Hz' if elapsed > 0 else 'N/A')
            self.get_logger().info('='*70)


def main(args=None):
    rclpy.init(args=args)
    logger = ContinuousLogger()
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        logger.cleanup()
        logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
