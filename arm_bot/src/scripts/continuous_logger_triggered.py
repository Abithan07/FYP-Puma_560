#!/usr/bin/env python3
"""
Continuous Logger - Triggered Mode
Waits for start/stop signals via ROS2 services for precise timing control.

This version is launched as a subprocess by the torque_publisher and logs only
during the trajectory execution phase with precise timing.

Usage:
    python3 continuous_logger_triggered.py
    
Services:
    /logger/start - Start logging (std_srvs/srv/Trigger)
    /logger/stop - Stop logging (std_srvs/srv/Trigger)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import csv
import os
from datetime import datetime
import signal
import sys


class TriggeredLogger(Node):
    def __init__(self):
        super().__init__('triggered_logger')
        
        # Setup log directory (relative to script location, not hard-coded)
        # This allows the script to work in any workspace
        script_dir = os.path.dirname(os.path.abspath(__file__))
        workspace_root = os.path.abspath(os.path.join(script_dir, '../..'))
        self.log_dir = os.path.join(workspace_root, 'logs')
        
        # Create logs directory if it doesn't exist
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            self.get_logger().info(f'Log directory: {self.log_dir}')
        except Exception as e:
            self.get_logger().error(f'Failed to create log directory: {e}')
            raise
        
        # Generate timestamped filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.test_number = self._get_next_test_number()
        self.csv_filename = os.path.join(
            self.log_dir, 
            f'trajectory_log_{self.test_number}_{timestamp}.csv'
        )
        
        # CSV file (opened when logging starts)
        self.csv_file = None
        self.csv_writer = None
        
        # Logging state
        self.logging_active = False
        self.start_time = None
        self.sample_count = 0
        
        # Subscribe to joint states at high frequency
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10  # QoS depth
        )
        
        # Timer to ensure consistent 100Hz logging
        self.log_timer = self.create_timer(0.01, self.timer_callback)  # 100Hz = 0.01s
        
        # Store latest joint state data
        self.latest_positions = [0.0, 0.0, 0.0]
        self.latest_velocities = [0.0, 0.0, 0.0]
        self.latest_efforts = [0.0, 0.0, 0.0]
        self.data_received = False
        
        # Create services for start/stop control
        self.start_service = self.create_service(
            Trigger,
            '/logger/start',
            self.start_logging_callback
        )
        
        self.stop_service = self.create_service(
            Trigger,
            '/logger/stop',
            self.stop_logging_callback
        )
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        self.get_logger().info('='*70)
        self.get_logger().info('TRIGGERED LOGGER - Ready for Commands')
        self.get_logger().info('='*70)
        self.get_logger().info(f'Log file ready: {self.csv_filename}')
        self.get_logger().info(f'Test number: {self.test_number}')
        self.get_logger().info('Waiting for /logger/start service call...')
        self.get_logger().info('='*70)
    
    def _get_next_test_number(self):
        """Find the next available test number by checking existing files."""
        test_num = 1
        while True:
            pattern = f"trajectory_log_{test_num}_"
            existing_files = [f for f in os.listdir(self.log_dir) if f.startswith(pattern)]
            if not existing_files:
                break
            test_num += 1
        return test_num
    
    def start_logging_callback(self, request, response):
        """Service callback to start logging"""
        if self.logging_active:
            response.success = False
            response.message = "Logging already active"
            return response
        
        # Open CSV file and write header
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write header
        self.csv_writer.writerow([
            'time_elapsed',
            'pos1', 'pos2', 'pos3',
            'vel1', 'vel2', 'vel3',
            'torque1', 'torque2', 'torque3'
        ])
        self.csv_file.flush()
        
        # Reset timing
        self.start_time = self.get_clock().now()
        self.sample_count = 0
        self.logging_active = True
        
        self.get_logger().info('✓ LOGGING STARTED - Recording trajectory data at 100Hz')
        
        response.success = True
        response.message = f"Logging started: {self.csv_filename}"
        return response
    
    def stop_logging_callback(self, request, response):
        """Service callback to stop logging"""
        if not self.logging_active:
            response.success = False
            response.message = "Logging not active"
            return response
        
        self.logging_active = False
        
        # Calculate final statistics
        elapsed = 0.0
        if self.start_time:
            current_time = self.get_clock().now()
            elapsed = (current_time - self.start_time).nanoseconds / 1e9
        
        # Close file
        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        
        self.get_logger().info('='*70)
        self.get_logger().info('✓ LOGGING STOPPED')
        self.get_logger().info('='*70)
        self.get_logger().info(f'File saved: {self.csv_filename}')
        self.get_logger().info(f'Total samples: {self.sample_count}')
        self.get_logger().info(f'Duration: {elapsed:.3f}s')
        self.get_logger().info(f'Average rate: {self.sample_count/elapsed:.1f} Hz' if elapsed > 0 else 'N/A')
        self.get_logger().info('='*70)
        
        response.success = True
        response.message = f"Logged {self.sample_count} samples in {elapsed:.3f}s"
        return response
    
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
        """Log data at consistent 100Hz rate when logging is active"""
        if not self.logging_active or not self.data_received:
            return
        
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
                f'Pos: [{self.latest_positions[0]:.3f}, {self.latest_positions[1]:.3f}, {self.latest_positions[2]:.3f}] rad'
            )
    
    def signal_handler(self, sig, frame):
        """Handle termination signals gracefully"""
        self.get_logger().info('\nReceived termination signal...')
        self.cleanup()
        sys.exit(0)
    
    def cleanup(self):
        """Close CSV file if still open"""
        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
            self.get_logger().info(f'File saved: {self.csv_filename}')


def main(args=None):
    rclpy.init(args=args)
    logger = TriggeredLogger()
    
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
