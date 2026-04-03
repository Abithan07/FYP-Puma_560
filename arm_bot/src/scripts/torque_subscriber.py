#!/usr/bin/env python3
"""
Joint Torque Subscriber
Subscribes to all joint torque command topics and logs data to terminal and CSV file.
Adaptively handles 1-DOF, 2-DOF, and 3-DOF configurations.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import csv
import os
from datetime import datetime
import time


class TorqueSubscriber(Node):
    def __init__(self):
        super().__init__('torque_subscriber')
        
        # Subscribers for all three joint torque commands
        self.sub1 = self.create_subscription(
            Float64MultiArray,
            '/joint_1_controller/commands',
            self.joint1_callback,
            10)
        
        self.sub2 = self.create_subscription(
            Float64MultiArray,
            '/joint_2_controller/commands',
            self.joint2_callback,
            10)
        
        self.sub3 = self.create_subscription(
            Float64MultiArray,
            '/joint_3_controller/commands',
            self.joint3_callback,
            10)
        
        # Store latest torque values
        self.torque1 = None
        self.torque2 = None
        self.torque3 = None
        
        # Track last received time for each joint (for availability detection)
        self.last_received_time = {
            'joint1': None,
            'joint2': None,
            'joint3': None
        }
        
        # Timeout threshold for considering a joint unavailable (seconds)
        self.timeout_threshold = 0.5
        
        # Current configuration mode (3dof, 2dof, 1dof, or checking)
        self.current_mode = 'checking'
        self.mode_last_checked = time.time()
        self.mode_check_interval = 2.0  # Recheck every 2 seconds if in 'checking' mode
        
        # Timestamp of last complete data set
        self.last_log_time = None
        
        # CSV file setup
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        logs_dir = os.path.expanduser('/data/ros2/ros2_ws2/arm_bot/logs')
        os.makedirs(logs_dir, exist_ok=True)
        self.csv_filename = os.path.join(logs_dir, f'torque_log_{timestamp}.csv')
        
        # Initialize CSV file with headers
        with open(self.csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'torque_joint1', 'torque_joint2', 'torque_joint3'])
        
        # Timer to periodically check joint availability and update mode
        self.availability_timer = self.create_timer(0.1, self.check_availability)
        
        self.get_logger().info('='*70)
        self.get_logger().info('JOINT TORQUE SUBSCRIBER STARTED')
        self.get_logger().info('='*70)
        self.get_logger().info(f'Logging to: {self.csv_filename}')
        self.get_logger().info('Subscribing to:')
        self.get_logger().info('  - /joint_1_controller/commands')
        self.get_logger().info('  - /joint_2_controller/commands')
        self.get_logger().info('  - /joint_3_controller/commands')
        self.get_logger().info('='*70)
        self.get_logger().info('Checking joint availability...')
        
    def joint1_callback(self, msg):
        """Callback for joint 1 torque commands"""
        if len(msg.data) > 0:
            self.torque1 = msg.data[0]
            self.last_received_time['joint1'] = time.time()
            self.check_and_log()
    
    def joint2_callback(self, msg):
        """Callback for joint 2 torque commands"""
        if len(msg.data) > 0:
            self.torque2 = msg.data[0]
            self.last_received_time['joint2'] = time.time()
            self.check_and_log()
    
    def joint3_callback(self, msg):
        """Callback for joint 3 torque commands"""
        if len(msg.data) > 0:
            self.torque3 = msg.data[0]
            self.last_received_time['joint3'] = time.time()
            self.check_and_log()
    
    def is_joint_available(self, joint_name):
        """Check if a joint has received data recently"""
        if self.last_received_time[joint_name] is None:
            return False
        time_since_last = time.time() - self.last_received_time[joint_name]
        return time_since_last < self.timeout_threshold
    
    def check_availability(self):
        """Periodically check which joints are available and update mode"""
        joint1_available = self.is_joint_available('joint1')
        joint2_available = self.is_joint_available('joint2')
        joint3_available = self.is_joint_available('joint3')
        
        # Determine the appropriate mode based on joint availability
        new_mode = 'checking'
        
        if joint3_available:
            new_mode = '3dof'
        elif joint2_available:
            new_mode = '2dof'
        elif joint1_available:
            new_mode = '1dof'
        else:
            # No joints available - stay in checking mode
            # Recheck periodically
            current_time = time.time()
            if current_time - self.mode_last_checked > self.mode_check_interval:
                self.mode_last_checked = current_time
                if self.current_mode != 'checking':
                    self.get_logger().warn('No joints available! Rechecking...')
            new_mode = 'checking'
        
        # If mode changed, log the change
        if new_mode != self.current_mode and new_mode != 'checking':
            self.current_mode = new_mode
            self.get_logger().info('='*70)
            if new_mode == '3dof':
                self.get_logger().info('MODE: 3-DOF (Logging all 3 joints)')
            elif new_mode == '2dof':
                self.get_logger().info('MODE: 2-DOF (Logging joints 1 & 2, filling joint 3 with 0)')
            elif new_mode == '1dof':
                self.get_logger().info('MODE: 1-DOF (Logging joint 1 only, filling joints 2 & 3 with 0)')
            self.get_logger().info('='*70)
        elif new_mode == 'checking' and self.current_mode != 'checking':
            self.current_mode = new_mode
            self.get_logger().warn('Lost joint connection! Rechecking availability...')
    
    def check_and_log(self):
        """Log data based on current mode and available joints"""
        # Skip if we're still in checking mode and haven't established a mode yet
        if self.current_mode == 'checking':
            return
        
        # Get current ROS time
        current_time = self.get_clock().now()
        timestamp = current_time.nanoseconds / 1e9  # Convert to seconds
        
        # Prepare values based on current mode
        j1_val = self.torque1 if self.torque1 is not None else 0.0
        j2_val = 0.0
        j3_val = 0.0
        
        if self.current_mode == '3dof':
            # Log all three joints
            if self.torque1 is not None and self.torque2 is not None and self.torque3 is not None:
                j2_val = self.torque2
                j3_val = self.torque3
            else:
                return  # Wait for all three
        
        elif self.current_mode == '2dof':
            # Log joints 1 & 2, fill joint 3 with 0
            if self.torque1 is not None and self.torque2 is not None:
                j2_val = self.torque2
                j3_val = 0.0
            else:
                return  # Wait for joints 1 & 2
        
        elif self.current_mode == '1dof':
            # Log joint 1 only, fill joints 2 & 3 with 0
            if self.torque1 is not None:
                j2_val = 0.0
                j3_val = 0.0
            else:
                return  # Wait for joint 1
        
        # Log to terminal
        mode_indicator = f'[{self.current_mode.upper()}]'
        self.get_logger().info(
            f'{mode_indicator:9s} Torques [Nm]: J1={j1_val:8.4f}, J2={j2_val:8.4f}, J3={j3_val:8.4f} | t={timestamp:.3f}s'
        )
        
        # Log to CSV
        with open(self.csv_filename, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, j1_val, j2_val, j3_val])


def main(args=None):
    rclpy.init(args=args)
    
    node = TorqueSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\nShutting down torque subscriber...')
    finally:
        node.get_logger().info('='*70)
        node.get_logger().info(f'Torque data logged to: {node.csv_filename}')
        node.get_logger().info('='*70)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
