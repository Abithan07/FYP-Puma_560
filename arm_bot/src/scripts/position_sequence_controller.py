#!/usr/bin/env python3
"""
Position Sequence Controller - PID Control with CSV Position Input
Moves robot arm to positions specified in a CSV file using PID control.
Each position has a 5-second execution window with 100Hz logging.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import csv
import math
import time
import os
from datetime import datetime


class PositionSequenceController(Node):
    def __init__(self, positions_csv_path, max_torques=[100.0, 100.0, 50.0], output_dir='./dataset'):
        """
        Initialize position sequence controller.
        
        Args:
            positions_csv_path: Path to CSV file with target positions (pos1, pos2, pos3 per row)
            max_torques: Maximum torque limits for each joint [tau1_max, tau2_max, tau3_max]
            output_dir: Directory for output log files
        """
        super().__init__('position_sequence_controller')
        
        # Publishers for torque commands
        self.pub1 = self.create_publisher(Float64MultiArray, '/joint_1_controller/commands', 10)
        self.pub2 = self.create_publisher(Float64MultiArray, '/joint_2_controller/commands', 10)
        self.pub3 = self.create_publisher(Float64MultiArray, '/joint_3_controller/commands', 10)
        
        # Subscriber to monitor joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Load target positions from CSV
        self.positions_csv_path = positions_csv_path
        self.target_positions = self.load_positions_csv()
        
        # Maximum torque limits per joint
        self.max_torques = max_torques
        
        # Output directory for log files
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Pre-allocate message objects
        self.msg1 = Float64MultiArray()
        self.msg2 = Float64MultiArray()
        self.msg3 = Float64MultiArray()
        
        # State variables - sensor readings
        self.current_joint_pos = [0.0, 0.0, 0.0]
        self.current_joint_vel = [0.0, 0.0, 0.0]
        self.joint_states_received = False
        
        # PID controller parameters (tuned for general use)
        self.kp = [50.0, 200.0, 150.0]    # Proportional gains
        self.ki = [5.0, 25.0, 20.0]       # Integral gains
        self.kd = [12.0, 35.0, 10.0]      # Derivative gains
        
        # PID state variables
        self.integral_error = [0.0, 0.0, 0.0]
        
        # Control parameters
        self.control_rate = 100  # Hz
        self.dt = 1.0 / self.control_rate  # 0.01s
        self.execution_window = 5.0  # seconds per position
        self.max_iterations = int(self.execution_window * self.control_rate)  # 500 iterations
        
        # Current motion state
        self.current_position_idx = 0
        self.current_iteration = 0
        self.motion_active = False
        self.motion_timer = None
        
        # Logging variables
        self.log_data = []
        self.current_torques = [0.0, 0.0, 0.0]
        self.dataset_counter = 1
        
        self.get_logger().info('='*70)
        self.get_logger().info('POSITION SEQUENCE CONTROLLER - PID WITH CSV INPUT')
        self.get_logger().info('='*70)
        self.get_logger().info(f'Positions CSV: {self.positions_csv_path}')
        self.get_logger().info(f'Number of target positions: {len(self.target_positions)}')
        self.get_logger().info(f'Max torques: {self.max_torques} Nm')
        self.get_logger().info(f'Execution window: {self.execution_window} s per position')
        self.get_logger().info(f'Control rate: {self.control_rate} Hz')
        self.get_logger().info(f'Output directory: {self.output_dir}')
        self.get_logger().info(f'PID gains - Kp: {self.kp}, Ki: {self.ki}, Kd: {self.kd}')
    
    def load_positions_csv(self):
        """Load target positions from CSV file"""
        positions = []
        try:
            with open(self.positions_csv_path, 'r') as f:
                reader = csv.reader(f)
                # Skip header if present
                first_row = next(reader, None)
                if first_row:
                    # Check if first row is header or data
                    try:
                        pos = [float(val) for val in first_row]
                        positions.append(pos)
                    except ValueError:
                        # First row is header, skip it
                        pass
                
                # Read remaining rows
                for row in reader:
                    if row and len(row) >= 3:
                        try:
                            pos = [float(row[0]), float(row[1]), float(row[2])]
                            positions.append(pos)
                        except ValueError:
                            self.get_logger().warning(f'Invalid row in CSV: {row}')
            
            self.get_logger().info(f'Loaded {len(positions)} target positions from CSV')
            return positions
        except Exception as e:
            self.get_logger().error(f'Failed to load positions CSV: {e}')
            return []
    
    def joint_state_callback(self, msg):
        """Update joint state variables from sensors"""
        try:
            idx1 = msg.name.index('joint_1')
            idx2 = msg.name.index('joint_2')
            idx3 = msg.name.index('joint_3')
            
            self.current_joint_pos = [
                msg.position[idx1],
                msg.position[idx2],
                msg.position[idx3]
            ]
            
            if len(msg.velocity) >= 3:
                self.current_joint_vel = [
                    msg.velocity[idx1],
                    msg.velocity[idx2],
                    msg.velocity[idx3]
                ]
            
            self.joint_states_received = True
        except (ValueError, IndexError):
            pass
    
    def calculate_gravity_compensation(self):
        """Calculate gravity compensation torques"""
        q2 = self.current_joint_pos[1]
        q3 = self.current_joint_pos[2]
        
        gravity_comp = [
            0.0,
            -44.0 * math.cos(q2),
            -12.0 * math.cos(q2 + q3)
        ]
        return gravity_comp
    
    def pid_control(self, target_pos):
        """
        Calculate PID control torques.
        
        Args:
            target_pos: Target position [pos1, pos2, pos3] in radians
        
        Returns:
            Saturated torques [tau1, tau2, tau3]
        """
        # Calculate position errors
        errors = [target_pos[i] - self.current_joint_pos[i] for i in range(3)]
        
        # Update integral error with anti-windup
        max_integral = [0.5, 1.0, 1.0]
        for i in range(3):
            self.integral_error[i] += errors[i] * self.dt
            self.integral_error[i] = max(-max_integral[i], min(max_integral[i], self.integral_error[i]))
        
        # Gravity compensation
        gravity_comp = self.calculate_gravity_compensation()
        
        # PID control law: τ = Kp*e + Ki*∫e - Kd*v + g(q)
        torques = [
            self.kp[i] * errors[i] + 
            self.ki[i] * self.integral_error[i] - 
            self.kd[i] * self.current_joint_vel[i] + 
            gravity_comp[i]
            for i in range(3)
        ]
        
        # Saturate torques to max limits
        torques = [
            max(-self.max_torques[i], min(self.max_torques[i], torques[i]))
            for i in range(3)
        ]
        
        return torques
    
    def init_log_file(self):
        """Initialize log data for current motion"""
        self.log_data = []
    
    def log_data_point(self, elapsed_time, target_pos):
        """
        Log a single data point.
        
        Args:
            elapsed_time: Time since motion start
            target_pos: Current target position
        """
        row = {
            't': elapsed_time,
            'tau1': self.current_torques[0],
            'tau2': self.current_torques[1],
            'tau3': self.current_torques[2],
            'dp1': self.current_joint_pos[0],
            'dp2': self.current_joint_pos[1],
            'dp3': self.current_joint_pos[2],
            'dv1': self.current_joint_vel[0],
            'dv2': self.current_joint_vel[1],
            'dv3': self.current_joint_vel[2]
        }
        self.log_data.append(row)
    
    def save_log_file(self):
        """Save logged data to CSV file"""
        filename = os.path.join(self.output_dir, f'dataset_{self.dataset_counter}.csv')
        
        try:
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                # Write header
                header = ['t', 'tau1', 'tau2', 'tau3', 'dp1', 'dp2', 'dp3', 'dv1', 'dv2', 'dv3']
                writer.writerow(header)
                
                # Write data rows
                for row in self.log_data:
                    writer.writerow([
                        f'{row["t"]:.4f}',
                        f'{row["tau1"]:.4f}',
                        f'{row["tau2"]:.4f}',
                        f'{row["tau3"]:.4f}',
                        f'{row["dp1"]:.4f}',
                        f'{row["dp2"]:.4f}',
                        f'{row["dp3"]:.4f}',
                        f'{row["dv1"]:.4f}',
                        f'{row["dv2"]:.4f}',
                        f'{row["dv3"]:.4f}'
                    ])
            
            self.get_logger().info(f'✓ Saved log file: {filename} ({len(self.log_data)} rows)')
            self.dataset_counter += 1
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save log file: {e}')
            return False
    
    def motion_callback(self):
        """Timer callback for motion execution at 100Hz"""
        if self.current_iteration >= self.max_iterations:
            # Motion window complete (5 seconds)
            self.complete_current_motion()
            return
        
        # Get current target position
        target_pos = self.target_positions[self.current_position_idx]
        
        # Calculate elapsed time
        elapsed_time = self.current_iteration * self.dt
        
        # Calculate PID control torques
        torques = self.pid_control(target_pos)
        self.current_torques = torques
        
        # Publish torques
        self.msg1.data = [torques[0]]
        self.msg2.data = [torques[1]]
        self.msg3.data = [torques[2]]
        
        self.pub1.publish(self.msg1)
        self.pub2.publish(self.msg2)
        self.pub3.publish(self.msg3)
        
        # Log data point
        self.log_data_point(elapsed_time, target_pos)
        
        # Increment iteration counter
        self.current_iteration += 1
        
        # Console log every 100 iterations (1 second)
        if self.current_iteration % 100 == 0:
            errors = [target_pos[i] - self.current_joint_pos[i] for i in range(3)]
            max_error = max(abs(e) for e in errors)
            max_error_deg = max_error * 180 / math.pi
            
            self.get_logger().info(
                f'Position {self.current_position_idx + 1}/{len(self.target_positions)} | '
                f't={elapsed_time:.1f}s | '
                f'Max Error: {max_error_deg:.2f}° | '
                f'τ=[{torques[0]:.1f}, {torques[1]:.1f}, {torques[2]:.1f}] Nm'
            )
    
    def complete_current_motion(self):
        """Complete current motion and move to next position"""
        # Cancel current timer
        if self.motion_timer:
            self.motion_timer.cancel()
            self.motion_timer = None
        
        # Save log file
        self.save_log_file()
        
        # Move to next position
        self.current_position_idx += 1
        
        if self.current_position_idx >= len(self.target_positions):
            # All positions completed
            self.get_logger().info('='*70)
            self.get_logger().info('ALL POSITIONS COMPLETED')
            self.get_logger().info('='*70)
            self.motion_active = False
            return
        
        # Start next motion
        self.start_next_motion()
    
    def start_next_motion(self):
        """Start motion to next target position"""
        target_pos = self.target_positions[self.current_position_idx]
        
        self.get_logger().info('='*70)
        self.get_logger().info(f'MOTION {self.current_position_idx + 1}/{len(self.target_positions)}')
        self.get_logger().info(f'Target: [{target_pos[0]:.4f}, {target_pos[1]:.4f}, {target_pos[2]:.4f}] rad')
        self.get_logger().info(f'Current: [{self.current_joint_pos[0]:.4f}, {self.current_joint_pos[1]:.4f}, {self.current_joint_pos[2]:.4f}] rad')
        self.get_logger().info('='*70)
        
        # Reset state for new motion
        self.current_iteration = 0
        self.integral_error = [0.0, 0.0, 0.0]
        self.init_log_file()
        
        # Create timer for motion execution at 100Hz
        self.motion_timer = self.create_timer(self.dt, self.motion_callback)
    
    def run(self):
        """Main execution sequence"""
        if not self.target_positions:
            self.get_logger().error('No target positions loaded! Exiting.')
            return
        
        self.get_logger().info('Waiting for joint states...')
        
        # Wait for joint states
        while not self.joint_states_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.joint_states_received:
            self.get_logger().error('Failed to receive joint states!')
            return
        
        self.get_logger().info(f'✓ Joint states received')
        self.get_logger().info(f'Initial position: [{self.current_joint_pos[0]:.4f}, {self.current_joint_pos[1]:.4f}, {self.current_joint_pos[2]:.4f}] rad')
        
        # Start execution
        self.get_logger().info('='*70)
        self.get_logger().info('STARTING POSITION SEQUENCE EXECUTION')
        self.get_logger().info('='*70)
        
        self.motion_active = True
        self.current_position_idx = 0
        
        # Start first motion
        self.start_next_motion()
        
        # Spin until all motions complete
        while self.motion_active and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)
        
        self.get_logger().info('Position sequence controller shutting down.')


def main(args=None):
    rclpy.init(args=args)
    
    # =====================================================================
    # CONFIGURATION - Modify these parameters as needed
    # =====================================================================
    
    # Path to CSV file with target positions (pos1, pos2, pos3 per row)
    positions_csv_path = os.path.expanduser('/data/ros2/ros2_ws2/arm_bot/src/scripts/script_resources/target_positions.csv')
    
    # Maximum torque limits per joint [tau1_max, tau2_max, tau3_max] in Nm
    max_torques = [10.0, 50.0, 50.0]
    
    # Output directory for log files
    output_dir = '/data/ros2/ros2_ws2/arm_bot/src/scripts/dataset'
    
    # =====================================================================
    
    node = PositionSequenceController(
        positions_csv_path=positions_csv_path,
        max_torques=max_torques,
        output_dir=output_dir
    )
    
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        # Zero out torques on shutdown
        zero_msg = Float64MultiArray()
        zero_msg.data = [0.0]
        node.pub1.publish(zero_msg)
        node.pub2.publish(zero_msg)
        node.pub3.publish(zero_msg)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
