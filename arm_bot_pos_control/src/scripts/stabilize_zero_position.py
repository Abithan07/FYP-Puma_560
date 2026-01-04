#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time
from trajectory_logger import TrajectoryLogger

class ZeroPositionStabilizer(Node):
    def __init__(self, target_position=None, position_threshold=None):
        super().__init__('zero_position_stabilizer')
        
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
        
        # Target position: use provided or default to [0, 0, 0]
        if target_position is not None:
            self.target_pos = target_position
        else:
            # pi/2 for joint 2 to account for initial offset
            self.target_pos = [0, -1.57, 0]
        
        # Position threshold for considering stabilization complete (radians)
        # Default to 5 degrees = 0.0873 radians
        self.position_threshold = position_threshold if position_threshold is not None else 0.0873
        
        # Current joint states
        self.current_joint_pos = [0.0, 0.0, 0.0]
        self.current_joint_vel = [0.0, 0.0, 0.0]
        self.joint_states_received = False
        
        # PID controller parameters
        self.kp = [30.0, 150.0, 80.0]  # Proportional gains
        self.ki = [0.5, 1.5, 0.8]      # Integral gains
        self.kd = [8.0, 25.0, 4.0]      # Derivative gains
        
        # PID state variables
        self.integral_error = [0.0, 0.0, 0.0]
        self.previous_error = [0.0, 0.0, 0.0]
        
        # Control parameters
        self.max_torques = [80.0, 150.0, 40.0]  # Maximum torque limits
        self.max_integral = [10.0, 25.0, 5.0]  # Anti-windup limits
        self.control_rate = 100  # Hz
        self.dt = 1.0 / self.control_rate
        
        # Stabilization status
        self.is_stabilized = False
        self.stabilization_start_time = None
        self.min_stabilization_time = 0.5  # Must stay stable for 0.5 seconds
        
        # Initialize trajectory logger
        self.logger_csv = TrajectoryLogger(base_name="stabilize_position", output_dir="./logs")
        self.csv_file = self.logger_csv.start_logging()
        self.last_torques = [0.0, 0.0, 0.0]  # Store last commanded torques for logging
        
        # Create timer for control loop
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('Position Stabilizer initialized')
        self.get_logger().info(f'Logging to: {self.csv_file} (Test #{self.logger_csv.get_test_number()})')
        self.get_logger().info(f'Target position: [{self.target_pos[0]:.4f}, {self.target_pos[1]:.4f}, {self.target_pos[2]:.4f}] rad')
        self.get_logger().info(f'Position threshold: {self.position_threshold:.4f} rad ({self.position_threshold * 180 / 3.14159:.2f} deg)')
        self.get_logger().info(f'PID gains - Kp: {self.kp}, Ki: {self.ki}, Kd: {self.kd}')
        self.get_logger().info(f'Control rate: {self.control_rate} Hz')
        
        self.stab_log = []  # For compact torque/position log
        self.stab_log_file = None

    def joint_state_callback(self, msg):
        """Monitor current joint positions and velocities"""
        try:
            # Find joint indices (joints should be joint_1, joint_2, joint_3)
            idx1 = msg.name.index('joint_1')
            idx2 = msg.name.index('joint_2')
            idx3 = msg.name.index('joint_3')
            
            self.current_joint_pos = [
                msg.position[idx1],
                msg.position[idx2],
                msg.position[idx3]
            ]
            
            if len(msg.velocity) > 0:
                self.current_joint_vel = [
                    msg.velocity[idx1],
                    msg.velocity[idx2],
                    msg.velocity[idx3]
                ]
            
            if not self.joint_states_received:
                self.joint_states_received = True
                self.get_logger().info(f'Joint states received. Starting from: {[f"{p:.3f}" for p in self.current_joint_pos]}')
        except (ValueError, IndexError) as e:
            pass
    
    def control_loop(self):
        """PID control loop running at fixed rate"""
        if not self.joint_states_received:
            return
        
        # Calculate position errors
        errors = [self.target_pos[i] - self.current_joint_pos[i] for i in range(3)]
        
        # Update integral with anti-windup
        for i in range(3):
            self.integral_error[i] += errors[i] * self.dt
            # Anti-windup: clamp integral term
            self.integral_error[i] = max(-self.max_integral[i], 
                                        min(self.max_integral[i], self.integral_error[i]))
        
        # Calculate derivative (error rate of change)
        derivative_error = [(errors[i] - self.previous_error[i]) / self.dt for i in range(3)]
        
        # PID control law
        torques = [
            self.kp[i] * errors[i] + 
            self.ki[i] * self.integral_error[i] + 
            self.kd[i] * derivative_error[i]
            for i in range(3)
        ]
        
        # Alternative: Use velocity directly for derivative term (more robust to noise)
        # torques = [
        #     self.kp[i] * errors[i] + 
        #     self.ki[i] * self.integral_error[i] - 
        #     self.kd[i] * self.current_joint_vel[i]
        #     for i in range(3)
        # ]
        
        # Saturate torques
        torques = [max(-self.max_torques[i], min(self.max_torques[i], torques[i])) 
                   for i in range(3)]
        
        # Publish torques
        msg1 = Float64MultiArray()
        msg2 = Float64MultiArray()
        msg3 = Float64MultiArray()
        
        msg1.data = [torques[0]]
        msg2.data = [torques[1]]
        msg3.data = [torques[2]]
        
        self.pub1.publish(msg1)
        self.pub2.publish(msg2)
        self.pub3.publish(msg3)
        
        # Store torques and log data to CSV
        self.last_torques = torques.copy()
        phase = 'STABILIZED' if self.is_stabilized else 'STABILIZING'
        self.logger_csv.log_data(
            positions=self.current_joint_pos,
            velocities=self.current_joint_vel,
            torques=torques,
            phase=phase
        )
        
        # Log torques and positions in requested order
        self.stab_log.append([
            torques[0], self.current_joint_pos[0],
            torques[1], self.current_joint_pos[1],
            torques[2], self.current_joint_pos[2]
        ])
        
        # Update previous error for next iteration
        self.previous_error = errors.copy()
        
        # Check if position is stabilized
        max_error = max(abs(e) for e in errors)
        if max_error < self.position_threshold:
            # Save compact log when stabilized for the first time
            if self.stabilization_start_time is None:
                self.stabilization_start_time = time.time()
            elif time.time() - self.stabilization_start_time >= self.min_stabilization_time:
                if not self.is_stabilized:
                    self.is_stabilized = True
                    self.get_logger().info(f'✓ Position STABILIZED! Max error: {max_error:.4f} rad ({max_error * 180 / 3.14159:.2f} deg)')
                    self.save_stab_log()
        else:
            self.stabilization_start_time = None
            self.is_stabilized = False
        
        # Log status every 2 seconds
        if not hasattr(self, 'last_log_time'):
            self.last_log_time = time.time()
        
        if time.time() - self.last_log_time >= 2.0:
            status = '✓ STABLE' if self.is_stabilized else 'STABILIZING...'
            self.get_logger().info(
                f'{status} | Pos: [{self.current_joint_pos[0]:6.3f}, {self.current_joint_pos[1]:6.3f}, {self.current_joint_pos[2]:6.3f}] | '
                f'Err: [{errors[0]:6.4f}, {errors[1]:6.4f}, {errors[2]:6.4f}] | '
                f'Max: {max_error:.4f} rad'
            )
            self.last_log_time = time.time()
    
    def is_stable(self):
        """Check if the robot has reached and is holding the target position"""
        return self.is_stabilized

    def save_stab_log(self):
        """Save the compact torque/position log to a CSV file."""
        import csv, datetime, os
        if not self.stab_log:
            return
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        fname = f'stabilization_torque_position_{timestamp}.csv'
        outdir = './logs'
        os.makedirs(outdir, exist_ok=True)
        fpath = os.path.join(outdir, fname)
        with open(fpath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['tau1','pos1','tau2','pos2','tau3','pos3'])
            writer.writerows(self.stab_log)
        self.get_logger().info(f'Compact stabilization log saved: {fpath}')

def main(args=None):
    rclpy.init(args=args)
    node = ZeroPositionStabilizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down stabilizer...')
    finally:
        # Stop logging
        final_file = node.logger_csv.stop_logging()
        node.get_logger().info(f'Logged {node.logger_csv.get_log_count()} data points to: {final_file}')
        
        # Send zero torques before shutting down
        zero_msg = Float64MultiArray()
        zero_msg.data = [0.0]
        node.pub1.publish(zero_msg)
        node.pub2.publish(zero_msg)
        node.pub3.publish(zero_msg)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
