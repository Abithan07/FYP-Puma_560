#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time

class ZeroPositionStabilizer(Node):
    def __init__(self):
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
        
        # Target position: [pi/4, pi/4, pi/4]
        self.target_pos = [3.14159/4, 3.14159/4, 3.14159/4]
        
        # Current joint states
        self.current_joint_pos = [0.0, 0.0, 0.0]
        self.current_joint_vel = [0.0, 0.0, 0.0]
        self.joint_states_received = False
        
        # PID controller parameters
        self.kp = [30.0, 100.0, 15.0]  # Proportional gains
        self.ki = [0.5, 1.5, 0.2]      # Integral gains
        self.kd = [8.0, 25.0, 4.0]      # Derivative gains
        
        # PID state variables
        self.integral_error = [0.0, 0.0, 0.0]
        self.previous_error = [0.0, 0.0, 0.0]
        
        # Control parameters
        self.max_torques = [80.0, 150.0, 40.0]  # Maximum torque limits
        self.max_integral = [10.0, 25.0, 5.0]  # Anti-windup limits
        self.control_rate = 100  # Hz
        self.dt = 1.0 / self.control_rate
        
        # Create timer for control loop
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('Zero Position Stabilizer initialized')
        self.get_logger().info(f'Target position: {self.target_pos}')
        self.get_logger().info(f'PID gains - Kp: {self.kp}, Ki: {self.ki}, Kd: {self.kd}')
        self.get_logger().info(f'Control rate: {self.control_rate} Hz')
    
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
        
        # Update previous error for next iteration
        self.previous_error = errors.copy()
        
        # Log status every 2 seconds
        if not hasattr(self, 'last_log_time'):
            self.last_log_time = time.time()
        
        if time.time() - self.last_log_time >= 2.0:
            max_error = max(abs(e) for e in errors)
            self.get_logger().info(
                f'Pos: [{self.current_joint_pos[0]:6.3f}, {self.current_joint_pos[1]:6.3f}, {self.current_joint_pos[2]:6.3f}] | '
                f'Err: [{errors[0]:6.4f}, {errors[1]:6.4f}, {errors[2]:6.4f}] | '
                f'Torq: [{torques[0]:5.1f}, {torques[1]:5.1f}, {torques[2]:5.1f}] | '
                f'Max Err: {max_error:.4f} rad'
            )
            self.last_log_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = ZeroPositionStabilizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down stabilizer...')
    finally:
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
