import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time
import csv
import os
import math

class TorquePublisher(Node):
    def __init__(self):
        super().__init__('torque_publisher')
        
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
        
        # Load CSV data
        self.csv_path = '/home/abithan_ubuntu/ros2_ws2/arm_bot/path_angle_005_joint_states.csv'
        self.load_trajectory_data()
        
        # Joint limits (from URDF)
        # Joint 1: [-2.7625, 2.7625]
        # Joint 2: [-3.9270, 0.7854]
        # Joint 3: [-0.7854, 3.9270]
        self.joint_limits = [
            [-2.7625, 2.7625],  # Joint 1 (shoulder)
            [-3.9270, 0.7854],   # Joint 2 (link_2)
            [-0.7854, 3.9270]    # Joint 3 (link_3)
        ]
        
        # State variables
        self.current_idx = 0
        self.initialized = False
        self.current_joint_pos = [0.0, 0.0, 0.0]
        self.current_joint_vel = [0.0, 0.0, 0.0]
        self.joint_states_received = False
        
        self.get_logger().info('Torque publisher initialized')
        self.get_logger().info(f'Loaded {len(self.time_data)} trajectory points')
        self.get_logger().info(f'Initial positions: dp1={self.dp1[0]:.3f}, dp2={self.dp2[0]:.3f}, dp3={self.dp3[0]:.3f}')
        
        # Validate trajectory limits
        # self.validate_trajectory_limits()  # Commented out - joint limits disabled
    
    def load_trajectory_data(self):
        """Load trajectory data from CSV file"""
        data = {}
        with open(self.csv_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if row:
                    key = row[0]
                    values = [float(val) for val in row[1:]]
                    data[key] = values
        
        # Extract required data
        self.time_data = data['t']
        self.dp1 = data['dp1']  # Initial joint 1 position
        self.dp2 = data['dp2']  # Initial joint 2 position
        self.dp3 = data['dp3']  # Initial joint 3 position
        self.tau1 = data['tau1']  # Joint 1 torques
        self.tau2 = data['tau2']  # Joint 2 torques
        self.tau3 = data['tau3']  # Joint 3 torques
        
        self.dt = self.time_data[1] - self.time_data[0] if len(self.time_data) > 1 else 0.01
    
    def validate_trajectory_limits(self):
        """Check if all trajectory points are within joint limits"""
        trajectories = [self.dp1, self.dp2, self.dp3]
        joint_names = ['joint_1', 'joint_2', 'joint_3']
        
        all_valid = True
        for joint_idx, (traj, limits, name) in enumerate(zip(trajectories, self.joint_limits, joint_names)):
            min_pos = min(traj)
            max_pos = max(traj)
            
            if min_pos < limits[0] or max_pos > limits[1]:
                all_valid = False
                self.get_logger().warn(
                    f'{name}: Trajectory range [{min_pos:.3f}, {max_pos:.3f}] '
                    f'exceeds limits [{limits[0]:.3f}, {limits[1]:.3f}]'
                )
            else:
                self.get_logger().info(
                    f'{name}: Trajectory range [{min_pos:.3f}, {max_pos:.3f}] '
                    f'within limits [{limits[0]:.3f}, {limits[1]:.3f}] ✓'
                )
        
        if all_valid:
            self.get_logger().info('All trajectory points are within joint limits!')
        else:
            self.get_logger().warn('WARNING: Some trajectory points exceed joint limits!')
    
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
            
            self.joint_states_received = True
        except (ValueError, IndexError) as e:
            pass
    
    def move_to_initial_position(self):
        """Use PD control to move joints to initial position"""
        target_pos = [self.dp1[0], self.dp2[0], self.dp3[0]]
        
        # Validate target position is within limits
        # for i, (pos, limits) in enumerate(zip(target_pos, self.joint_limits)):
        #     if pos < limits[0] or pos > limits[1]:
        #         self.get_logger().error(
        #             f'Initial position joint_{i+1}={pos:.3f} is outside limits [{limits[0]:.3f}, {limits[1]:.3f}]!'
        #         )
        #         raise ValueError(f'Initial position for joint {i+1} violates limits')
        
        self.get_logger().info('Moving to initial position...')
        self.get_logger().info(f'Current: [{self.current_joint_pos[0]:.3f}, {self.current_joint_pos[1]:.3f}, {self.current_joint_pos[2]:.3f}]')
        self.get_logger().info(f'Target:  [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]')
        self.get_logger().info(f'Errors:  [{target_pos[0]-self.current_joint_pos[0]:.3f}, {target_pos[1]-self.current_joint_pos[1]:.3f}, {target_pos[2]-self.current_joint_pos[2]:.3f}]')
        
        # PD gains for positioning (tune these if needed) - REDUCED to prevent oscillation
        kp = [20.0, 20.0, 10.0]  # Proportional gains
        kd = [5.0, 5.0, 2.0]      # Derivative gains
        
        position_threshold = 0.01  # 0.01 rad = ~0.57 degrees
        max_iterations = 1000  # 10 seconds at 100Hz (increased timeout)
        iteration = 0
        
        rate = self.create_rate(100)  # 100 Hz control loop
        
        while iteration < max_iterations and rclpy.ok():
            # Calculate position errors
            errors = [target_pos[i] - self.current_joint_pos[i] for i in range(3)]
            
            # Check if we've reached the target
            max_error = max(abs(e) for e in errors)
            if max_error < position_threshold:
                self.get_logger().info(f'Initial position reached! Max error: {max_error:.4f} rad')
                break
            
            # Calculate PD control torques
            torques = [
                kp[i] * errors[i] - kd[i] * self.current_joint_vel[i]
                for i in range(3)
            ]
            
            # Saturate torques to prevent excessive forces
            max_torques = [50.0, 50.0, 30.0]  # Maximum torque limits
            torques = [max(-max_torques[i], min(max_torques[i], torques[i])) for i in range(3)]
            
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
            
            # Log progress every 50 iterations (0.5 seconds)
            if iteration % 50 == 0:
                self.get_logger().info(
                    f'Error: [{errors[0]:.4f}, {errors[1]:.4f}, {errors[2]:.4f}] rad | '
                    f'Max: {max_error:.4f} rad'
                )
            
            iteration += 1
            rclpy.spin_once(self, timeout_sec=0.01)
            rate.sleep()
        
        if iteration >= max_iterations:
            self.get_logger().warn('Initial position not fully reached, continuing anyway...')
        
        # Let the robot settle
        self.get_logger().info('Settling for 1 second...')
        time.sleep(1.0)
    
    def run(self):
        """Main control loop"""
        self.get_logger().info('Waiting for initial joint states...')
        
        # Wait for joint states to be available
        timeout = 5.0
        start_wait = time.time()
        while not self.joint_states_received and (time.time() - start_wait) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.joint_states_received:
            self.get_logger().error('Failed to receive joint states!')
            return
        
        self.get_logger().info(f'Current joint positions: [{self.current_joint_pos[0]:.3f}, {self.current_joint_pos[1]:.3f}, {self.current_joint_pos[2]:.3f}]')
        self.get_logger().info(f'Target initial positions: [{self.dp1[0]:.3f}, {self.dp2[0]:.3f}, {self.dp3[0]:.3f}]')
        
        # Move to initial position using PD control
        self.move_to_initial_position()
        
        self.get_logger().info('='*60)
        self.get_logger().info('Starting trajectory execution with computed torques!')
        self.get_logger().info('='*60)
        time.sleep(1.0)
        
        start_time = time.time()
        
        while rclpy.ok() and self.current_idx < len(self.time_data):
            # Calculate elapsed time
            elapsed = time.time() - start_time
            
            # Find the closest time index
            while (self.current_idx < len(self.time_data) - 1 and 
                   self.time_data[self.current_idx] < elapsed):
                self.current_idx += 1
            
            # Publish torques
            msg1 = Float64MultiArray()
            msg2 = Float64MultiArray()
            msg3 = Float64MultiArray()
            
            msg1.data = [self.tau1[self.current_idx]]
            msg2.data = [self.tau2[self.current_idx]]
            msg3.data = [self.tau3[self.current_idx]]
            
            self.pub1.publish(msg1)
            self.pub2.publish(msg2)
            self.pub3.publish(msg3)
            
            # Spin once to update joint states
            rclpy.spin_once(self, timeout_sec=0)
            
            # Log progress every 0.5 seconds
            if self.current_idx % int(0.5 / self.dt) == 0:
                # Calculate position errors
                pos_error = [
                    self.dp1[self.current_idx] - self.current_joint_pos[0],
                    self.dp2[self.current_idx] - self.current_joint_pos[1],
                    self.dp3[self.current_idx] - self.current_joint_pos[2]
                ]
                max_error = max(abs(e) for e in pos_error)
                
                self.get_logger().info(
                    f'Time: {self.time_data[self.current_idx]:.2f}s | '
                    f'Torques: [{self.tau1[self.current_idx]:.1f}, {self.tau2[self.current_idx]:.1f}, {self.tau3[self.current_idx]:.1f}] | '
                    f'Pos Error: {max_error:.4f} rad'
                )
            
            # Sleep to maintain real-time execution
            time.sleep(self.dt)
        
        self.get_logger().info('Trajectory execution completed!')
        # Hold final torques for a bit
        time.sleep(2.0)

def main(args=None):
    rclpy.init(args=args)
    node = TorquePublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()