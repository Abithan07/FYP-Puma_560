#!/usr/bin/env python3
"""
Optimized Torque Publisher - Zero Delay, 100Hz Operation
Publishes pre-computed inverse dynamics torques from CSV with minimal latency.
Now with integrated triggered logging for precise data capture.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import csv
import math
import subprocess
import time
import os

class TorquePublisher(Node):
    def __init__(self):
        super().__init__('torque_publisher')
        
        # Publishers for torque commands (QoS=10 for reliability)
        self.pub1 = self.create_publisher(Float64MultiArray, '/joint_1_controller/commands', 10)
        self.pub2 = self.create_publisher(Float64MultiArray, '/joint_2_controller/commands', 10)
        self.pub3 = self.create_publisher(Float64MultiArray, '/joint_3_controller/commands', 10)
        
        # Subscriber to monitor joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Load CSV data (pre-load ALL data to avoid I/O delays during execution)
        self.csv_path = os.path.expanduser('/data/ros2/ros2_ws2/arm_bot/src/scripts/script_resources/path_001_joint_states_modified.csv')
        self.load_trajectory_data()
        
        # Pre-allocate message objects (avoid allocation overhead during control loop)
        self.msg1 = Float64MultiArray()
        self.msg2 = Float64MultiArray()
        self.msg3 = Float64MultiArray()
        
        # State variables
        self.current_idx = 0
        self.current_joint_pos = [0.0, 0.0, 0.0]
        self.current_joint_vel = [0.0, 0.0, 0.0]
        self.joint_states_received = False
        self.trajectory_active = False
        self.trajectory_timer = None
        
        # PID control timer for stabilization phase
        self.stabilization_timer = None
        self.stabilization_complete = False
        self.stabilization_iterations = 0
        
        # Integral error accumulation for PID
        self.integral_error = [0.0, 0.0, 0.0]
        
        # Logger subprocess and service clients
        self.logger_process = None
        self.logger_start_client = None
        self.logger_stop_client = None
        
        self.get_logger().info('='*70)
        self.get_logger().info('OPTIMIZED TORQUE PUBLISHER - ZERO DELAY MODE')
        self.get_logger().info('='*70)
        self.get_logger().info(f'Loaded {len(self.time_data)} trajectory points')
        self.get_logger().info(f'Trajectory duration: {self.time_data[-1]:.2f}s')
        self.get_logger().info(f'Control frequency: 100 Hz (dt={self.dt:.4f}s)')
        self.get_logger().info(f'Initial target: [{self.dp1[0]:.4f}, {self.dp2[0]:.4f}, {self.dp3[0]:.4f}] rad')
    
    def load_trajectory_data(self):
        """Load trajectory data from CSV file (executed once at startup)"""
        data = {}
        with open(self.csv_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if row:
                    key = row[0]
                    values = [float(val) for val in row[1:]]
                    data[key] = values
        
        # Extract trajectory arrays (stored in memory for fast access)
        self.time_data = data['t']
        self.dp1 = data['dp1']  # Desired joint positions
        self.dp2 = data['dp2']
        self.dp3 = data['dp3']
        self.tau1 = data['tau1']  # Computed torques
        self.tau2 = data['tau2']
        self.tau3 = data['tau3']
        
        # Calculate time step (should be 0.01s for 100Hz)
        self.dt = self.time_data[1] - self.time_data[0] if len(self.time_data) > 1 else 0.01
    
    def joint_state_callback(self, msg):
        """Minimal callback - just update state variables"""
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
    
    def launch_logger(self):
        """Launch the triggered logger as a subprocess"""
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            logger_script = os.path.join(script_dir, 'continuous_logger_triggered.py')
            
            self.get_logger().info('Launching triggered logger subprocess...')
            # Don't pipe stdout/stderr - let subprocess output directly to console
            # This prevents ROS2 initialization blocking issues in subprocess
            self.logger_process = subprocess.Popen(
                ['python3', logger_script],
                stdout=None,  # Inherit parent's stdout
                stderr=None   # Inherit parent's stderr
            )
            
            # Give logger time to initialize and create services (100ms)
            time.sleep(0.1)
            
            # Create service clients
            self.logger_start_client = self.create_client(Trigger, '/logger/start')
            self.logger_stop_client = self.create_client(Trigger, '/logger/stop')
            
            # Wait for services to be available (max 5 seconds)
            timeout = 5.0
            start_time = time.time()
            while not self.logger_start_client.wait_for_service(timeout_sec=0.1):
                if time.time() - start_time > timeout:
                    self.get_logger().error('Logger start service not available!')
                    return False
                rclpy.spin_once(self, timeout_sec=0.01)
            
            self.get_logger().info('✓ Logger subprocess ready')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to launch logger: {e}')
            return False
    
    def start_logger(self):
        """Call the logger start service"""
        if not self.logger_start_client:
            self.get_logger().error('Logger client not initialized')
            return False
        
        request = Trigger.Request()
        future = self.logger_start_client.call_async(request)
        
        # Wait for response (with timeout)
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 1.0:
                self.get_logger().error('Logger start service timeout')
                return False
            rclpy.spin_once(self, timeout_sec=0.01)
        
        response = future.result()
        if response.success:
            self.get_logger().info('✓ Logger recording started')
        else:
            self.get_logger().error(f'Logger start failed: {response.message}')
        
        return response.success
    
    def stop_logger(self):
        """Call the logger stop service"""
        if not self.logger_stop_client:
            self.get_logger().error('Logger client not initialized')
            return False
        
        request = Trigger.Request()
        future = self.logger_stop_client.call_async(request)
        
        # Wait for response (with timeout)
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 1.0:
                self.get_logger().error('Logger stop service timeout')
                return False
            rclpy.spin_once(self, timeout_sec=0.01)
        
        response = future.result()
        if response.success:
            self.get_logger().info(f'✓ Logger stopped: {response.message}')
        else:
            self.get_logger().error(f'Logger stop failed: {response.message}')
        
        return response.success
    
    def shutdown_logger(self):
        """Terminate the logger subprocess"""
        if self.logger_process:
            self.logger_process.terminate()
            try:
                self.logger_process.wait(timeout=2.0)
                self.get_logger().info('✓ Logger subprocess terminated')
            except subprocess.TimeoutExpired:
                self.logger_process.kill()
                self.get_logger().warning('Logger subprocess killed (timeout)')
    
    def stabilization_callback(self):
        """Timer callback for PID stabilization at 100Hz"""
        target_pos = [self.dp1[0], self.dp2[0], self.dp3[0]]
        
        # Calculate position errors
        errors = [target_pos[i] - self.current_joint_pos[i] for i in range(3)]
        max_error = max(abs(e) for e in errors)
        
        # Check convergence (strict threshold: 0.2°)
        if max_error < 0.0000035:  # 0.0002° in radians
            self.get_logger().info(f'✓ Initial position reached! Max error: {max_error*180/3.14159:.5f}°')
            if self.stabilization_timer:
                self.stabilization_timer.cancel()
            self.stabilization_complete = True
            return
        
        # Timeout check (60 seconds = 6000 iterations at 100Hz - increased for tighter convergence)
        self.stabilization_iterations += 1
        if self.stabilization_iterations >= 9000:
            self.get_logger().error(f'Timeout! Failed to reach 0.5° target. Current error: {max_error*180/3.14159:.3f}°')
            if self.stabilization_timer:
                self.stabilization_timer.cancel()
            self.stabilization_complete = True
            return
        
        # Full PID control with gravity compensation and integral windup protection
        # Tuned gains for tight convergence (0.5°)
        kp = [50.0, 200.0, 150.0]    # Increased Kp for joint 3
        ki = [5.0, 25.0, 20.0]       # Increased Ki for joint 2 (15→25)
        kd = [12.0, 35.0, 10.0]      # Increased Kd for joint 3
        
        # Accumulate integral error (with anti-windup)
        dt = 0.01  # 100Hz = 0.01s
        for i in range(3):
            self.integral_error[i] += errors[i] * dt
            # Anti-windup: clamp integral to prevent excessive accumulation
            max_integral = [0.5, 1.0, 1.0]  # Increased limit for joints 2 and 3
            self.integral_error[i] = max(-max_integral[i], min(max_integral[i], self.integral_error[i]))
        
        # Gravity compensation (feedforward) - further tuned values
        q2 = self.current_joint_pos[1]
        q3 = self.current_joint_pos[2]
        gravity_comp = [
            0.0,
            -44.0 * math.cos(q2),        # Increased for joint 2 (42→44)
            -12.0 * math.cos(q2 + q3)    # Significantly increased for joint 3
        ]
        
        # Full PID control law: τ = Kp*e + Ki*∫e + Kd*ė + g(q)
        torques = [
            kp[i] * errors[i] + ki[i] * self.integral_error[i] - kd[i] * self.current_joint_vel[i] + gravity_comp[i]
            for i in range(3)
        ]
        
        # Torque saturation
        max_torques = [100.0, 100.0, 50.0]
        torques = [max(-max_torques[i], min(max_torques[i], torques[i])) for i in range(3)]
        
        # Publish torques (use pre-allocated messages)
        self.msg1.data = [torques[0]]
        self.msg2.data = [torques[1]]
        self.msg3.data = [torques[2]]
        
        self.pub1.publish(self.msg1)
        self.pub2.publish(self.msg2)
        self.pub3.publish(self.msg3)
        
        # Log progress every 50 iterations (0.5s)
        if self.stabilization_iterations % 50 == 0:
            self.get_logger().info(
                f'  t={self.stabilization_iterations/100:.1f}s | '
                f'Error: [{errors[0]*180/3.14159:.2f}°, {errors[1]*180/3.14159:.2f}°, {errors[2]*180/3.14159:.2f}°] | '
                f'Max: {max_error*180/3.14159:.2f}° | '
                f'Integral: [{self.integral_error[0]:.3f}, {self.integral_error[1]:.3f}, {self.integral_error[2]:.3f}]'
            )
    
    def trajectory_callback(self):
        """Timer callback for trajectory execution at EXACTLY 100Hz"""
        if self.current_idx >= len(self.time_data):
            self.get_logger().info('='*70)
            self.get_logger().info('TRAJECTORY EXECUTION COMPLETED')
            self.get_logger().info('='*70)
            if self.trajectory_timer:
                self.trajectory_timer.cancel()
            self.trajectory_active = False
            return
        
        # Publish torques directly (no logging, no conditionals - minimal overhead)
        self.msg1.data = [self.tau1[self.current_idx]]
        self.msg2.data = [self.tau2[self.current_idx]]
        self.msg3.data = [self.tau3[self.current_idx]]
        
        self.pub1.publish(self.msg1)
        self.pub2.publish(self.msg2)
        self.pub3.publish(self.msg3)
        
        # Increment index (simple increment - no time synchronization logic)
        self.current_idx += 1
        
        # Optional: Log every 50 points (0.5s) - OUTSIDE the critical path
        if self.current_idx % 50 == 0 and self.current_idx < len(self.time_data):
            # Calculate tracking error
            pos_err = [
                abs(self.dp1[self.current_idx] - self.current_joint_pos[0]),
                abs(self.dp2[self.current_idx] - self.current_joint_pos[1]),
                abs(self.dp3[self.current_idx] - self.current_joint_pos[2])
            ]
            max_err_deg = max(pos_err) * 180 / 3.14159
            
            self.get_logger().info(
                f't={self.time_data[self.current_idx]:.1f}s | '
                f'idx={self.current_idx}/{len(self.time_data)} | '
                f'τ=[{self.tau1[self.current_idx]:.1f}, {self.tau2[self.current_idx]:.1f}, {self.tau3[self.current_idx]:.1f}] Nm | '
                f'err={max_err_deg:.1f}°'
            )
    
    def run(self):
        """Main execution sequence"""
        self.get_logger().info('Waiting for joint states...')
        
        # Wait for joint states (blocking)
        while not self.joint_states_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.joint_states_received:
            self.get_logger().error('Failed to receive joint states!')
            return
        
        self.get_logger().info(f'Current position: [{self.current_joint_pos[0]:.4f}, {self.current_joint_pos[1]:.4f}, {self.current_joint_pos[2]:.4f}] rad')
        
        # # Phase 1: Move to initial position using PID control
        # self.get_logger().info('='*70)
        # self.get_logger().info('PHASE 1: STABILIZATION (Moving to initial position)')
        # self.get_logger().info('='*70)
        # self.get_logger().info('Target: 0.5° convergence with full PID + gravity compensation')
        # self.stabilization_iterations = 0
        # self.stabilization_complete = False
        # self.integral_error = [0.0, 0.0, 0.0]  # Reset integral error
        # self.stabilization_timer = self.create_timer(0.01, self.stabilization_callback)  # 100 Hz
        
        # # Wait for stabilization to complete
        # while not self.stabilization_complete and rclpy.ok():
        #     rclpy.spin_once(self, timeout_sec=0.001)
        
        # if not rclpy.ok():
        #     return
        
        # self.get_logger().info('Holding position for 1 second...')
        # # Hold with timer (100Hz)
        # hold_iterations = [0]
        # def hold_callback():
        #     hold_iterations[0] += 1
        #     if hold_iterations[0] >= 100:  # 1 second
        #         hold_timer.cancel()
        #         return
        #     # Keep publishing last stabilization torque
        #     self.stabilization_callback()
        
        # hold_timer = self.create_timer(0.01, hold_callback)
        # while hold_iterations[0] < 100 and rclpy.ok():
        #     rclpy.spin_once(self, timeout_sec=0.001)
        
        # Launch logger subprocess
        self.get_logger().info('='*70)
        self.get_logger().info('Launching data logger...')
        if not self.launch_logger():
            self.get_logger().error('Failed to launch logger, continuing without logging')
        
        # Phase 2: Execute trajectory with computed torques
        self.get_logger().info('='*70)
        self.get_logger().info('PHASE 2: TRAJECTORY EXECUTION (Open-loop torque control)')
        self.get_logger().info('='*70)
        self.get_logger().info(f'Publishing {len(self.time_data)} torque commands at 100Hz...')
        
        # Start logger 100ms before trajectory execution
        time.sleep(0.1)
        if self.logger_start_client:
            self.start_logger()
        
        # Small delay to ensure logger is recording before first torque command
        time.sleep(0.01)
        
        self.current_idx = 0
        self.trajectory_active = True
        
        # Create 100Hz timer for trajectory execution
        # Period = 1/100 = 0.01 seconds = 10ms
        self.trajectory_timer = self.create_timer(0.01, self.trajectory_callback)
        
        # Spin until trajectory completes
        while self.trajectory_active and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)
        
        # Stop logger 100ms after trajectory completion
        time.sleep(0.1)
        if self.logger_stop_client:
            self.stop_logger()
        
        # Give logger time to write final data
        time.sleep(0.1)
        
        self.get_logger().info('Torque publisher shutting down.')

def main(args=None):
    rclpy.init(args=args)
    node = TorquePublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        # Shutdown logger if running
        node.shutdown_logger()
        
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

