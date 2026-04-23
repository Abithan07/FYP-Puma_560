#!/usr/bin/env python3
"""
Sinusoidal Torque Publisher - 100Hz Operation with Real-time Logging
Injects sinusoidal torque commands: τ = 60 * sin(2π * t / T)
Logs:
  - Injected torque commands (sinusoidal)
  - Sensor readings: position and velocity from Gazebo
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import math
import subprocess
import time
import os
import csv
from datetime import datetime


class SinusoidalTorquePublisher(Node):
    def __init__(self, max_torque=[20.0, 40.0, 5.0], period=10.0, duration=30.0):
        """
        Initialize sinusoidal torque publisher.
        
        Args:
            max_torque: Maximum torque amplitude (Nm) - default 60
            period: Period for one complete cycle (seconds) - default 10s
            duration: Total duration of test (seconds) - default 30s (3 cycles)
        """
        super().__init__('sinusoidal_torque_publisher')
        
        # Publishers for torque commands (QoS=10 for reliability)
        self.pub1 = self.create_publisher(Float64MultiArray, '/joint_1_controller/commands', 10)
        self.pub2 = self.create_publisher(Float64MultiArray, '/joint_2_controller/commands', 10)
        self.pub3 = self.create_publisher(Float64MultiArray, '/joint_3_controller/commands', 10)
        
        # Subscriber to monitor joint states (position, velocity from Gazebo)
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Configuration parameters
        self.max_torque = max_torque
        self.period = period
        self.duration = duration
        self.frequency = 1.0 / period  # Hz
        
        # Pre-allocate message objects
        self.msg1 = Float64MultiArray()
        self.msg2 = Float64MultiArray()
        self.msg3 = Float64MultiArray()
        
        # State variables - Gazebo sensor readings
        self.start_time = None
        self.gazebo_joint_pos = [0.0, 0.0, 0.0]      # Position from Gazebo sensors
        self.gazebo_joint_vel = [0.0, 0.0, 0.0]      # Velocity from Gazebo sensors
        self.joint_states_received = False
        self.test_active = False
        self.test_timer = None
        
        # Logger subprocess and service clients
        self.logger_process = None
        self.logger_start_client = None
        self.logger_stop_client = None
        
        # CSV logging for sensor data
        self.csv_file = None
        self.csv_writer = None
        
        # Create logs directory if it doesn't exist
        self.logs_dir = os.path.expanduser('/data/ros2/ros2_ws2/arm_bot/src/scripts/dataset')
        os.makedirs(self.logs_dir, exist_ok=True)
        
        # Generate log filename with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = os.path.join(self.logs_dir, f'sinusoidal_dataset_{timestamp}.csv')
        
        self.get_logger().info('='*70)
        self.get_logger().info('SINUSOIDAL TORQUE PUBLISHER - 100Hz OPERATION WITH LOGGING')
        self.get_logger().info('='*70)
        self.get_logger().info(f'Max Torque: {self.max_torque} Nm')
        self.get_logger().info(f'Period: {self.period} s')
        self.get_logger().info(f'Frequency: {self.frequency:.2f} Hz')
        self.get_logger().info(f'Total Duration: {self.duration} s')
        self.get_logger().info(f'Control Frequency: 100 Hz (dt=0.01s)')
        self.get_logger().info(f'Number of Cycles: {self.duration / self.period:.1f}')
        self.get_logger().info(f'Log file: {self.csv_filename}')
        self.get_logger().info('')
        self.get_logger().info('Logged Data:')
        self.get_logger().info('  - injected_tau1/2/3: Sinusoidal command torques')
        self.get_logger().info('  - gazebo_pos1/2/3: Position readings from Gazebo sensors')
        self.get_logger().info('  - gazebo_vel1/2/3: Velocity readings from Gazebo sensors')
    
    def joint_state_callback(self, msg):
        """Update joint state variables from Gazebo (position, velocity)"""
        try:
            idx1 = msg.name.index('joint_1')
            idx2 = msg.name.index('joint_2')
            idx3 = msg.name.index('joint_3')
            
            # Read position from Gazebo sensors
            self.gazebo_joint_pos = [
                msg.position[idx1],
                msg.position[idx2],
                msg.position[idx3]
            ]
            
            # Read velocity from Gazebo sensors
            if len(msg.velocity) >= 3:
                self.gazebo_joint_vel = [
                    msg.velocity[idx1],
                    msg.velocity[idx2],
                    msg.velocity[idx3]
                ]
            
            self.joint_states_received = True
        except (ValueError, IndexError):
            pass
    
    def init_csv_logging(self):
        """Initialize CSV file with headers"""
        try:
            self.csv_file = open(self.csv_filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # Write header
            header = [
                'time_elapsed',
                'injected_tau1', 'injected_tau2', 'injected_tau3',
                'gazebo_pos1', 'gazebo_pos2', 'gazebo_pos3',
                'gazebo_vel1', 'gazebo_vel2', 'gazebo_vel3'
            ]
            self.csv_writer.writerow(header)
            self.csv_file.flush()
            
            self.get_logger().info('✓ CSV logging initialized')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CSV logging: {e}')
            return False
    
    def log_data_point(self, elapsed_time, injected_torques):
        """Log a single data point to CSV
        
        Args:
            elapsed_time: Elapsed time in seconds
            injected_torques: List of [tau1, tau2, tau3] injected command values
        """
        try:
            row = [
                f'{elapsed_time:.4f}',
                f'{injected_torques[0]:.4f}', f'{injected_torques[1]:.4f}', f'{injected_torques[2]:.4f}',
                f'{self.gazebo_joint_pos[0]:.4f}', f'{self.gazebo_joint_pos[1]:.4f}', f'{self.gazebo_joint_pos[2]:.4f}',
                f'{self.gazebo_joint_vel[0]:.4f}', f'{self.gazebo_joint_vel[1]:.4f}', f'{self.gazebo_joint_vel[2]:.4f}'
            ]
            self.csv_writer.writerow(row)
        except Exception as e:
            self.get_logger().error(f'Failed to log data point: {e}')
    
    def flush_csv(self):
        """Flush CSV data to disk"""
        if self.csv_file:
            try:
                self.csv_file.flush()
            except Exception as e:
                self.get_logger().error(f'Failed to flush CSV: {e}')
    
    def close_csv(self):
        """Close CSV file"""
        if self.csv_file:
            try:
                self.csv_file.close()
                self.get_logger().info(f'✓ CSV file saved: {self.csv_filename}')
            except Exception as e:
                self.get_logger().error(f'Failed to close CSV: {e}')
    
    def launch_logger(self):
        """Launch the triggered logger as a subprocess"""
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            logger_script = os.path.join(script_dir, 'continuous_logger_triggered.py')
            
            if not os.path.exists(logger_script):
                self.get_logger().warning(f'Logger script not found: {logger_script}')
                return False
            
            self.get_logger().info('Launching triggered logger subprocess...')
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
    
    def calculate_sinusoidal_torque(self, elapsed_time, joint_index=0):
        """
        Calculate sinusoidal torque: τ = τ_max * sin(2π * t / T)
        
        Args:
            elapsed_time: Time in seconds since start
        
        Returns:
            Torque value in Nm
        """
        # Full sinusoidal cycle: 0 → 1 → 0 → -1 → 0
        phase = 2.0 * math.pi * elapsed_time / self.period
        torque = self.max_torque[joint_index] * math.sin(phase)
        return torque
    
    def test_callback(self):
        """Timer callback for sinusoidal torque injection at 100Hz"""
        if not self.start_time:
            self.start_time = time.time()
        
        elapsed_time = time.time() - self.start_time
        
        # Check if test duration exceeded
        if elapsed_time > self.duration:
            self.get_logger().info('='*70)
            self.get_logger().info('TEST EXECUTION COMPLETED')
            self.get_logger().info('='*70)
            if self.test_timer:
                self.test_timer.cancel()
            self.test_active = False
            return
        
        # Calculate sinusoidal torque
        torque1 = self.calculate_sinusoidal_torque(elapsed_time, joint_index=0)
        torque2 = self.calculate_sinusoidal_torque(elapsed_time, joint_index=1)
        torque3 = self.calculate_sinusoidal_torque(elapsed_time, joint_index=2)
        injected_torques = [torque1, torque2, torque3]
        
        # Publish same torque to all three joints
        self.msg1.data = [torque1]
        self.msg2.data = [torque2]
        self.msg3.data = [torque3]
        
        self.pub1.publish(self.msg1)
        self.pub2.publish(self.msg2)
        self.pub3.publish(self.msg3)
        
        # Log data: injected torques + Gazebo sensor readings (position and velocity)
        self.log_data_point(elapsed_time, injected_torques)
        
        # Flush CSV every 50 iterations (0.5s) to ensure data is written
        iteration = int(elapsed_time / 0.01)
        if iteration % 50 == 0:
            self.flush_csv()
        
        # Console log every 100 iterations (1s)
        if iteration % 100 == 0:
            cycle_progress = (elapsed_time % self.period) / self.period * 100
            cycle_num = int(elapsed_time / self.period) + 1
            
            self.get_logger().info(
                f't={elapsed_time:.2f}s | '
                f'Cycle {cycle_num} ({cycle_progress:.1f}%) | '
                f'Inj τ=[{torque1:.2f}, {torque2:.2f}, {torque3:.2f}] Nm | '
                f'Gazebo Pos=[{self.gazebo_joint_pos[0]:.3f}, {self.gazebo_joint_pos[1]:.3f}, {self.gazebo_joint_pos[2]:.3f}] rad | '
                f'Vel=[{self.gazebo_joint_vel[0]:.3f}, {self.gazebo_joint_vel[1]:.3f}, {self.gazebo_joint_vel[2]:.3f}] rad/s'
            )
    
    def run(self):
        """Main execution sequence"""
        self.get_logger().info('Waiting for joint states from Gazebo...')
        
        # Wait for joint states (blocking)
        while not self.joint_states_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.joint_states_received:
            self.get_logger().error('Failed to receive joint states from Gazebo!')
            return
        
        self.get_logger().info(f'✓ Joint states received from Gazebo')
        self.get_logger().info(f'Initial position: [{self.gazebo_joint_pos[0]:.4f}, {self.gazebo_joint_pos[1]:.4f}, {self.gazebo_joint_pos[2]:.4f}] rad')
        
        # Initialize CSV logging
        if not self.init_csv_logging():
            self.get_logger().warning('Failed to initialize CSV logging')
        
        # Launch logger subprocess
        self.get_logger().info('='*70)
        self.get_logger().info('Launching data logger...')
        if not self.launch_logger():
            self.get_logger().warning('Failed to launch logger, continuing without logging')
        
        # Wait a moment before starting test
        self.get_logger().info('='*70)
        self.get_logger().info('Starting sinusoidal torque injection in 2 seconds...')
        time.sleep(2.0)
        
        # Start logger 100ms before trajectory execution
        time.sleep(0.1)
        if self.logger_start_client:
            self.start_logger()
        
        # Small delay to ensure logger is recording before first torque command
        time.sleep(0.01)
        
        self.get_logger().info('='*70)
        self.get_logger().info('SINUSOIDAL TEST EXECUTION (Open-loop torque control)')
        self.get_logger().info('='*70)
        
        self.test_active = True
        
        # Create 100Hz timer for torque injection
        self.test_timer = self.create_timer(0.01, self.test_callback)
        
        # Spin until test completes
        while self.test_active and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)
        
        # Stop logger 100ms after test completion
        time.sleep(0.1)
        if self.logger_stop_client:
            self.stop_logger()
        
        # Give logger time to write final data
        time.sleep(0.1)
        
        # Close CSV file
        self.close_csv()
        
        self.get_logger().info('Torque publisher shutting down.')


def main(args=None):
    rclpy.init(args=args)
    
    # Configuration parameters
    max_torque = [1.0, 35.0, 20.0]      # Maximum torque amplitude (Nm)
    period = 30.0          # Period for one complete cycle (seconds)
    duration = 30.0        # Total test duration (seconds) - 1 cycles
    
    node = SinusoidalTorquePublisher(
        max_torque=max_torque,
        period=period,
        duration=duration
    )
    
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
        
        # Close CSV file on shutdown
        node.close_csv()
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()