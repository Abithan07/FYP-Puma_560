import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time
import math
import os # Added for file path

# ANSI color codes for printing
COLORS = {
    'joint_1': '\033[92m',  # Green
    'joint_2': '\033[94m',  # Blue
    'joint_3': '\033[93m',  # Yellow
    'ENDC': '\033[0m'
}

class ArmTesterNode(Node):
    def __init__(self):
        super().__init__('arm_tester_node')
        
        # --- Parameters ---
        # Declare a parameter to choose the test mode.
        # You can change this on the command line when you run:
        # ros2 run <your_pkg> arm_tester_node --ros-args -p test_mode:=ramp
        self.declare_parameter('test_mode', 'sine')
        self.test_mode = self.get_parameter('test_mode').get_parameter_value().string_value
        self.get_logger().info(f'Starting test in "{self.test_mode}" mode.')

        # --- Publisher Setup ---
        self.get_logger().info('Creating torque publishers...')
        self.pub1 = self.create_publisher(Float64MultiArray, '/joint_1_controller/commands', 10)
        self.pub2 = self.create_publisher(Float64MultiArray, '/joint_2_controller/commands', 10)
        self.pub3 = self.create_publisher(Float64MultiArray, '/joint_3_controller/commands', 10)
        
        # --- Signal Parameters ---
        # Amplitudes for the torque signals (max torque)
        self.torque_amplitudes = [10.0, 45.0, 2.0]
        # Frequencies for the torque signals (in Hz - how many cycles per second)
        self.torque_frequencies = [0.1, 0.2, 0.3] # Slow frequencies for a big arm
        # For 'toggle' mode
        self.values_pos = [10.0, 45.0, 2.0]
        self.values_neg = [-10.0, -45.0, -2.0]
        self.toggle_period = 15.0 # seconds

        # --- Subscriber Setup ---
        self.get_logger().info('Creating joint state subscriber...')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        # Dictionary to store the most recent joint positions
        self.latest_positions = {}
        self.last_torques = [0.0, 0.0, 0.0]

        # --- Data Logging Setup ---
        log_filename = 'arm_test_log.csv'
        self.log_file = open(log_filename, 'w')
        self.log_file.write('timestamp,j1_pos_rad,j2_pos_rad,j3_pos_rad,j1_torque_cmd,j2_torque_cmd,j3_torque_cmd\n')
        self.get_logger().info(f'Logging data to {os.path.abspath(log_filename)}')

        # --- Timers ---
        self.start_time = self.get_clock().now()
        
        # High-frequency timer for publishing commands and logging data (50Hz)
        self.publish_log_timer_period = 0.02  # seconds (50 Hz)
        self.publish_timer = self.create_timer(self.publish_log_timer_period, self.publish_log_callback)
        
        # Low-frequency timer for printing to console (every 2 seconds)
        self.print_timer_period = 2.0 # seconds
        self.print_timer = self.create_timer(self.print_timer_period, self.print_status_callback)
        
        self.get_logger().info('Arm Tester Node has started.')

    def publish_log_callback(self):
        """
        High-frequency callback (50Hz).
        Calculates and publishes torques, then logs data.
        """
        msg1 = Float64MultiArray()
        msg2 = Float64MultiArray()
        msg3 = Float64MultiArray()
        
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        torques = [0.0, 0.0, 0.0]

        if self.test_mode == 'sine':
            for i in range(3):
                amplitude = self.torque_amplitudes[i]
                frequency = self.torque_frequencies[i]
                # A = amplitude, f = frequency, t = time
                # Torque = A * sin(2 * pi * f * t)
                torques[i] = amplitude * math.sin(2 * math.pi * frequency * elapsed_time)
        
        elif self.test_mode == 'ramp':
            for i in range(3):
                amplitude = self.torque_amplitudes[i]
                frequency = self.torque_frequencies[i]
                # Creates a sawtooth wave: ramps from 0 to Amplitude
                # The '% 1.0' makes the wave repeat
                torques[i] = amplitude * ((elapsed_time * frequency) % 1.0)
                
        elif self.test_mode == 'toggle':
            # This logic mimics your original 10-second toggle
            if int(elapsed_time / self.toggle_period) % 2 == 0:
                torques = self.values_pos
            else:
                torques = self.values_neg
        
        self.last_torques = torques
        msg1.data = [torques[0]]
        msg2.data = [torques[1]]
        msg3.data = [torques[2]]
        
        self.pub1.publish(msg1)
        self.pub2.publish(msg2)
        self.pub3.publish(msg3)
        
        # --- Log Data ---
        # Get the latest positions (or default to 0.0 if not yet received)
        p1 = self.latest_positions.get('joint_1', 0.0)
        p2 = self.latest_positions.get('joint_2', 0.0)
        p3 = self.latest_positions.get('joint_3', 0.0)
        
        # Write data to CSV
        self.log_file.write(f'{elapsed_time:.4f},{p1:.6f},{p2:.6f},{p3:.6f},{torques[0]:.4f},{torques[1]:.4f},{torques[2]:.4f}\n')

    def joint_state_callback(self, msg):
        """
        Low-priority callback.
        Triggered when /joint_states arrives.
        Simply stores the latest position data.
        """
        for i, name in enumerate(msg.name):
            if name in ['joint_1', 'joint_2', 'joint_3']:
                self.latest_positions[name] = msg.position[i]

    def print_status_callback(self):
        """
        Low-frequency callback (every 2s).
        Prints the latest status to the console.
        """
        self.get_logger().info('Current Status:')
        
        # Print positions
        for name in ['joint_1', 'joint_2', 'joint_3']:
            position = self.latest_positions.get(name, None)
            if position is not None:
                degrees = math.degrees(position)
                color = COLORS.get(name, COLORS['ENDC'])
                print(f"  {color}Joint {name}: position={position:.4f} rad, {degrees:.2f} deg{COLORS['ENDC']}")
            else:
                print(f"  Joint {name}: (No data yet)")

        # Print last commanded torques
        self.get_logger().info(f'  Last Torques: j1={self.last_torques[0]:.2f}, j2={self.last_torques[1]:.2f}, j3={self.last_torques[2]:.2f}')

    def close_log_file(self):
        """
        Called on shutdown to ensure the log file is closed properly.
        """
        if self.log_file:
            self.log_file.close()
            self.get_logger().info('Log file closed.')

def main(args=None):
    rclpy.init(args=args)
    
    arm_tester_node = ArmTesterNode()
    
    try:
        rclpy.spin(arm_tester_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        arm_tester_node.close_log_file()
        arm_tester_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

