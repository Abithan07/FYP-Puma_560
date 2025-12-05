# Import the main ROS 2 Python library
import rclpy
# Import the Node class from rclpy.node
from rclpy.node import Node
# Import the JointState message type (for subscribing)
from sensor_msgs.msg import JointState
# Import the Float64MultiArray message type (for publishing)
from std_msgs.msg import Float64MultiArray

# Import numpy for efficient array calculations
import numpy as np

class PidControllerNode(Node):

    def __init__(self):
        # Initialize the Node with the name 'pid_controller_node'
        super().__init__('pid_controller_node')
        self.get_logger().info('PID Controller Node has started.')

        # --- Parameters ---
        # Set our target position (in radians)
        # We want joint_1 at 0.5 rad, joint_2 at 0.5 rad, joint_3 at 0.5 rad
        self.setpoint = np.array([0.5, 0.5, 0.5])
        
        # Define our PID gains (Proportional, Integral, Derivative)
        # We need one set of gains for each of the 3 joints
        # Format: [joint_1_gains, joint_2_gains, joint_3_gains]
        self.kp = np.array([50.0, 50.0, 50.0])  # Proportional gain (handles present error)
        self.ki = np.array([0.1, 0.1, 0.1])    # Integral gain (handles past error buildup)
        self.kd = np.array([5.0, 5.0, 5.0])    # Derivative gain (predicts future error)

        # --- Internal PID state variables ---
        # We need to store these for each joint
        self.prev_error = np.array([0.0, 0.0, 0.0])
        self.integral = np.array([0.0, 0.0, 0.0])
        
        # --- Robot state variables ---
        # Stores the current position of all joints. We'll update this
        # in the joint_state_callback.
        self.current_positions = np.array([0.0, 0.0, 0.0])
        
        # A dictionary to map joint names to their index in our arrays
        # This is CRITICAL because /joint_states can publish in any order
        self.joint_name_to_index = {
            'joint_1': 0,
            'joint_2': 1,
            'joint_3': 2
        }

        # --- Create Publishers ---
        # We need one publisher for each joint's torque command topic
        self.pub_j1 = self.create_publisher(Float64MultiArray, '/joint_1_controller/commands', 10)
        self.pub_j2 = self.create_publisher(Float64MultiArray, '/joint_2_controller/commands', 10)
        self.pub_j3 = self.create_publisher(Float64MultiArray, '/joint_3_controller/commands', 10)
        
        # Store them in a list for easy access
        self.pubs = [self.pub_j1, self.pub_j2, self.pub_j3]

        # --- Create Subscriber ---
        # This subscribes to the robot's sensor feed (/joint_states)
        # Every time a new message comes in, it calls self.joint_state_callback
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

    def joint_state_callback(self, msg):
        """
        This function is called every time a new /joint_states message is received.
        """
        
        # --- 1. Read Current Robot State ---
        # The 'msg' object contains the names and positions of ALL joints
        # We need to loop through them and update our 'current_positions' array
        for i, name in enumerate(msg.name):
            if name in self.joint_name_to_index:
                # Find the index (0, 1, or 2) corresponding to the joint name
                index = self.joint_name_to_index[name]
                # Update our array with the position from the message
                self.current_positions[index] = msg.position[i]
        
        # --- 2. Calculate PID Control Law ---
        
        # Calculate the error (Setpoint - Current Position)
        error = self.setpoint - self.current_positions
        
        # Calculate the Integral term (accumulated error)
        # We add a check to prevent "integral windup"
        self.integral += error
        # Clamp the integral to a reasonable range
        self.integral = np.clip(self.integral, -10.0, 10.0)

        # Calculate the Derivative term (rate of change of error)
        derivative = error - self.prev_error
        
        # --- 3. Calculate Final Torque ---
        # This is the PID formula: tau = Kp*e + Ki*Integral(e) + Kd*Derivative(e)
        torque = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        # Update the previous error for the next loop
        self.prev_error = error

        # --- 4. Publish Torque Commands ---
        
        # Create a new message for each joint and publish the torque
        for i in range(3):
            torque_msg = Float64MultiArray()
            # The data field must be a list, e.g., [10.0]
            torque_msg.data = [torque[i]]
            self.pubs[i].publish(torque_msg)

        # Log the info to the terminal (so we can see what's happening)
        # We'll uncomment this if we need to debug
        # self.get_logger().info(f'Setpoint: {self.setpoint}')
        # self.get_logger().info(f'Position: {np.round(self.current_positions, 2)}')
        # self.get_logger().info(f'Error:    {np.round(error, 2)}')
        # self.get_logger().info(f'Torque:   {np.round(torque, 2)}')


def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)
    
    # Create the PID Controller Node
    pid_controller = PidControllerNode()
    
    # 'spin' the node, which means it will stay alive,
    # processing callbacks (like joint_state_callback) until you press Ctrl+C
    rclpy.spin(pid_controller)
    
    # Clean up and shutdown
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()