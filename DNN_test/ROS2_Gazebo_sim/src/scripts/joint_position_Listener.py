import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math

# ANSI color codes
COLORS = {
    'joint_1': '\033[92m',  # Green
    'joint_2': '\033[94m',  # Blue
    'joint_3': '\033[93m',  # Yellow
    'ENDC': '\033[0m'
}

class PositionListener(Node):
    def __init__(self):
        super().__init__('position_listener')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.last_print_time = time.time()
        self.latest_msg = None

    def listener_callback(self, msg):
        self.latest_msg = msg
        current_time = time.time()
        if current_time - self.last_print_time >= 0.5:
            for name, position in zip(msg.name, msg.position):
                degrees = math.degrees(position)
                color = COLORS.get(name, COLORS['ENDC'])
                print(f"{color}Joint {name}: position={position:.4f} rad, {degrees:.2f} deg{COLORS['ENDC']}")
            self.last_print_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = PositionListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()