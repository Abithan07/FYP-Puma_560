import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class TorquePublisher(Node):
    def __init__(self):
        super().__init__('torque_publisher')
        self.pub1 = self.create_publisher(Float64MultiArray, '/joint_1_controller/commands', 10)
        self.pub2 = self.create_publisher(Float64MultiArray, '/joint_2_controller/commands', 10)
        self.pub3 = self.create_publisher(Float64MultiArray, '/joint_3_controller/commands', 10)
        self.timer_period = 10.0
        self.values_pos = [10.0, 15.0, 2.0]
        self.values_neg = [-10.0, -15.0, -2.0]

    def run(self):
        while rclpy.ok():
            msg1 = Float64MultiArray()
            msg2 = Float64MultiArray()
            msg3 = Float64MultiArray()
            msg1.data = [self.values_pos[0]]
            msg2.data = [self.values_pos[1]]
            msg3.data = [self.values_pos[2]]
            self.get_logger().info(f'Publishing torques: {self.values_pos}')
            self.pub1.publish(msg1)
            self.pub2.publish(msg2)
            self.pub3.publish(msg3)
            time.sleep(self.timer_period)

            msg1.data = [self.values_neg[0]]
            msg2.data = [self.values_neg[1]]
            msg3.data = [self.values_neg[2]]
            self.get_logger().info(f'Publishing torques: {self.values_neg}')
            self.pub1.publish(msg1)
            self.pub2.publish(msg2)
            self.pub3.publish(msg3)
            time.sleep(self.timer_period)

def main(args=None):
    rclpy.init(args=args)
    node = TorquePublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()