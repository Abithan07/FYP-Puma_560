#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener, TransformException
from std_msgs.msg import Bool

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')

        # Frames/offset can be tuned without code edits.
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('link_frame', 'link_3')
        self.declare_parameter('tip_offset_x', 0.0)
        self.declare_parameter('tip_offset_y', -0.233)
        self.declare_parameter('tip_offset_z', 0.0)
        self.declare_parameter('line_width', 0.01)
        self.declare_parameter('max_points', 1000)
        self.declare_parameter('recording_state_topic', '/line_drawer/recording_active')

        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.link_frame = self.get_parameter('link_frame').get_parameter_value().string_value
        self.tip_offset_x = self.get_parameter('tip_offset_x').get_parameter_value().double_value
        self.tip_offset_y = self.get_parameter('tip_offset_y').get_parameter_value().double_value
        self.tip_offset_z = self.get_parameter('tip_offset_z').get_parameter_value().double_value
        self.line_width = self.get_parameter('line_width').get_parameter_value().double_value
        self.max_points = self.get_parameter('max_points').get_parameter_value().integer_value
        self.recording_state_topic = self.get_parameter(
            'recording_state_topic'
        ).get_parameter_value().string_value

        # Create publisher on the standard marker topic
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.recording_active = False
        self.state_subscriber = self.create_subscription(
            Bool,
            self.recording_state_topic,
            self.recording_state_callback,
            state_qos,
        )
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_marker)
        self.points = []  # Store trail points
        self.current_run_id = 0
        self.run_colors = [
            (0.90, 0.20, 0.20),  # red
            (0.20, 0.70, 1.00),  # blue
            (0.20, 0.85, 0.30),  # green
            (1.00, 0.65, 0.10),  # orange
            (0.95, 0.30, 0.95),  # magenta
            (0.20, 0.90, 0.90),  # cyan
            (1.00, 0.95, 0.20),  # yellow
            (0.95, 0.55, 0.75),  # pink
        ]
        self.warn_every = 0

    def recording_state_callback(self, msg):
        new_state = bool(msg.data)
        if new_state and not self.recording_active:
            # Start a fresh line strip for each recording run so runs do not connect.
            self.current_run_id += 1
            self.points = []
        self.recording_active = new_state

    @staticmethod
    def rotate_vector_by_quaternion(x, y, z, qx, qy, qz, qw):
        # Rotate vector v by quaternion q: v' = q * v * q_conjugate.
        tx = 2.0 * (qy * z - qz * y)
        ty = 2.0 * (qz * x - qx * z)
        tz = 2.0 * (qx * y - qy * x)

        rx = x + qw * tx + (qy * tz - qz * ty)
        ry = y + qw * ty + (qz * tx - qx * tz)
        rz = z + qw * tz + (qx * ty - qy * tx)
        return rx, ry, rz

    def publish_marker(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.link_frame,
                Time()
            )
        except TransformException as ex:
            self.warn_every += 1
            if self.warn_every % 20 == 0:
                self.get_logger().warn(
                    f'No TF {self.target_frame} <- {self.link_frame} yet: {ex}'
                )
            return

        t = transform.transform.translation
        q = transform.transform.rotation

        # Compute tip point in world frame from link frame offset.
        ox, oy, oz = self.rotate_vector_by_quaternion(
            self.tip_offset_x,
            self.tip_offset_y,
            self.tip_offset_z,
            q.x,
            q.y,
            q.z,
            q.w,
        )

        if self.recording_active:
            point = Point()
            point.x = t.x + ox
            point.y = t.y + oy
            point.z = t.z + oz
            self.points.append(point)

            if len(self.points) > self.max_points:
                self.points.pop(0)

        marker = Marker()
        # Publish in world so trajectory stays spatially stable.
        marker.header.frame_id = self.target_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # Unique namespace and ID for this marker
        marker.ns = "end_effector_marker"
        marker.id = self.current_run_id
        
        # OPTION 1: Draw a LINE_STRIP (persistent trail)
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.points = self.points

        # Scale settings - line width
        marker.scale.x = self.line_width
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        # Color settings (RGBA) - Green line
        color = self.run_colors[(self.current_run_id - 1) % len(self.run_colors)]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0  # Opaque

        # Lifetime (0 means infinite)
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

        self.publisher_.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
