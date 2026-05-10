import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose2D


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/wheel_encoders',
            self.encoder_callback,
            10
        )

        self.pose_publisher = self.create_publisher(Pose2D, '/odometry', 10)

        # Robot parameters
        self.wheel_radius = 0.0318
        self.wheel_base = 0.10
        self.ticks_per_revolution = 20.0

        # Pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Vorige encoderwaarden
        self.prev_left_ticks = None
        self.prev_right_ticks = None

        self.get_logger().info('Odometry node gestart.')

    def encoder_callback(self, msg):
        left_ticks = msg.data[0]
        right_ticks = msg.data[1]

        if self.prev_left_ticks is None or self.prev_right_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            return

        delta_left_ticks = left_ticks - self.prev_left_ticks
        delta_right_ticks = right_ticks - self.prev_right_ticks

        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        left_distance = 2 * math.pi * self.wheel_radius * (delta_left_ticks / self.ticks_per_revolution)
        right_distance = 2 * math.pi * self.wheel_radius * (delta_right_ticks / self.ticks_per_revolution)

        delta_s = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base

        self.x += delta_s * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_s * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        pose_msg = Pose2D()
        pose_msg.x = self.x
        pose_msg.y = self.y
        pose_msg.theta = self.theta

        self.pose_publisher.publish(pose_msg)

        self.get_logger().info(
            f'Pose -> x: {self.x:.3f}, y: {self.y:.3f}, theta: {self.theta:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()