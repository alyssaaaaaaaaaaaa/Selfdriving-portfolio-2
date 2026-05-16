import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D


class FusionNode(Node):

    def __init__(self):
        super().__init__('fusion_node')

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0

        self.visual_x = 0.0
        self.visual_y = 0.0
        self.visual_theta = 0.0

        self.fused_pub = self.create_publisher(
            Pose2D,
            '/fused_pose',
            10
        )

        self.create_subscription(
            Pose2D,
            '/odometry',
            self.odom_callback,
            10
        )

        self.create_subscription(
            Pose2D,
            '/visual_motion',
            self.visual_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.publish_fused_pose)

        self.get_logger().info('Fusion node gestart.')

    def odom_callback(self, msg):
        self.odom_x = msg.x
        self.odom_y = msg.y
        self.odom_theta = msg.theta

    def visual_callback(self, msg):
        self.visual_x = msg.x
        self.visual_y = msg.y
        self.visual_theta = msg.theta

    def publish_fused_pose(self):

        fused_msg = Pose2D()

        # simpele sensorfusie
        fused_msg.x = 0.7 * self.odom_x + 0.3 * self.visual_x
        fused_msg.y = 0.7 * self.odom_y + 0.3 * self.visual_y
        fused_msg.theta = 0.7 * self.odom_theta + 0.3 * self.visual_theta

        self.fused_pub.publish(fused_msg)

        self.get_logger().info(
            f'Fused pose -> '
            f'x: {fused_msg.x:.3f}, '
            f'y: {fused_msg.y:.3f}, '
            f'theta: {fused_msg.theta:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = FusionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()