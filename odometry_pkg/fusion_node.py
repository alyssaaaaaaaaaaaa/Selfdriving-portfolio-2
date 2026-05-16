"""
fusion_node.py

Deze ROS2-node voert eenvoudige sensorfusie uit voor taak 3 van het
self-driving portfolio.

De node combineert twee verschillende schattingen van de robotpositie:

1. Odometry-data afkomstig van wiel-encoders.
2. Visuele bewegingsdata afkomstig van de SLAM-node.

Door beide bronnen te combineren ontstaat een stabielere en betrouwbaardere
positie-inschatting van de robot.

De node ontvangt data van de volgende topics:

    /odometry
    /visual_motion

De gecombineerde positie wordt gepubliceerd op:

    /fused_pose
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D


class FusionNode(Node):
    """
    ROS2-node voor eenvoudige sensorfusie.

    De node combineert odometrie en visuele beweging met behulp
    van een gewogen gemiddelde. Hierbij krijgt odometrie meer
    gewicht dan de visuele schatting.
    """

    def __init__(self):
        """
        Initialiseert de fusion node.

        Hierbij worden:
        - subscribers voor odometrie en visuele beweging aangemaakt;
        - een publisher voor de gecombineerde pose aangemaakt;
        - beginwaarden voor positie en oriëntatie ingesteld.
        """
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
        """
        Ontvangt nieuwe odometriegegevens.

        Deze gegevens zijn afkomstig van de odometry_node,
        die positie berekent op basis van wiel-encoderdata.
        """
        self.odom_x = msg.x
        self.odom_y = msg.y
        self.odom_theta = msg.theta

    def visual_callback(self, msg):
        """
        Ontvangt nieuwe visuele bewegingsdata.

        Deze gegevens zijn afkomstig van de SLAM-node,
        die beweging schat op basis van feature tracking
        in camerabeelden.
        """
        self.visual_x = msg.x
        self.visual_y = msg.y
        self.visual_theta = msg.theta

    def publish_fused_pose(self):
        """
        Combineert odometrie en visuele beweging tot één pose.

        Voor de sensorfusie wordt een gewogen gemiddelde gebruikt:
        - 70% gewicht voor odometrie;
        - 30% gewicht voor visuele beweging.

        De gecombineerde positie wordt gepubliceerd op /fused_pose.
        """
        fused_msg = Pose2D()

        # Simpele sensorfusie
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
    """
    Startpunt van de ROS2-node.

    Initialiseert ROS2, start de FusionNode en houdt
    de node actief totdat deze wordt gestopt.
    """
    rclpy.init(args=args)

    node = FusionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()