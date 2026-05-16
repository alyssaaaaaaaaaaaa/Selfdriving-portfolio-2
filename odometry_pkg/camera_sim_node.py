"""
camera_sim_node.py

Deze ROS2-node simuleert een eenvoudige monoculaire camera voor taak 2
van het self-driving portfolio.

De node maakt kunstmatige camerabeelden met simpele geometrische vormen
zoals een rechthoek, cirkel en lijn. Deze vormen bewegen langzaam door
het beeld. Daardoor ontstaan duidelijke hoeken en patronen die door de
SLAM-node gebruikt kunnen worden voor feature detection en optical flow.

De beelden worden gepubliceerd op het ROS2-topic:

    /camera/image_raw

Dit topic wordt gebruikt als input voor de slam_node.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraSimNode(Node):
    """
    ROS2-node die gesimuleerde camerabeelden publiceert.

    De node genereert iedere 0.1 seconde een nieuw beeld. In het beeld
    staan zwarte vormen op een witte achtergrond. Door de offset veranderen
    sommige vormen van positie, waardoor beweging in het camerabeeld ontstaat.
    """

    def __init__(self):
        """
        Initialiseert de camera simulator.

        Hierbij worden de publisher, CvBridge en timer aangemaakt.
        De publisher stuurt beelden naar /camera/image_raw.
        """
        super().__init__('camera_sim_node')

        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        self.t = 0
        self.timer = self.create_timer(0.1, self.publish_image)

        self.get_logger().info('Camera simulator gestart.')

    def publish_image(self):
        """
        Maakt een nieuw gesimuleerd camerabeeld en publiceert dit als ROS Image.

        Het beeld bestaat uit:
        - een witte achtergrond;
        - een bewegende zwarte rechthoek;
        - een bewegende zwarte cirkel;
        - een zwarte lijn.

        Deze vormen leveren duidelijke features op, zoals hoeken en randen.
        """
        img = np.ones((480, 640, 3), dtype=np.uint8) * 255

        offset = self.t % 200

        cv2.rectangle(img, (100 + offset, 100), (180 + offset, 180), (0, 0, 0), -1)
        cv2.circle(img, (400 - offset, 300), 40, (0, 0, 0), -1)
        cv2.line(img, (50, 400), (600, 400), (0, 0, 0), 5)

        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.publisher.publish(msg)

        self.t += 5


def main(args=None):
    """
    Startpunt van de ROS2-node.

    Initialiseert ROS2, start de CameraSimNode en houdt de node actief
    totdat deze wordt gestopt.
    """
    rclpy.init(args=args)
    node = CameraSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()