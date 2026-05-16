"""
slam_node.py

Deze ROS2-node implementeert een eenvoudige vision-based monocular
SLAM-oplossing voor taak 2 van het self-driving portfolio.

De node ontvangt camerabeelden en gebruikt OpenCV om:
- features (hoeken/randen) te detecteren;
- features tussen frames te volgen;
- visuele beweging van de camera te schatten;
- een eenvoudige feature-map op te bouwen.

De node ontvangt camerabeelden van:

    /camera/image_raw

De node publiceert:
- visuele bewegingsschatting op /visual_motion;
- feature-afbeeldingen op /slam/feature_image.

Daarnaast wordt een afbeelding met gedetecteerde features opgeslagen
op het systeem voor demonstratie- en debugdoeleinden.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2
import numpy as np


class SLAMNode(Node):
    """
    ROS2-node voor eenvoudige vision-based monocular SLAM.

    De node gebruikt OpenCV om features in camerabeelden te detecteren
    en te volgen met optical flow. Op basis van de gemiddelde beweging
    van features wordt een schatting gemaakt van de camerabeweging.
    """

    def __init__(self):
        """
        Initialiseert de SLAM node.

        Hierbij worden:
        - subscribers voor camerabeelden aangemaakt;
        - publishers voor visual motion en feature-afbeeldingen aangemaakt;
        - variabelen voor feature tracking geïnitialiseerd.
        """
        super().__init__('slam_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.motion_pub = self.create_publisher(
            Pose2D,
            '/visual_motion',
            10
        )

        self.feature_image_pub = self.create_publisher(
            Image,
            '/slam/feature_image',
            10
        )

        # Vorig frame in grayscale
        self.prev_gray = None

        # Vorige featurepunten
        self.prev_points = None

        # Simpele feature-map
        self.map_points = []

        # Visuele bewegingsschatting
        self.visual_x = 0.0
        self.visual_y = 0.0
        self.visual_theta = 0.0

        self.get_logger().info(
            'SLAM node gestart. Wacht op camerabeelden...'
        )

    def image_callback(self, msg):
        """
        Verwerkt een nieuw camerabeeld.

        De functie:
        1. converteert het beeld naar grayscale;
        2. detecteert features;
        3. volgt features met optical flow;
        4. schat camerabeweging;
        5. tekent features op het beeld;
        6. publiceert de resultaten.
        """
        # ROS Image omzetten naar OpenCV afbeelding
        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        # Omzetten naar grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Eerste frame initialiseren
        if self.prev_gray is None:

            self.prev_gray = gray

            self.prev_points = cv2.goodFeaturesToTrack(
                gray,
                maxCorners=100,
                qualityLevel=0.3,
                minDistance=7,
                blockSize=7
            )

            self.get_logger().info(
                'Eerste features gevonden.'
            )

            return

        # Optical flow berekenen
        next_points, status, error = cv2.calcOpticalFlowPyrLK(
            self.prev_gray,
            gray,
            self.prev_points,
            None
        )

        # Nieuwe features zoeken indien tracking mislukt
        if next_points is None or status is None:

            self.prev_gray = gray

            self.prev_points = cv2.goodFeaturesToTrack(
                gray,
                maxCorners=100,
                qualityLevel=0.3,
                minDistance=7,
                blockSize=7
            )

            return

        # Alleen geldige punten behouden
        good_new = next_points[status == 1]
        good_old = self.prev_points[status == 1]

        # Alleen verwerken indien voldoende features bestaan
        if len(good_new) > 5:

            movement = good_new - good_old

            # Gemiddelde featurebeweging berekenen
            avg_dx = np.mean(movement[:, 0])
            avg_dy = np.mean(movement[:, 1])

            scale = 0.001

            # Visuele positie bijwerken
            self.visual_x += -avg_dx * scale
            self.visual_y += -avg_dy * scale
            self.visual_theta += 0.0

            # Visual motion publiceren
            motion_msg = Pose2D()

            motion_msg.x = self.visual_x
            motion_msg.y = self.visual_y
            motion_msg.theta = self.visual_theta

            self.motion_pub.publish(motion_msg)

            # Punten toevoegen aan feature-map
            for point in good_new:

                x, y = point.ravel()

                self.map_points.append(
                    (int(x), int(y))
                )

            # Grootte van map beperken
            if len(self.map_points) > 1000:
                self.map_points = self.map_points[-1000:]

            self.get_logger().info(
                f'Visual motion -> '
                f'x: {self.visual_x:.3f}, '
                f'y: {self.visual_y:.3f}, '
                f'features: {len(good_new)}'
            )

        # Features tekenen op afbeelding
        for new, old in zip(good_new, good_old):

            x_new, y_new = new.ravel()
            x_old, y_old = old.ravel()

            # Groen featurepunt
            cv2.circle(
                frame,
                (int(x_new), int(y_new)),
                5,
                (0, 255, 0),
                -1
            )

            # Blauwe trackinglijn
            cv2.line(
                frame,
                (int(x_new), int(y_new)),
                (int(x_old), int(y_old)),
                (255, 0, 0),
                2
            )

        # Simpele feature-map
        map_view = np.zeros(
            (500, 500, 3),
            dtype=np.uint8
        )

        for point in self.map_points:

            x, y = point

            map_x = int(x * 500 / frame.shape[1])
            map_y = int(y * 500 / frame.shape[0])

            cv2.circle(
                map_view,
                (map_x, map_y),
                2,
                (0, 255, 0),
                -1
            )

        # Feature-afbeelding publiceren
        feature_msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding='bgr8'
        )

        self.feature_image_pub.publish(feature_msg)

        # Feature-afbeelding opslaan
        cv2.imwrite(
            '/tmp/slam_feature_image.png',
            frame
        )

        # Huidige frame opslaan voor volgende iteratie
        self.prev_gray = gray

        self.prev_points = good_new.reshape(
            -1,
            1,
            2
        )

        # Nieuwe features zoeken indien te weinig punten overblijven
        if len(self.prev_points) < 20:

            self.prev_points = cv2.goodFeaturesToTrack(
                gray,
                maxCorners=100,
                qualityLevel=0.3,
                minDistance=7,
                blockSize=7
            )


def main(args=None):
    """
    Startpunt van de ROS2-node.

    Initialiseert ROS2, start de SLAMNode en houdt
    de node actief totdat deze wordt gestopt.
    """
    rclpy.init(args=args)

    node = SLAMNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()