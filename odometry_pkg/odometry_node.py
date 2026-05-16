"""
odometry_node.py

Deze ROS2-node berekent de positie van de robot met behulp van
wiel-encoderdata voor taak 1 van het self-driving portfolio.

De node ontvangt encoder ticks van het linker- en rechterwiel en gebruikt
differentiële drive-kinematica om de positie en oriëntatie van de robot
te schatten.

De node ontvangt data van:

    /wheel_encoders

De berekende positie wordt gepubliceerd op:

    /odometry

De positie bestaat uit:
- x-positie;
- y-positie;
- rotatiehoek (theta).
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose2D


class OdometryNode(Node):
    """
    ROS2-node voor odometrieberekening.

    De node gebruikt encoder ticks van beide wielen om
    de beweging van de robot te berekenen. Hiervoor wordt
    differentiële drive-kinematica toegepast.
    """

    def __init__(self):
        """
        Initialiseert de odometry node.

        Hierbij worden:
        - een subscriber voor encoderdata aangemaakt;
        - een publisher voor de berekende pose aangemaakt;
        - robotparameters en beginwaarden ingesteld.
        """
        super().__init__('odometry_node')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/wheel_encoders',
            self.encoder_callback,
            10
        )

        self.pose_publisher = self.create_publisher(
            Pose2D,
            '/odometry',
            10
        )

        # Robot parameters
        self.wheel_radius = 0.0318
        self.wheel_base = 0.10
        self.ticks_per_revolution = 20.0

        # Huidige positie van de robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Vorige encoderwaarden
        self.prev_left_ticks = None
        self.prev_right_ticks = None

        self.get_logger().info('Odometry node gestart.')

    def encoder_callback(self, msg):
        """
        Verwerkt nieuwe encoderdata en berekent de robotpositie.

        De functie:
        1. leest linker- en rechter encoder ticks;
        2. berekent de verandering sinds de vorige meting;
        3. zet ticks om naar afgelegde afstand;
        4. berekent translatie en rotatie;
        5. werkt de pose van de robot bij.
        """
        left_ticks = msg.data[0]
        right_ticks = msg.data[1]

        # Eerste meting opslaan
        if self.prev_left_ticks is None or self.prev_right_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            return

        # Verschil in ticks berekenen
        delta_left_ticks = left_ticks - self.prev_left_ticks
        delta_right_ticks = right_ticks - self.prev_right_ticks

        # Vorige waarden updaten
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        # Ticks omzetten naar afstand
        left_distance = (
            2 * math.pi * self.wheel_radius *
            (delta_left_ticks / self.ticks_per_revolution)
        )

        right_distance = (
            2 * math.pi * self.wheel_radius *
            (delta_right_ticks / self.ticks_per_revolution)
        )

        # Gemiddelde translatie
        delta_s = (left_distance + right_distance) / 2.0

        # Verandering in rotatie
        delta_theta = (
            (right_distance - left_distance) /
            self.wheel_base
        )

        # Pose bijwerken
        self.x += delta_s * math.cos(
            self.theta + delta_theta / 2.0
        )

        self.y += delta_s * math.sin(
            self.theta + delta_theta / 2.0
        )

        self.theta += delta_theta

        # Pose publiceren
        pose_msg = Pose2D()

        pose_msg.x = self.x
        pose_msg.y = self.y
        pose_msg.theta = self.theta

        self.pose_publisher.publish(pose_msg)

        self.get_logger().info(
            f'Pose -> '
            f'x: {self.x:.3f}, '
            f'y: {self.y:.3f}, '
            f'theta: {self.theta:.3f}'
        )


def main(args=None):
    """
    Startpunt van de ROS2-node.

    Initialiseert ROS2, start de OdometryNode en houdt
    de node actief totdat deze wordt gestopt.
    """
    rclpy.init(args=args)

    node = OdometryNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()