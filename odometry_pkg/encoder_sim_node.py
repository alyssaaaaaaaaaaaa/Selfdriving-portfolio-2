"""
encoder_sim_node.py

Deze ROS2-node simuleert wiel-encoderdata voor taak 1 van het
self-driving portfolio.

De node genereert kunstmatige encoder ticks voor het linker- en rechterwiel.
Deze ticks worden gebruikt door de odometry_node om de positie en oriëntatie
van de robot te berekenen met behulp van differentiële drive-kinematica.

De encoderwaarden worden gepubliceerd op het ROS2-topic:

    /wheel_encoders

Dit topic wordt gebruikt als input voor de odometry_node.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class EncoderSimNode(Node):
    """
    ROS2-node die gesimuleerde encoderdata publiceert.

    De node simuleert draaiende wielen door encoder ticks iedere
    0.1 seconde te verhogen. Het linker- en rechterwiel kunnen
    verschillende snelheden hebben, waardoor een bochtbeweging
    ontstaat in de odometrieberekening.
    """

    def __init__(self):
        """
        Initialiseert de encoder simulator.

        Hierbij worden de publisher, timer en beginwaarden
        van de encoder ticks aangemaakt.
        """
        super().__init__('encoder_sim_node')

        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            '/wheel_encoders',
            10
        )

        self.timer = self.create_timer(0.1, self.publish_encoder_data)

        self.dt = 0.1
        self.left_ticks = 0.0
        self.right_ticks = 0.0

        # Gesimuleerde encoder ticks per seconde
        self.left_ticks_per_sec = 60.0
        self.right_ticks_per_sec = 80.0

        self.get_logger().info('Encoder simulator gestart.')

    def publish_encoder_data(self):
        """
        Verhoogt de encoder ticks en publiceert deze als ROS2-bericht.

        Het bericht bevat:
        - totale ticks van het linkerwiel;
        - totale ticks van het rechterwiel.

        Omdat het rechterwiel sneller draait dan het linkerwiel,
        simuleert de robot een draaiende beweging.
        """
        self.left_ticks += self.left_ticks_per_sec * self.dt
        self.right_ticks += self.right_ticks_per_sec * self.dt

        msg = Float32MultiArray()
        msg.data = [self.left_ticks, self.right_ticks]

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'Ticks -> left: {self.left_ticks:.2f}, '
            f'right: {self.right_ticks:.2f}'
        )


def main(args=None):
    """
    Startpunt van de ROS2-node.

    Initialiseert ROS2, start de EncoderSimNode en houdt
    de node actief totdat deze wordt gestopt.
    """
    rclpy.init(args=args)

    node = EncoderSimNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()