import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class EncoderSimNode(Node):
    def __init__(self):
        super().__init__('encoder_sim_node')

        self.publisher_ = self.create_publisher(Float32MultiArray, '/wheel_encoders', 10)
        self.timer = self.create_timer(0.1, self.publish_encoder_data)

        self.dt = 0.1
        self.left_ticks = 0.0
        self.right_ticks = 0.0

        # Gesimuleerde encoder ticks per seconde
        self.left_ticks_per_sec = 60.0
        self.right_ticks_per_sec = 80.0

        self.get_logger().info('Encoder simulator gestart.')

    def publish_encoder_data(self):
        self.left_ticks += self.left_ticks_per_sec * self.dt
        self.right_ticks += self.right_ticks_per_sec * self.dt

        msg = Float32MultiArray()
        msg.data = [self.left_ticks, self.right_ticks]
        self.publisher_.publish(msg)

        self.get_logger().info(
            f'Ticks -> left: {self.left_ticks:.2f}, right: {self.right_ticks:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = EncoderSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()