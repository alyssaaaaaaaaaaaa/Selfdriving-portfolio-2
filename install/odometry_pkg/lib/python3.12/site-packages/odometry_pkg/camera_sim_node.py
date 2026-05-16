import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraSimNode(Node):
    def __init__(self):
        super().__init__('camera_sim_node')

        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        self.t = 0
        self.timer = self.create_timer(0.1, self.publish_image)

        self.get_logger().info('Camera simulator gestart.')

    def publish_image(self):
        img = np.ones((480, 640, 3), dtype=np.uint8) * 255

        offset = self.t % 200

        cv2.rectangle(img, (100 + offset, 100), (180 + offset, 180), (0, 0, 0), -1)
        cv2.circle(img, (400 - offset, 300), 40, (0, 0, 0), -1)
        cv2.line(img, (50, 400), (600, 400), (0, 0, 0), 5)

        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.publisher.publish(msg)

        self.t += 5


def main(args=None):
    rclpy.init(args=args)
    node = CameraSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()