import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.prev_gray = None
        self.prev_points = None

        self.get_logger().info('SLAM node gestart. Wacht op camerabeelden...')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is None:
            self.prev_points = cv2.goodFeaturesToTrack(
                gray,
                maxCorners=100,
                qualityLevel=0.3,
                minDistance=7,
                blockSize=7
            )
            self.prev_gray = gray
            self.get_logger().info('Eerste features gevonden.')
            return

        next_points, status, error = cv2.calcOpticalFlowPyrLK(
            self.prev_gray,
            gray,
            self.prev_points,
            None
        )

        if next_points is None:
            self.prev_gray = gray
            return

        good_new = next_points[status == 1]
        good_old = self.prev_points[status == 1]

        for new, old in zip(good_new, good_old):
            x_new, y_new = new.ravel()
            x_old, y_old = old.ravel()

            cv2.circle(frame, (int(x_new), int(y_new)), 5, (0, 255, 0), -1)
            cv2.line(
                frame,
                (int(x_new), int(y_new)),
                (int(x_old), int(y_old)),
                (255, 0, 0),
                2
            )

        cv2.imshow('Vision-based Monocular SLAM - Feature Tracking', frame)
        cv2.waitKey(1)

        self.prev_gray = gray.copy()
        self.prev_points = good_new.reshape(-1, 1, 2)


def main(args=None):
    rclpy.init(args=args)
    node = SlamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()