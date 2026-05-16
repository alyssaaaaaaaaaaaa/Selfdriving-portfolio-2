import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2
import numpy as np


class SLAMNode(Node):
    def __init__(self):
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

        self.prev_gray = None
        self.prev_points = None

        self.map_points = []

        self.visual_x = 0.0
        self.visual_y = 0.0
        self.visual_theta = 0.0

        self.get_logger().info('SLAM node gestart. Wacht op camerabeelden...')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is None:
            self.prev_gray = gray
            self.prev_points = cv2.goodFeaturesToTrack(
                gray,
                maxCorners=100,
                qualityLevel=0.3,
                minDistance=7,
                blockSize=7
            )
            self.get_logger().info('Eerste features gevonden.')
            return

        next_points, status, error = cv2.calcOpticalFlowPyrLK(
            self.prev_gray,
            gray,
            self.prev_points,
            None
        )

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

        good_new = next_points[status == 1]
        good_old = self.prev_points[status == 1]

        if len(good_new) > 5:
            movement = good_new - good_old

            avg_dx = np.mean(movement[:, 0])
            avg_dy = np.mean(movement[:, 1])

            scale = 0.001

            self.visual_x += -avg_dx * scale
            self.visual_y += -avg_dy * scale
            self.visual_theta += 0.0

            motion_msg = Pose2D()
            motion_msg.x = self.visual_x
            motion_msg.y = self.visual_y
            motion_msg.theta = self.visual_theta
            self.motion_pub.publish(motion_msg)

            for point in good_new:
                x, y = point.ravel()
                self.map_points.append((int(x), int(y)))

            if len(self.map_points) > 1000:
                self.map_points = self.map_points[-1000:]

            self.get_logger().info(
                f'Visual motion -> x: {self.visual_x:.3f}, '
                f'y: {self.visual_y:.3f}, '
                f'features: {len(good_new)}'
            )

        for new, old in zip(good_new, good_old):
            x_new, y_new = new.ravel()
            x_old, y_old = old.ravel()

            cv2.circle(frame, (int(x_new), int(y_new)), 5, (0, 255, 0), -1)
            cv2.line(frame, (int(x_new), int(y_new)), (int(x_old), int(y_old)), (255, 0, 0), 2)

        map_view = np.zeros((500, 500, 3), dtype=np.uint8)

        for point in self.map_points:
            x, y = point
            map_x = int(x * 500 / frame.shape[1])
            map_y = int(y * 500 / frame.shape[0])
            cv2.circle(map_view, (map_x, map_y), 2, (0, 255, 0), -1)

        #cv2.imshow('Vision-based Monocular SLAM - Feature Tracking', frame)
        #cv2.imshow('Simple Feature Map', map_view)
        #cv2.waitKey(1)

        feature_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.feature_image_pub.publish(feature_msg)
        cv2.imwrite('/tmp/slam_feature_image.png', frame)

        self.prev_gray = gray
        self.prev_points = good_new.reshape(-1, 1, 2)

        if len(self.prev_points) < 20:
            self.prev_points = cv2.goodFeaturesToTrack(
                gray,
                maxCorners=100,
                qualityLevel=0.3,
                minDistance=7,
                blockSize=7
            )


def main(args=None):
    rclpy.init(args=args)
    node = SLAMNode()
    rclpy.spin(node)
    node.destroy_node()
    #cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()