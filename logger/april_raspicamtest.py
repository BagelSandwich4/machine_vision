"""Test file to ensure AprilTags are detected via the Raspberry Pi camera."""
import cv2
from cv_bridge import CvBridge
from pupil_apriltags import Detector
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class PiCamSubscriber(Node):
    """Subscribe to the Pi camera feed and draw detected AprilTags."""

    def __init__(self):
        """Set up the image subscription, cvbridge, and tag detector."""
        super().__init__('picam_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.detector = Detector(families='tag25h9')

    def listener_callback(self, msg):
        """Detect tags in an incoming frame, log them, and display the result."""
        try:
            # Convert ROS Image message to OpenCV (BGR) image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)

        for r in results:
            self.get_logger().info(f'Detected Tag ID: {r.tag_id} at center {r.center}')

            corners = r.corners.astype(int)
            cv2.polylines(frame, [corners.reshape(-1, 1, 2)], True, (0, 255, 0), 2)

            cx, cy = r.center.astype(int)
            cv2.putText(frame, str(r.tag_id), (cx, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.imshow('AprilTag Detection', frame)
        cv2.waitKey(1)


def main(args=None):
    """Run the node until interrupted."""
    rclpy.init(args=args)
    node = PiCamSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
