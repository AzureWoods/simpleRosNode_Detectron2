import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'topic', 10)
        timer_period = 10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        cv2image = cv2.imread("/home/ubuntu/dev_ros2_ws/src/imgpub_pkg/imgpub_pkg/test.jpg")
        self.get_logger().info("%s" % type(cv2image))
        if isinstance(cv2image, (np.ndarray, np.generic)):
            self.get_logger().info("YES")
        else:
            self.get_logger().info("NO")
        bridge = CvBridge()
        self.image = bridge.cv2_to_imgmsg(cv2image)

    def timer_callback(self):
        self.publisher_.publish(self.image)
        self.get_logger().info('Publishing: No."%d" Image.' % self.i)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    image_publisher = ImagePublisher()

    rclpy.spin(image_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()