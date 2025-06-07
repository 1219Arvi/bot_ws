import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageFlipNode(Node):
    def __init__(self):
        super().__init__('image_flip')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.image_callback,
            10)
        self.pub = self.create_publisher(Image, '/flip_depth/image', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        flipped = cv2.flip(cv_image, 0)  # flip vertically
        flipped_msg = self.bridge.cv2_to_imgmsg(flipped, encoding=msg.encoding)
        flipped_msg.header = msg.header
        self.pub.publish(flipped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageFlipNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

