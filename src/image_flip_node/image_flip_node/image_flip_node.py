import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageFlipThrottleNode(Node):
    def __init__(self):
        super().__init__('image_flip_throttle')
        
        # Parameters
        self.declare_parameter('input_topic', '/camera/depth/image_raw')
        self.declare_parameter('output_topic', '/flip_depth/image')
        self.declare_parameter('rate', 10.0)  # Hz

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.latest_msg = None

        self.sub = self.create_subscription(Image, self.input_topic, self.image_callback, 10)
        self.pub = self.create_publisher(Image, self.output_topic, 10)

        self.timer = self.create_timer(1.0 / self.rate, self.publish_throttled_image)

        self.get_logger().info(f"Subscribed to: {self.input_topic}, publishing throttled to: {self.output_topic} at {self.rate} Hz")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            flipped = cv2.flip(cv_image, 0)  # vertical flip
            flipped_msg = self.bridge.cv2_to_imgmsg(flipped, encoding=msg.encoding)
            flipped_msg.header = msg.header
            self.latest_msg = flipped_msg
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def publish_throttled_image(self):
        if self.latest_msg:
            self.pub.publish(self.latest_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageFlipThrottleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
