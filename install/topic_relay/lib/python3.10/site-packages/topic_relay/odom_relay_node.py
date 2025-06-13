#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomRelayNode(Node):
    def __init__(self):
        super().__init__('odom_relay_node')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/diff_drive_base_controller/odom',
            self.odom_callback,
            10
        )
        self.get_logger().info('Odom relay node has started.')

    def odom_callback(self, msg):
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomRelayNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
