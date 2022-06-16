#!/usr/bin/env python3
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
import rclpy
from rclpy.node import Node


class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")
        self.subscription = self.create_subscription(
            PointCloud2, "/camera/depth/points", self.listener_callback, qos_profile_sensor_data
        )

    def listener_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)

    test_node = TestNode()

    rclpy.spin(test_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
