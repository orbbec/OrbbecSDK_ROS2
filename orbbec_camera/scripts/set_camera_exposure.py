#!/usr/bin/python3

from cgi import test
import sys
from urllib import response
from orbbec_camera_msgs.srv import SetInt32
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
import rclpy


class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")
        sensor_name = str(sys.argv[1])
        assert not sensor_name is None
        self.cli = self.create_client(
            SetInt32, "/camera/set/" + sensor_name + "/exposure"
        )
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = SetInt32.Request()

    def send_request(self):
        self.req.data = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    test_node.send_request()
    while rclpy.ok():
        rclpy.spin_once(test_node)
        if test_node.future.done():
            try:
                response = test_node.future.result()
            except Exception as e:
                test_node.get_logger().info("Service call failed %r" % (e,))
        else:
            test_node.get_logger().info("set camera exposure success")
            break

    test_node.destry_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
