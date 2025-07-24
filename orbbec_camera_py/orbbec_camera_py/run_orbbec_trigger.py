# Copyright (C) 2023, Locus Robotics. All rights reserved.
# Unauthorized copying of this file, via any medium, is strictly prohibited
# Proprietary and confidential

from typing import cast

import time
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from orbbec_camera_msgs.srv import CameraTrigger


class OrbbecTriggerNode(Node):
    def __init__(self):
        super().__init__("orbbec_camera_trigger")

        self.declare_parameter("camera_name", rclpy.Parameter.Type.STRING)
        self.declare_parameter("trigger_period", rclpy.Parameter.Type.DOUBLE)

        self.img_publisher = self.create_publisher(
            Image, "~/trigger_rgb_image", 1
        )
        self.bridge = CvBridge()

        self.camera_name = cast(str, self.get_parameter("camera_name").value)
        self.trigger_period = cast(float, self.get_parameter("trigger_period").value)

        self.client = self.create_client(CameraTrigger, self.camera_name + "/send_service_trigger")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")
        self.get_logger().info("Service available")
        self.request = CameraTrigger.Request()

    def spin(self):
        time.sleep(self.trigger_period)
        self.get_logger().info("Calling service...")
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)

        if self.future.result() is not None and self.future.result().success:
            response = self.future.result()

            self.img_publisher.publish(self.future.result().rgb_image)
            self.get_logger().error("Received Images!")
        else:
            self.get_logger().error("Service call failed!")


def main(args=None):
    rclpy.init(args=args)
    node = OrbbecTriggerNode()

    while rclpy.ok():
        try:
            node.spin()
        except KeyboardInterrupt:
            pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
