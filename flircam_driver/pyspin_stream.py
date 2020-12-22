#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from simple_pyspin import Camera
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from ruamel.yaml import YAML
from pathlib import Path

yaml = YAML(typ="safe")


class SpinnakerCameraNode(Node):
    def __init__(self):
        # print(cam_identifier)
        super().__init__(
            "pyspin_stream",
            # allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # declare default parameters
        default_param = {
            "config_found": False,
            "cam_id": None,
            "camera_topic_base": "/camera/camera_x",
        }
        for key, value in default_param.items():
            if not self.has_parameter(key):
                self.declare_parameter(key, value)

        self.get_logger().info(
            f"Config found: {self.get_parameter('config_found').value}"
        )

        self.cam_id = self.get_parameter("cam_id").value
        self.get_logger().info(f"cam_id: {self.cam_id}")

        self.camera_topic_base = self.get_parameter("camera_topic_base").value

        # set all camera settings through pyspin simple
        self.cam_init()
        self.cam.start()

        self.publish_timer = self.create_timer(
            1 / self.cam_framerate, self.stream_camera
        )
        # create topic
        topic_name = f"{self.camera_topic_base}/image_mono"
        self.pub_stream = self.create_publisher(Image, topic_name, 1)
        self.bridge = CvBridge()
        self.get_logger().info("Node initialized")

    def cam_init(self):
        if self.cam_id is None:
            self.cam = Camera()  # Acquire Camera
        else:
            self.cam = Camera(self.cam_id)  # Acquire Camera
        self.cam.init()  # Initialize camera

        self.cam_id = self.cam.get_info("DeviceSerialNumber")["value"]

        self.cam_framerate = self.cam.get_info("AcquisitionFrameRate")["value"]

        parameter_dict = self.get_parameters_by_prefix("camera_settings")
        cam_dict = {}
        for param_name, param in parameter_dict.items():
            cam_dict[param_name] = self.get_parameter(
                f"camera_settings.{param_name}"
            ).value
        # self.get_logger().info(f"param_dict: {cam_dict}")
        for attribute_name, attribute_value in cam_dict.items():
            setattr(self.cam, attribute_name, attribute_value)
            # self.get_logger().info(f"setting {attribute_name}: {attribute_value}")
        self.get_logger().info(f"Camera settings successful")

    def stream_camera(self):
        img_cv = self.cam.get_array()
        img_msg = self.bridge.cv2_to_imgmsg(img_cv)
        img_msg.header.frame_id = self.cam_id
        self.pub_stream.publish(img_msg)

    def shutdown_hook(self):
        print("executing shutdown hook")
        self.cam.close()  # You should explicitly clean up


def main(args=None):
    rclpy.init()
    camera_node = SpinnakerCameraNode()
    rclpy.get_default_context().on_shutdown(camera_node.shutdown_hook)
    rclpy.spin(camera_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()