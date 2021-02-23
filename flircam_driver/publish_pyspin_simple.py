#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from simple_pyspin import Camera
import cv2
from sensor_msgs.msg import Image, Temperature
from cv_bridge import CvBridge
import numpy as np
from ruamel.yaml import YAML
from pathlib import Path
import time

yaml = YAML(typ="safe")


class SpinnakerCameraNode(Node):
    def __init__(self):
        super().__init__(
            "publish_camera",
            automatically_declare_parameters_from_overrides=True,
        )

        # declare default parameters
        default_param = {
            "config_found": False,
            "cam_id": None,
            "camera_topic": "/camera/camera_x/image",
            "reset_camera_settings": False,
            "publish_latency": True,
            "latency_topic": "camera/rigX/latency",
        }
        for key, value in default_param.items():
            if not self.has_parameter(key):
                self.declare_parameter(key, value)

        # config found parameter is just a check that a config_found parameter was set to true.
        # This checks for the common naming error, where the name of the node in the launch file and the name of the node in the config file are different
        self.get_logger().info(
            f"Config found: {self.get_parameter('config_found').value}"
        )

        self.cam_id = self.get_parameter("cam_id").value
        self.get_logger().info(f"cam_id: {self.cam_id}")

        self.camera_topic = self.get_parameter("camera_topic").value

        # Call method that sets camera properties
        self.set_camera_settings()

        self.cam.start()
        self.offset_nanosec = self.latch_timing_offset()

        self.publish_latency = self.get_parameter("publish_latency").value
        latency_topic = self.get_parameter("latency_topic").value
        self.get_logger().info(f"Latency topic for camera: {latency_topic}")

        if self.publish_latency:
            self.pub_latency = self.create_publisher(Temperature, latency_topic, 10)

        # setup image publisher
        self.pub_stream = self.create_publisher(Image, self.camera_topic, 1)
        self.bridge = CvBridge()
        self.get_logger().info("Node initialized")

    def latch_timing_offset(self):
        self.cam.cam.TimestampLatch.Execute()
        time_nanosec = self.get_clock().now().nanoseconds
        timestamp = self.cam.cam.Timestamp.GetValue()
        offset_nanosec = time_nanosec - timestamp
        return offset_nanosec

    def set_camera_settings(self):
        if self.cam_id is None:
            self.cam = Camera()
        else:
            self.cam = Camera(self.cam_id)
        self.cam.init()

        # Camera Reset
        # The camera will actually turn off and on, resetting all parameters to the default
        if self.get_parameter("reset_camera_settings").value:
            self.cam.init()
            self.cam.DeviceReset()
            self.get_logger().info("Resetting camera, sleeping for 5 seconds")
            time.sleep(5)
            if self.cam_id is None:
                self.cam = Camera()
            else:
                self.cam = Camera(self.cam_id)

        self.cam.init()
        self.cam_id = self.cam.get_info("DeviceSerialNumber")["value"]

        # get desired camera settings
        parameter_dict = self.get_parameters_by_prefix("camera_settings")
        cam_dict = {}
        for param_name, param in parameter_dict.items():
            cam_dict[param_name] = self.get_parameter(
                f"camera_settings.{param_name}"
            ).value

        # set camera settings
        for attribute_name, attribute_value in cam_dict.items():
            setattr(self.cam, attribute_name, attribute_value)

        # get chunk settings
        chunk_params = self.get_parameters_by_prefix("camera_chunkdata")
        chunk_dict = {}
        for chunk_paramname, chunkparam in chunk_params.items():
            chunk_paramval = self.get_parameter(
                f"camera_chunkdata.{chunk_paramname}"
            ).value

            chunk_parts = chunk_paramname.split(".")
            chunk_selector = chunk_parts[0]
            sub_dict = {chunk_parts[1]: chunk_paramval}
            if chunk_dict.get(chunk_selector, "") == "":
                chunk_dict[chunk_selector] = sub_dict
            else:
                chunk_dict[chunk_selector] = {**chunk_dict[chunk_selector], **sub_dict}

        # set chunk settings
        for chunkselector, chunkswitches in chunk_dict.items():
            setattr(self.cam, "ChunkSelector", chunkselector)
            for chunkswitch, chunkbool in chunkswitches.items():
                setattr(self.cam, chunkswitch, chunkbool)

        self.get_logger().info(f"Camera settings successful")

    def stream_camera(self):
        img_cv, chunk_data = self.cam.get_array(get_chunk=True)

        img_msg = self.bridge.cv2_to_imgmsg(img_cv)

        timestamp = chunk_data.GetTimestamp() + self.offset_nanosec
        frame_id = chunk_data.GetFrameID()

        secs = int(timestamp / 1e9)
        nsecs = int(timestamp - secs * 1e9)

        img_msg.header.stamp.sec = secs
        img_msg.header.stamp.nanosec = nsecs
        img_msg.header.frame_id = str(frame_id)
        self.pub_stream.publish(img_msg)

        if self.publish_latency:
            latency_msg = Temperature()
            latency_msg.header = img_msg.header
            current_timestamp = self.get_clock().now().nanoseconds
            latency = np.float(current_timestamp - timestamp)
            latency_msg.temperature = latency
            self.pub_latency.publish(latency_msg)

    def shutdown_hook(self):
        print("executing shutdown hook")
        self.get_logger().info("Releasing camera")
        self.cam.close()  # You should explicitly clean up


def main(args=None):
    rclpy.init()
    camera_node = SpinnakerCameraNode()
    rclpy.get_default_context().on_shutdown(camera_node.shutdown_hook)
    while rclpy.ok():
        camera_node.stream_camera()
    rclpy.shutdown()


if __name__ == "__main__":
    main()