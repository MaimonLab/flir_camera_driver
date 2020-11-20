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
    def __init__(self, cam_identifier=None, preview_image=False):
        # print(cam_identifier)
        super().__init__("pyspin_stream")

        self.declare_parameter("cam_identifier", None)
        self.cam_identifier = self.get_parameter("cam_identifier").value

        print(f"cam id: {self.cam_identifier}")
        self.get_logger().info(f"cam_id: {self.cam_identifier}")

        config_path = Path(
            "/home/maimon/ros2_fictrac_ws/src",
            "flircam_driver/config/example_config.yaml",
        )
        example_config = yaml.load(config_path)

        print(f"Cam identifier: {self.cam_identifier}")
        if self.cam_identifier is None:
            self.cam = Camera()  # Acquire Camera
            self.cam.init()  # Initialize camera
        else:
            self.cam = Camera(self.cam_identifier)  # Acquire Camera
            self.cam.init()  # Initialize camera
            cam_config = example_config[self.cam_identifier]

            for key, item in cam_config.items():
                setattr(self.cam, key, item)

        self.cam_id = self.cam.get_info("DeviceSerialNumber")["value"]
        self.cam_framerate = self.cam.get_info("AcquisitionFrameRate")["value"]

        # for attrib in self.cam.camera_attributes.keys():
        #     try:
        #         print(f"{attrib}, {self.cam.get_info(attrib)['value']}")
        #     except:
        #         pass

        self.publish_timer = self.create_timer(
            1 / self.cam_framerate, self.stream_camera
        )

        self.preview_image = preview_image
        self.cam.start()
        topic_name = f"/camera/id_{self.cam_id}/image_mono"
        self.pub_stream = self.create_publisher(Image, topic_name, 1)
        self.counter = 0
        self.bridge = CvBridge()
        self.get_logger().info("pyspin_streamer has been started")

    def stream_camera(self):
        img_cv = self.cam.get_array()
        # print(img_cv.shape)
        img_msg = self.bridge.cv2_to_imgmsg(img_cv)
        img_msg.header.frame_id = self.cam_id

        self.pub_stream.publish(img_msg)

        img_size = np.array([img_msg.height, img_msg.width])
        img_resized = img_size

        if self.preview_image:
            img_resized = cv2.resize(img_cv, (img_resized[0], img_resized[1]))
            cv2.imshow(f"camera {self.cam_id}", img_resized)
            cv2.waitKey(1)

    def shutdown_hook(self):
        print("executing shutdown hook")

        # self.get_logger().info("executing shutdown hook")
        # self.cam.stop()  # Stop recording
        self.cam.close()  # You should explicitly clean up


def main(args=None):
    rclpy.init()
    camera_node = SpinnakerCameraNode(args)
    rclpy.get_default_context().on_shutdown(camera_node.shutdown_hook)
    rclpy.spin(camera_node)
    rclpy.shutdown()


if __name__ == "__main__":
    if len(sys.argv) == 0:
        main()
    else:
        main(sys.argv[1])