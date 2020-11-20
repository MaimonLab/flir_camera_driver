#!/usr/bin/env python3

import rclpy
import numpy as np
from pathlib import Path
from rclpy.node import Node
from sensor_msgs.msg import Image
from ruamel.yaml import YAML
import cv2
from cv_bridge import CvBridge

yaml = YAML(typ="safe")


class ImagePreviewNode(Node):
    def __init__(self):
        super().__init__("image_previewer")
        config_path = Path(
            "/home/maimon/ros2_fictrac_ws/src",
            "flircam_driver/config/example_config.yaml",
        )
        example_config = yaml.load(config_path)

        cams = list(example_config.keys())
        print(cams)

        for cam_id in cams:
            self.subs = self.create_subscription(
                Image, f"/camera/id_{cam_id}/image_mono", self.image_callback, 1
            )

        self.bridge = CvBridge()

    def image_callback(self, img_msg):

        cam_name = img_msg.header.frame_id
        cv_image = self.bridge.imgmsg_to_cv2(img_msg)

        img_size = np.array([img_msg.height, img_msg.width]).astype(int)

        resize_ratio = 256.0 / max(img_size)
        if max(img_size) > 256:
            img_new_size = (img_size * resize_ratio).astype(int)
            img_resized = cv2.resize(cv_image, (img_new_size[1], img_new_size[0]))
        else:
            img_new_size = img_size
            img_resized = cv_image
        cv2.imshow(f"cam_{cam_name}", img_resized)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    previewer_node = ImagePreviewNode()
    rclpy.spin(previewer_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
