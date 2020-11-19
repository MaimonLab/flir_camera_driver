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
        # all_topics = self.get_topic_names_and_types()
        # all_topics = self.get_topic_names_and_types(no_demangle=True)
        # print(all_topics)

        # camera_topics = []
        # for topic_tuple in all_topics:
        #     print(f"found topic: {topic_tuple[0]}")
        #     if "camera" in topic_tuple[0]:
        #         camera_topics.append(topic_tuple[0])

        # print(f"Camera topics: {camera_topics}")
        config_path = Path(
            "/home/maimon/ros2_template_ws/src",
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

        img_resized = img_size
        # img_resized = cv2.resize(cv_image, (img_resized[0], img_resized[1]))
        im_small = cv2.resize(cv_image, (img_resized[1], img_resized[0]))
        cv2.imshow(f"cam_{cam_name}", im_small)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    previewer_node = ImagePreviewNode()
    rclpy.spin(previewer_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
