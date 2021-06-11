#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from simple_pyspin import Camera
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from ruamel.yaml import YAML
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import sys
import datetime
import cv2

yaml = YAML(typ="safe")

CAMERA_PRIORITY_SET_ORDER = [
    "TriggerMode",
    "GainAuto",
    "Gain",
    "AcquisitionFrameRateAuto",
    "AcquisitionFrameRateEnabled",
    "AcquisitionFrameRate",
    "LineSelector",
    "LineMode",
    "LineSource",
    "LineInverter",
    "BinningVertical",
    "ExposureAuto",
    "Exposure",
]


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
            "image_topic": "/camera/default_rig/image",
            "reset_camera_settings": False,
            "latch_timing_interval_s": 60,
            "add_timestamp": False,
        }
        for key, value in default_param.items():
            if not self.has_parameter(key):
                self.declare_parameter(key, value)

        self.add_timestamp = self.get_parameter("add_timestamp").value

        self.cam_id = self.get_parameter("cam_id").value
        self.get_logger().info(f"cam_id: {self.cam_id}")

        self.image_topic = self.get_parameter("image_topic").value

        # Call method that sets camera properties
        self.set_camera_settings()

        try:
            self.cam.start()
        except BaseException:
            self.get_logger().error(f"error: {sys.exc_info()}")
            raise Exception
            # rclpy.shutdown()
            return
        # exit()

        # latch a timestamp to find the offset betweem computer and camera clock
        self.latch_timing_offset()
        # setup parameters to periodically update latch period
        self.last_latch_time = time.time()
        self.latch_timer_period = self.get_parameter("latch_timing_interval_s").value

        # setup image publisher
        self.pub_stream = self.create_publisher(
            Image,
            self.image_topic,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE),
        )
        self.bridge = CvBridge()

    def latch_timing_offset(self):
        self.cam.cam.TimestampLatch.Execute()
        time_nanosec = self.get_clock().now().nanoseconds
        timestamp = self.cam.cam.Timestamp.GetValue()
        self.offset_nanosec = time_nanosec - timestamp

    def set_camera_settings(self):
        if self.cam_id is None:
            self.cam = Camera()
        else:
            # Cameras can be initialized by index or by id.
            # If a large int, it most likely means an id, which is of a string format
            # note that leading zeros in cam id of type int would result in bugs that can only be solved before yaml is produced
            if (type(self.cam_id) == int) and (self.cam_id > 10):
                self.cam_id = f"{self.cam_id}"
            self.cam = Camera(self.cam_id)
        try:
            self.cam.init()
        except:
            self.get_logger().error(
                f"Failed to open Camera: {self.cam_id}, is it already opened, or not plugged in? Closing Node."
            )
            self.destroy_node()
            exit()

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
            self.get_logger().info("Camera restarted")

        self.cam.init()
        self.cam_id = self.cam.get_info("DeviceSerialNumber")["value"]

        # get desired camera settings
        parameter_dict = self.get_parameters_by_prefix("camera_settings")
        cam_dict = {}
        for param_name, param in parameter_dict.items():
            cam_dict[param_name] = self.get_parameter(
                f"camera_settings.{param_name}"
            ).value

        # force camera dictionary in the following order
        # out of order setting of parameters can result in error
        # e.g. AcquisitionFrameRatesetting before it is enabled
        priority_cam_dict = {}
        for item in CAMERA_PRIORITY_SET_ORDER:
            if item in list(cam_dict):
                priority_cam_dict[item] = cam_dict[item]

        unset_cam_dict = {}
        for param_name, param_value in cam_dict.items():
            if param_name not in CAMERA_PRIORITY_SET_ORDER:
                unset_cam_dict[param_name] = param_value

        # merge priority camera dict and unset cam dict
        ordered_cam_dict = {**priority_cam_dict, **unset_cam_dict}

        # set camera settings
        for attribute_name, attribute_value in ordered_cam_dict.items():

            # it would be better to see if attribute should be string, by inspecting expected variable type
            if attribute_name in [
                "TriggerMode",
                "GainAuto",
                "AutoFunctionAOIsControl",
                "AcquisitionFrameRateAuto",
            ]:
                if attribute_value == False:
                    attribute_value = "Off"
                elif attribute_value == True:
                    attribute_value = "On"

            try:
                setattr(self.cam, attribute_name, attribute_value)
            except:
                self.get_logger().warn(
                    f"Error setting {attribute_name}: {attribute_value}, skipping"
                )

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

    def stream_camera(self):
        img_cv, chunk_data = self.cam.get_array(get_chunk=True)

        if self.add_timestamp:
            height = len(img_cv)
            datetime_str = datetime.datetime.now().strftime("%y/%m/%d %H:%M:%S")
            cv2.rectangle(img_cv, (0, height - 35), (320, height), 0, -1)
            cv2.putText(
                img_cv,
                datetime_str,
                (5, height - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2,
            )

        img_msg = self.bridge.cv2_to_imgmsg(img_cv)

        timestamp = chunk_data.GetTimestamp() + self.offset_nanosec
        frame_id = chunk_data.GetFrameID()

        secs = int(timestamp / 1e9)
        nsecs = int(timestamp - secs * 1e9)

        img_msg.header.stamp.sec = secs
        img_msg.header.stamp.nanosec = nsecs
        img_msg.header.frame_id = str(frame_id)
        self.pub_stream.publish(img_msg)

        if (time.time() - self.last_latch_time) > self.latch_timer_period:
            self.latch_timing_offset()
            self.last_latch_time = time.time()

    def shutdown_hook(self):
        print("executing shutdown hook")
        self.get_logger().info("Releasing camera")
        self.cam.close()  # You should explicitly clean up


def main(args=None):
    rclpy.init()
    try:
        camera_node = SpinnakerCameraNode()
        rclpy.get_default_context().on_shutdown(camera_node.shutdown_hook)
    except:
        # camera_node.get_logger().error(f"Exception in camera init {sys.exc_info()}")
        # camera_node.get_logger().info(f"shutting down ")
        rclpy.shutdown()
        return

    try:
        while rclpy.ok():
            camera_node.stream_camera()
    except KeyboardInterrupt:
        pass
    except BaseException:
        camera_node.get_logger().error(f"Exception in camera node: {sys.exc_info()}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()