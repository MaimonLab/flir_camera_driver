#!/usr/bin/env python3

""" 
publish_pyspin_simple.py

This node opens a FLIR camera through pyspin and publishes that to the topic specified in the config. 
In the initialization function you can set camera parameters, though this part is fragile. 
The list CAMERA_PRIORITY_SET_ORDER contains an order of parameters and will be used to sort incoming parameters accoridng to this list. 
Any camera setting not in this list will be set after these parameters are set. 

The camera can come with pre-set parameters. 
To ensure you start with a blank camera state, you can pass the parameter reset_camera_settings. 
This will reboot the camera, and turn all settings into the default values. 
This does take ~5 seconds, but especially when testing new parameter configurations I recommend setting it to True. 
-TLM 
"""

import rclpy
from rclpy.node import Node
from simple_pyspin import Camera
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ruamel.yaml import YAML
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import sys
import datetime
import cv2
from contextlib import contextmanager

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

# PARAMETER_FALSE_MEANS_OFF = [
#     "TriggerMode",
#     "GainAuto",
#     "AutoFunctionAOIsControl",
#     "AcquisitionFrameRateAuto",
# ]

BOOLEAN_STRING_DICT = {False: "Off", True: "On"}


class SpinnakerCameraNode(Node):
    def __init__(self):
        super().__init__(
            "publish_camera", automatically_declare_parameters_from_overrides=True
        )

        # declare default parameters
        default_param = {
            "cam_id": None,
            "image_topic": "camera/image",
            "reset_camera_settings": False,
            "latch_timing_interval_s": 5,
            "add_timestamp": False,
        }
        for key, value in default_param.items():
            if not self.has_parameter(key):
                self.declare_parameter(key, value)

        self.add_timestamp = self.get_parameter("add_timestamp").value

        self.cam_id = self.get_parameter("cam_id").value
        self.image_topic = self.get_parameter("image_topic").value

        # Call method that sets camera properties
        self.set_camera_settings()

        try:
            self.cam.start()
        except BaseException:
            self.get_logger().error(f"error: {sys.exc_info()}")
            raise Exception

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

    @contextmanager
    def error_if_unavailable(self):
        try:
            yield
        except:
            if self.cam_id:
                self.get_logger().error(
                    f"Failed to open Camera with serial: {self.cam_id}. It might be opened elsewhere or not plugged in. Closing Node."
                )
            else:
                self.get_logger().error(
                    f"Failed to open any camera: are all cameras already opened, or are none plugged in? Closing Node."
                )
            self.destroy_node()
            exit()

    def set_camera_settings(self):
        """"""

        # parse cam_id
        # Cameras can be initialized by index or by id.
        # If a large int, it most likely means an id, which is of a string format
        # note that leading zeros in cam id of type int would result in bugs that can only be solved before yaml is produced
        if (type(self.cam_id) == int) and (self.cam_id > 10):
            self.cam_id = f"{self.cam_id}"

        with self.error_if_unavailable():
            if self.cam_id is None:
                self.cam = Camera()
            else:
                self.cam = Camera(self.cam_id)
                self.get_logger().info(f"Opening camera by serial: {self.cam_id}")

        # unsure if this one also needs to be in context manager
        with self.error_if_unavailable():
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
            self.get_logger().info("Camera restarted")

        self.cam.init()

        # if no cam_id specified in param, log serial of camera opened
        if self.cam_id is None:
            acquired_cam_id = self.cam.get_info("DeviceSerialNumber")["value"]
            self.get_logger().info(
                f"No serial specified, opened camera has serial: {acquired_cam_id}"
            )

        # get desired camera settings
        parameter_dict = self.get_parameters_by_prefix("camera_settings")
        cam_dict = {}
        for param_name, _ in parameter_dict.items():
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

        ordered_cam_dict = {"AcquisitionFrameRateAuto": False}
        # set camera settings
        for attribute_name, attribute_value in ordered_cam_dict.items():

            try:
                setattr(self.cam, attribute_name, attribute_value)
            except TypeError:
                self.get_logger().info(
                    f"TypeError for {attribute_name}: {attribute_value}, using '{BOOLEAN_STRING_DICT[attribute_value]}'"
                )
                attribute_value = BOOLEAN_STRING_DICT[attribute_value]

                # try with converted type, otherwise throw bigger error
                try:
                    setattr(self.cam, attribute_name, attribute_value)
                except:
                    self.get_logger().warn(
                        f"Error setting [{attribute_name}: {attribute_value}], skipping"
                    )
            except:
                self.get_logger().warn(
                    f"Error setting [{attribute_name}: {attribute_value}], skipping"
                )

        # get chunk settings
        chunk_params = self.get_parameters_by_prefix("camera_chunkdata")
        chunk_dict = {}
        for chunk_paramname, _ in chunk_params.items():
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

    def burn_timestamp(self, img_cv, frame_id, timestamp):
        height = len(img_cv)

        datetime_str = datetime.datetime.fromtimestamp(timestamp / 1e9).strftime(
            "%y/%m/%d %H:%M:%S"
        )

        cv2.rectangle(img_cv, (0, height - 17), (165, height), 0, -1)
        cv2.putText(
            img_cv,
            datetime_str,
            (0, height - 5),
            cv2.FONT_HERSHEY_PLAIN,
            1,
            (255, 255, 255),
            1,
        )
        cv2.rectangle(img_cv, (0, height - 34), (66, height - 17), 0, -1)
        cv2.putText(
            img_cv,
            f"id: {frame_id}",
            (0, height - 22),
            cv2.FONT_HERSHEY_PLAIN,
            1,
            (255, 255, 255),
            1,
        )

    def stream_camera(self):
        img_cv, chunk_data = self.cam.get_array(get_chunk=True)
        timestamp = chunk_data.GetTimestamp() + self.offset_nanosec
        frame_id = chunk_data.GetFrameID()

        if self.add_timestamp:
            self.burn_timestamp(img_cv, frame_id, timestamp)

        img_msg = self.bridge.cv2_to_imgmsg(img_cv)

        secs = int(timestamp / 1e9)
        nsecs = int(timestamp - secs * 1e9)

        img_msg.header.stamp.sec = secs
        img_msg.header.stamp.nanosec = nsecs
        img_msg.header.frame_id = str(frame_id)
        self.pub_stream.publish(img_msg)

        if (time.time() - self.last_latch_time) > self.latch_timer_period:
            self.latch_timing_offset()
            self.last_latch_time = time.time()


def main():
    rclpy.init()
    camera_node = SpinnakerCameraNode()

    try:
        while rclpy.ok():
            camera_node.stream_camera()
    except KeyboardInterrupt:
        pass
    except BaseException:
        camera_node.get_logger().error(f"Exception in camera node: {sys.exc_info()}")


if __name__ == "__main__":
    main()