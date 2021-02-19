#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node

# from simple_pyspin import Camera
import PySpin
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import Temperature
from cv_bridge import CvBridge
import numpy as np
from ruamel.yaml import YAML
from pathlib import Path

import time

yaml = YAML(typ="safe")


class SpinnakerCameraNode(Node):
    def __init__(self):
        # print(cam_identifier)
        super().__init__(
            "publish_camera",
            # allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # declare default parameters
        default_param = {
            "config_found": False,
            "cam_id": 0,
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
        # self.cam.start()
        self.cam_framerate = 30.0

        # latency
        self.declare_parameter("publish_latency", True)
        self.publish_latency = self.get_parameter("publish_latency").value
        self.declare_parameter("latency_topic", "camera/rigX/latency")
        latency_topic = self.get_parameter("latency_topic").value
        if self.publish_latency:
            self.pub_latency = self.create_publisher(Temperature, latency_topic, 10)

        self.publish_timer = self.create_timer(
            1 / self.cam_framerate, self.stream_camera
        )
        # create topic
        topic_name = f"{self.camera_topic_base}/image_mono"
        self.pub_stream = self.create_publisher(Image, topic_name, 1)
        self.bridge = CvBridge()
        self.get_logger().info("Node initialized")

    def cam_init(self):
        self.system = PySpin.System.GetInstance()
        self.cam_list = self.system.GetCameras()
        num_cameras = self.cam_list.GetSize()

        if num_cameras == 0:
            self.cam_list.Clear()
            self.system.ReleaseInstance()

        cam_serial_numbers = []
        for cam in self.cam_list:
            # nodemap_tldevice = cam.GetTLDeviceNodeMap()
            serial_number = cam.TLDevice.DeviceSerialNumber.ToString()
            cam_serial_numbers.append(serial_number)
            # print(f"Device serial found: {serial_number}")

        if isinstance(self.cam_id, str):
            self.cam = self.cam_list.GetBySerial(self.cam_id)
        else:
            self.cam = self.cam_list.GetByIndex(self.cam_id)

        # if self.get_parameter("reset_camera_settings").value:

        #     self.cam.Init()  # Initialize camera
        #     self.cam.DeviceReset()
        #     self.get_logger().info("Resetting camera, sleeping for 5 seconds")
        #     time.sleep(5)

        # if isinstance(self.cam_id, str):
        #     self.cam = self.cam_list.GetBySerial(self.cam_id)
        # else:
        #     self.cam = self.cam_list.GetByIndex(self.cam_id)

        self.cam.Init()
        serial_number = self.cam.DeviceID.ToString()
        other_cams = [x for x in cam_serial_numbers if x != serial_number]

        print(f"Cam streaming: {serial_number}, other cams found {other_cams}")

        # do latch
        self.cam.TimestampLatch.Execute()
        time_nanosec = self.get_clock().now().nanoseconds

        timestamp = self.cam.Timestamp.GetValue()

        self.offset_nanosec = time_nanosec - timestamp

        print(f"Current time    {time_nanosec}")
        print(f"Timestamp latch {timestamp}")
        print(f"Offset:         {self.offset_nanosec}")

        # Get all parameters to set
        parameter_dict = self.get_parameters_by_prefix("camera_settings")
        cam_dict = {}
        for param_name, param in parameter_dict.items():
            cam_dict[param_name] = self.get_parameter(
                f"camera_settings.{param_name}"
            ).value

        cam_dict["AcquisitionFrameRateEnable"] = True
        cam_dict["AcquisitionFrameRate"] = 120.0

        for key, value in cam_dict.items():
            print(key, value)
            # breakpoint()

            # attribute = getattr(self.cam, key)
            # print(f"attribute: {key}, getvalue: {attribute.GetValue()}")
            # self.get_logger().info(
            #     f"attribute: {key}, getvalue: {attribute.GetValue()}"
            # )
            # attribute.GetValue()
            # attribute.SetValue(value)
            # breakpoint()
            # print(f"attribute: {key}, getvalue: {attribute.GetValue()}")
            # self.get_logger().info(
            #     f"attribute: {key}, getvalue: {attribute.GetValue()}"
            # )

        self.cam.BeginAcquisition()

        # self.cam.

        # # cam_list =
        # if self.cam_id is None:
        #     self.cam = Camera()  # Acquire Camera
        # else:
        #     self.cam = Camera(self.cam_id)  # Acquire Camera
        # self.cam.init()  # Initialize camera

        # self.cam_id = self.cam.get_info("DeviceSerialNumber")["value"]
        # self.cam_framerate = self.cam.get_info("AcquisitionFrameRate")["value"]

        # # set camera settings
        # for attribute_name, attribute_value in cam_dict.items():
        #     setattr(self.cam, attribute_name, attribute_value)

        # # get chunk settings
        # chunk_params = self.get_parameters_by_prefix("camera_chunkdata")
        # chunk_dict = {}
        # for chunk_paramname, chunkparam in chunk_params.items():
        #     # self.get_logger().info(f"{chunk_paramname}, {chunkparam}")
        #     chunk_paramval = self.get_parameter(
        #         f"camera_chunkdata.{chunk_paramname}"
        #     ).value

        #     chunk_parts = chunk_paramname.split(".")
        #     chunk_selector = chunk_parts[0]
        #     sub_dict = {chunk_parts[1]: chunk_paramval}
        #     if chunk_dict.get(chunk_selector, "") == "":
        #         chunk_dict[chunk_selector] = sub_dict
        #     else:
        #         chunk_dict[chunk_selector] = {**chunk_dict[chunk_selector], **sub_dict}

        # # set chunk settings
        # for chunkselector, chunkswitches in chunk_dict.items():
        #     setattr(self.cam, "ChunkSelector", chunkselector)
        #     for chunkswitch, chunkbool in chunkswitches.items():
        #         setattr(self.cam, chunkswitch, chunkbool)

        # self.get_logger().info(f"Camera settings successful")

    def stream_camera(self):

        image_result = self.cam.GetNextImage()

        if image_result.IsIncomplete():
            print(
                "Image incomplete with image status %d ... \n"
                % image_result.GetImageStatus()
            )
        else:
            # Print image information
            width = image_result.GetWidth()
            height = image_result.GetHeight()
            # print(f"Camera grabbed image , width = {width}, height = {height}")

            # Convert image to mono 8
            # image_converted = image_result.Convert(
            #     PySpin.PixelFormat_Mono8, PySpin.HQ_LINEAR
            # )
            image_converted = image_result

            image = image_converted.GetNDArray()
            # print(f"Image shape: {image.shape}")
            chunk_data = image_converted.GetChunkData()

            cv2.imshow("image", image)
            cv2.waitKey(1)

            img_msg = self.bridge.cv2_to_imgmsg(image)

            timestamp = chunk_data.GetTimestamp() + self.offset_nanosec
            frame_id = chunk_data.GetFrameID()
            print(frame_id)

            secs = int(timestamp / 1e9)
            nsecs = int(timestamp - secs * 1e9)

            img_msg.header.stamp.sec = secs
            img_msg.header.stamp.nanosec = nsecs
            img_msg.header.frame_id = str(frame_id)
            self.pub_stream.publish(img_msg)

        if self.publish_latency:
            latency_msg = Temperature()
            latency_msg.header = img_msg.header

            # msg_timestamp = (
            #     img_msg.header.stamp.sec * 1e9 + img_msg.header.stamp.nanosec
            # )
            current_timestamp = self.get_clock().now().nanoseconds
            latency = np.float(current_timestamp - timestamp)

            latency_msg.temperature = latency
            self.pub_latency.publish(latency_msg)

    def shutdown_hook(self):
        print("executing shutdown hook")
        # self.cam.close()  # You should explicitly clean up
        cv2.destroyAllWindows()
        self.cam.DeInit()

        self.cam_list.Clear()
        self.system.ReleaseInstance()


def main(args=None):
    rclpy.init()
    camera_node = SpinnakerCameraNode()
    rclpy.get_default_context().on_shutdown(camera_node.shutdown_hook)
    rclpy.spin(camera_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()