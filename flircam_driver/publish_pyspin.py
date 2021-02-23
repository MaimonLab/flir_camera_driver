#!/usr/bin/env python3

"""Pyspin node without using simple-pyspin

This is still heavily under construction. It turns out using pyspin simple is much .. simpler. 
I still have some issues setting parameters
"""

import rclpy
from rclpy.node import Node
import PySpin
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import Temperature
from cv_bridge import CvBridge
import numpy as np
import time
from ruamel.yaml import YAML

yaml = YAML(typ="safe")


class SpinnakerCameraNode(Node):
    def __init__(self):
        # print(cam_identifier)
        super().__init__(
            "publish_camera",
            automatically_declare_parameters_from_overrides=True,
        )

        # Declare ros-parameters with default values
        default_param = {
            "config_found": False,
            "cam_id": 0,
            "camera_topic": "/camera/camera_x/image_mono",
            "reset_camera_settings": False,
            "publish_latency": True,
        }
        for key, value in default_param.items():
            if not self.has_parameter(key):
                self.declare_parameter(key, value)

        # get all the parameters in the "camera_settings" part of the parameter config file
        parameter_dict = self.get_parameters_by_prefix("camera_settings")
        self.camera_parameters_to_set = {}
        for param_name, param in parameter_dict.items():
            self.camera_parameters_to_set[param_name] = self.get_parameter(
                f"camera_settings.{param_name}"
            ).value

        # config found parameter is just a check that a config_found parameter was set to true.
        # This is a hacky check for the common naming error, where the name of the node and the name in the config file are different
        self.get_logger().info(
            f"Config found: {self.get_parameter('config_found').value}"
        )

        # Get cam_id parameter, we will see if this camera is available.
        # This can either be a string of 8 digits, e.g.:  "18518215" to select by serial,
        # or an integer, to select by index, e.g.: 0
        self.cam_id = self.get_parameter("cam_id").value
        self.get_logger().info(f"cam_id: {self.cam_id}")

        # Method that does all camera setup functions
        self.cam_init()

        # setup latency publisher
        # self.declare_parameter("publish_latency", True)
        self.publish_latency = self.get_parameter("publish_latency").value
        # self.declare_parameter("latency_topic", "camera/rigX/latency")
        latency_topic = self.get_parameter("latency_topic").value
        if self.publish_latency:
            self.pub_latency = self.create_publisher(Temperature, latency_topic, 10)

        # setup image publisher
        self.camera_topic = self.get_parameter("camera_topic").value
        self.pub_stream = self.create_publisher(Image, self.camera_topic, 1)
        self.bridge = CvBridge()
        self.get_logger().info("Node initialized")

    def cam_init(self):

        # Pyspin system setup
        self.system = PySpin.System.GetInstance()
        self.cam_list = self.system.GetCameras()
        num_cameras = self.cam_list.GetSize()
        if num_cameras == 0:
            self.cam_list.Clear()
            self.system.ReleaseInstance()

        # find all cameras and select the camera by Serial or Index
        cam_serial_numbers = []
        for cam in self.cam_list:
            serial_number = cam.TLDevice.DeviceSerialNumber.ToString()
            cam_serial_numbers.append(serial_number)
        if isinstance(self.cam_id, str):
            self.cam = self.cam_list.GetBySerial(self.cam_id)
        else:
            self.cam = self.cam_list.GetByIndex(self.cam_id)
        self.cam_list.Clear()

        # initialize camera, after this the
        self.cam.Init()
        serial_number = self.cam.DeviceID.ToString()
        other_cams = [x for x in cam_serial_numbers if x != serial_number]
        print(f"Cam streaming: {serial_number}, other cams found {other_cams}")

        # Camera Reset
        # The camera will actually turn off and on, resetting all parameters to the default
        if self.get_parameter("reset_camera_settings").value:
            self.cam.DeviceReset.Execute()
            self.cam.DeInit()
            self.get_logger().info("Resetting camera, sleeping for 5 seconds")
            time.sleep(5)

            self.cam_list = self.system.GetCameras()
            if isinstance(self.cam_id, str):
                self.cam = self.cam_list.GetBySerial(self.cam_id)
            else:
                self.cam = self.cam_list.GetByIndex(self.cam_id)
            self.cam.Init()

        self.set_custom_camera_parameters()

        #
        self.offset_nanosec = self.timestamp_latch()

        self.cam.BeginAcquisition()

    def set_custom_camera_parameters(self):
        pass

        # set parameters
        # for node in self.cam.GetNodeMap().GetNodes():
        #     pit = node.GetPrincipalInterfaceType()
        #     name = node.GetName()
        #     self.cam.camera_node_types[name] = self.cam._attr_type_names.get(pit, pit)
        # # if pit == PySpin.intfICommand:
        # #     self.camera_methods[name] = PySpin.CCommandPtr(node)
        # if pit in self.cam._attr_types:
        #     self.cam.camera_attributes[name] = self.cam._attr_types[pit](node)

        # def __setattr__(self, attr, val):
        # for
        # if attr in self.camera_attributes:

        #     prop = self.camera_attributes[attr]
        #     if not PySpin.IsWritable(prop):
        #         raise CameraError("Property '%s' is not currently writable!" % attr)

        #     if hasattr(prop, 'SetValue'):
        #         prop.SetValue(val)
        #     else:
        #         prop.FromString(val)

        # elif attr in self.camera_methods:
        #     raise CameraError("Camera method '%s' is a function -- you can't assign it a value!" % attr)
        # else:
        #     if attr not in self.__dict__ and self.lock and self.initialized:
        #         raise CameraError("Unknown property '%s'." % attr)
        #     else:
        #         super().__setattr__(attr, val)

        # Latch

        # Get all parameters to set
        # parameter_dict = self.get_parameters_by_prefix("camera_settings")
        # cam_dict = {}
        # for param_name, param in parameter_dict.items():
        #     cam_dict[param_name] = self.get_parameter(
        #         f"camera_settings.{param_name}"
        #     ).value

        # # cam_dict["AcquisitionFrameRateEnable"] = True
        # cam_dict["AcquisitionFrameRate"] = 120.0

        # nodemap = self.cam.GetNodeMap()

        # frame_rate_auto_node = PySpin.CEnumerationPtr(
        #     nodemap.GetNode("AcquisitionFrameRateAuto")
        # )

        # node_frame_rate_auto_off = frame_rate_auto_node.GetEntryByName("Off")
        # frame_rate_auto_off = node_frame_rate_auto_off.GetValue()
        # frame_rate_auto_node.SetIntValue(frame_rate_auto_off)

        # enable_rate_mode = PySpin.CBooleanPtr(
        #     nodemap.GetNode("AcquisitionFrameRateEnabled")
        # )
        # if not PySpin.IsAvailable(enable_rate_mode) or not PySpin.IsWritable(
        #     enable_rate_mode
        # ):
        #     print("W:enable_rate_mode not available/writable. Aborting...")
        # try:
        #     enable_rate_mode.SetValue(True)
        # except PySpin.SpinnakerException as ex:
        #     print("E:Could not enable frame rate: {0}".format(ex))

        # # old settings
        # for key, value in cam_dict.items():
        #     print(key, value)
        #     self.get_logger().info(f"Key: {key}, value: {value}")

        #     attribute = getattr(self.cam, key)
        #     self.get_logger().info(
        #         f"attribute: {key}, getvalue: {attribute.GetValue()}"
        #     )
        #     node_acquisition_mode = PySpin.CEnumerationPtr(nodemap.GetNode(key))

        #     if not PySpin.IsAvailable(node_acquisition_mode) or not PySpin.IsWritable(
        #         node_acquisition_mode
        #     ):
        #         print(f"Node map key: {key}")

        #     # node_acquisition_mode = PySpin.CEnumerationPtr(nodemap.GetNode("AcquisitionMode"))
        #     try:
        #         attribute.SetValue(value)
        #     except:
        #         print(f"Could not set {key}")

        #     self.get_logger().info(
        #         f"attribute: {key}, getvalue: {attribute.GetValue()}"
        #     )

    def timestamp_latch(self):
        self.cam.TimestampLatch.Execute()
        time_nanosec = self.get_clock().now().nanoseconds
        timestamp = self.cam.Timestamp.GetValue()
        offset_nanosec = time_nanosec - timestamp

        return offset_nanosec

    def stream_camera(self):

        image_result = self.cam.GetNextImage()

        if image_result.IsIncomplete():
            self.get_logger().warn(
                f"Image incomplete with image status: {image_result.GetImageStatus()}"
            )
            return
        else:
            width = image_result.GetWidth()
            height = image_result.GetHeight()

        # I don't think we need image conversion
        # The pyspin examples do this so I kept it here for reference
        convert_image = False
        if convert_image:
            image_converted = image_result.Convert(
                PySpin.PixelFormat_Mono8, PySpin.HQ_LINEAR
            )
        else:
            image_converted = image_result

        image = image_converted.GetNDArray()
        chunk_data = image_converted.GetChunkData()

        img_msg = self.bridge.cv2_to_imgmsg(image)

        timestamp = chunk_data.GetTimestamp() + self.offset_nanosec
        frame_id = chunk_data.GetFrameID()

        secs = int(timestamp / 1e9)
        nsecs = int(timestamp - secs * 1e9)

        img_msg.header.stamp.sec = secs
        img_msg.header.stamp.nanosec = nsecs
        img_msg.header.frame_id = str(frame_id)
        self.pub_stream.publish(img_msg)

        # Publishing latency ros-parameter is on by default
        if self.publish_latency:
            latency_msg = Temperature()
            latency_msg.header = img_msg.header
            current_timestamp = self.get_clock().now().nanoseconds
            latency = np.float(current_timestamp - timestamp)
            latency_msg.temperature = latency
            self.pub_latency.publish(latency_msg)

    def shutdown_hook(self):
        self.get_logger().info("cleaning up camera")
        try:
            self.cam.EndAcquisition()
            self.cam.DeInit()
        except:
            print("acquisition already ended")

        del self.cam
        self.cam_list.Clear()
        self.system.ReleaseInstance()


def main(args=None):
    rclpy.init()
    camera_node = SpinnakerCameraNode()
    rclpy.get_default_context().on_shutdown(camera_node.shutdown_hook)
    try:
        while True:
            camera_node.stream_camera()
        # rclpy.spin(camera_node)
    finally:
        camera_node.shutdown_hook()
        rclpy.shutdown()


if __name__ == "__main__":
    main()