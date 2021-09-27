#!/usr/bin/env python3

""" 
publish_camera.py

This node opens a FLIR camera through pyspin and publishes that to the topic specified in the config. 
In the initialization function you can set camera parameters, though this part is fragile. 
The list CAMERA_PRIORITY_SET_ORDER contains an order of parameters and will be used to sort incoming parameters accoridng to this list. 
Any camera setting not in this list will be set after these parameters are set. 

The camera can come with pre-set parameters. 
To ensure you start with a blank camera state, you can pass the parameter reset_camera_settings. 
This will reboot the camera, and turn all settings into the default values. 
This does take ~5 seconds, but especially when testing new parameter configurations I recommend setting it to True. 

Parameters:
- cam_id: unique camera identifier. Find by e.g. opening up spinview
- image_topic: Images will be published to this topic 
- reset_camera_settings: At launching the node, this option can reset the camera. This takes roughly 5 seconds. 
- latch_timing_interval_s: Redo the latching every x seconds to ensure image timestamps do not drift 
- add_timestamp: Option to burn a human-readable timestamp on the bottom left of the image
- camera_settings.xxx: camera settings will be a dictionary of parameters that need to be set to the camera (see example_config)
- camera_chunkdata.xxx camera_chunkdata is a dictionary of dictionaries, see example_config 

for a list of parameters that can be set on a FLIR chameleon3: 
https://klecknerlab.github.io/simple_pyspin/cameras/Point_Grey_Research_Chameleon3_CM3-U3-13Y3M.html

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

        # flag to burn timestamp in bottom left of image
        self.add_timestamp = self.get_parameter("add_timestamp").value

        # Call method that sets camera properties
        self.set_camera_settings()

        # start acquiring images
        self.cam.start()

        # latch a timestamp to find the offset betweem computer and camera clock
        self.latch_timing_offset()

        # setup parameters to periodically update latch period
        self.last_latch_time = time.time()
        self.latch_timer_period = self.get_parameter("latch_timing_interval_s").value

        # setup image publisher
        image_topic = self.get_parameter("image_topic").value
        self.pub_stream = self.create_publisher(
            Image,
            image_topic,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE),
        )

        # bridge translates Image messages to cv2 images and vice versa
        self.bridge = CvBridge()

    def latch_timing_offset(self):
        """Obtain offset between camera and computer clocks
        This offset will be added to timestamp that comes with image
        More on latching:
            https://www.flir.com/support-center/iis/machine-vision/application-note/synchronizing-a-blackfly-or-grasshopper3-gige-cameras-time-to-pc-time/
        """

        # call camera to store a timestamp
        self.cam.cam.TimestampLatch.Execute()
        # get computer to call a timestamp
        time_nanosec = self.get_clock().now().nanoseconds

        # request timestamp it stored from the camera
        timestamp = self.cam.cam.Timestamp.GetValue()

        # compute offset
        self.offset_nanosec = time_nanosec - timestamp

    @contextmanager
    def error_if_unavailable(self):
        """Log error and kill node if unable to open a camera"""
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
        """Initialize camera object and apply camera_settings (e.g. AcquisitionFrameRate). """

        # parse cam_id, Cameras can be initialized by index or by id.
        self.cam_id = self.get_parameter("cam_id").value
        if (type(self.cam_id) == int) and (self.cam_id > 10):
            # If a large int, it most likely means an id, which is of a string format
            # note that leading zeros in cam id of type int would result in bugs that can only be solved before yaml is produced
            self.cam_id = f"{self.cam_id}"
        elif type(self.cam_id) == str:
            # You can prepend an id with a dollar sign in the yaml file to force the string type. We'll remove that here.
            self.cam_id = self.cam_id.replace("$", "")

        with self.error_if_unavailable():
            if self.cam_id is None:
                self.cam = Camera()
            else:
                self.cam = Camera(self.cam_id)
                self.get_logger().info(f"Opening camera by serial: {self.cam_id}")

        # Camera Reset, this will turn the camera off and on, resetting all parameters to the default
        if self.get_parameter("reset_camera_settings").value:
            # init to get access to resetting function
            self.cam.init()
            self.cam.DeviceReset()
            self.get_logger().info("Resetting camera, sleeping for 5 seconds")
            time.sleep(5)
            if self.cam_id is None:
                self.cam = Camera()
            else:
                self.cam = Camera(self.cam_id)
            self.get_logger().info("Camera restarted")

        # Initialize camera to gain access to full functionality
        self.cam.init()

        # if no cam_id specified in param, log serial of camera opened
        if self.cam_id is None:
            acquired_cam_id = self.cam.get_info("DeviceSerialNumber")["value"]
            self.get_logger().info(
                f"No serial specified, opened camera has serial: {acquired_cam_id}"
            )

        # camera_settings will be applied to the camera, as opposed to the other parameters which are used to control the node behavior.
        unparsed_cam_dict = self.get_parameters_by_prefix("camera_settings")
        cam_dict = {}
        for param_name, _ in unparsed_cam_dict.items():
            cam_dict[param_name] = self.get_parameter(
                f"camera_settings.{param_name}"
            ).value

        # cam_dict parameters are divided into ones found in the priority set order, and those that are not
        # out of order setting of parameters can result in error
        # e.g. if attempting to set AcquisitionFrameRate setting before AcquisitionFrameRateAuto is turned off, the setting is ignored.
        priority_cam_dict = {}
        for item in CAMERA_PRIORITY_SET_ORDER:
            if item in list(cam_dict):
                priority_cam_dict[item] = cam_dict[item]

        unprioritized_cam_dict = {}
        for param_name, param_value in cam_dict.items():
            if param_name not in CAMERA_PRIORITY_SET_ORDER:
                unprioritized_cam_dict[param_name] = param_value

        # merge priority camera dict and unset cam dict.
        # Prioritized cam dict will be applied first, and is sorted to ensure they are not ignored
        ordered_cam_dict = {**priority_cam_dict, **unprioritized_cam_dict}

        # attempt to apply setting to camera
        # failure to apply setting will log warning, but does not cause node failure
        for attribute_name, attribute_value in ordered_cam_dict.items():
            try:
                setattr(self.cam, attribute_name, attribute_value)
            except TypeError:
                # yaml turns "On" to True, and "Off" to False under certain circumstances
                # The camera does not accept True for "On" and will ignore this setting attempt
                # if we get the type error, we can guess a remapping and retry the setting
                self.get_logger().info(
                    f"TypeError for {attribute_name}: {attribute_value}, using '{BOOLEAN_STRING_DICT[attribute_value]}'"
                )
                attribute_value = BOOLEAN_STRING_DICT[attribute_value]

                # try with converted type, otherwise throw bigger error
                try:
                    setattr(self.cam, attribute_name, attribute_value)
                except:
                    self.get_logger().warn(
                        f"Error setting [{attribute_name}: {attribute_value}] after type conversion! skipping"
                    )
            except:
                self.get_logger().warn(
                    f"Error setting [{attribute_name}: {attribute_value}], skipping"
                )

        # convert chunkdata. Chunkdata is another word for image meta data
        chunk_params = self.get_parameters_by_prefix("camera_chunkdata")

        # build up the nested chunk_dictionary, it will be of the form:
        # chunk_dict = {'timestamp': {'ChunkEnable': True, 'ChunkModeActive': True}}
        chunk_dict = {}
        for chunk_param_name, _ in chunk_params.items():
            chunk_param_value = self.get_parameter(
                f"camera_chunkdata.{chunk_param_name}"
            ).value

            chunk_parts = chunk_param_name.split(".")
            chunk_selector = chunk_parts[0]
            sub_dict = {chunk_parts[1]: chunk_param_value}
            if chunk_dict.get(chunk_selector, "") == "":
                chunk_dict[chunk_selector] = sub_dict
            else:
                chunk_dict[chunk_selector] = {**chunk_dict[chunk_selector], **sub_dict}

        # Apply chunk data settings
        for chunk_name, chunk_switches in chunk_dict.items():
            try:
                setattr(self.cam, "ChunkSelector", chunk_name)
                for chunkswitch, chunkbool in chunk_switches.items():
                    setattr(self.cam, chunkswitch, chunkbool)
            except:
                self.get_logger().warn(
                    f"Error setting chunkdata: {chunk_name}: {chunk_switches}, ignoring settings"
                )

    def burn_timestamp(self, img_cv, frame_id, timestamp):
        """Burn timestamp in bottom left of the image"""

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

    def grab_and_publish_frame(self):
        """Grab image and meta data, convert to image message, and publish to topic"""

        img_cv, chunk_data = self.cam.get_array(get_chunk=True)

        # offset is computed during latching
        timestamp = chunk_data.GetTimestamp() + self.offset_nanosec
        frame_id = chunk_data.GetFrameID()

        if self.add_timestamp:
            self.burn_timestamp(img_cv, frame_id, timestamp)

        # convert image to ros2 Image message type
        # this will initialize an empty header
        img_msg = self.bridge.cv2_to_imgmsg(img_cv)

        # fill in header from camera chunk data
        img_msg.header.stamp = rclpy.time.Time(nanoseconds=timestamp).to_msg()
        img_msg.header.frame_id = str(frame_id)

        self.pub_stream.publish(img_msg)

        # if latching is past it's timer period, redo latching
        if (time.time() - self.last_latch_time) > self.latch_timer_period:
            self.latch_timing_offset()
            self.last_latch_time = time.time()


def main():
    rclpy.init()
    camera_node = SpinnakerCameraNode()

    try:
        while rclpy.ok():
            camera_node.grab_and_publish_frame()
    except KeyboardInterrupt:
        pass
    except BaseException:
        camera_node.get_logger().error(f"Exception in camera node: {sys.exc_info()}")


if __name__ == "__main__":
    main()