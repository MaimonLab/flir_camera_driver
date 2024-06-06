#!/usr/bin/env python3

""" 
publish_camera.py

This node opens a FLIR camera through pyspin and publishes that to the topic specified in the config. 
In the initialization function you can set camera parameters, though this part is fragile. 
The list CAMERA_PRIORITY_SET_ORDER contains an order of parameters used to sort incoming parameters.
Any camera setting not in this list will be set after the priority parameters are set.

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

import cv2
from cv_bridge import CvBridge
import datetime
import numpy as np
from queue import Queue
from sensor_msgs.msg import Image
import subprocess
import time
import rclpy
from rclpy.time import Time

from flir_camera_driver.ros_pyspin import Camera
from maimon_classes.basic_node import BasicNode


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


class SpinnakerCameraNode(BasicNode):
    def __init__(self):
        super().__init__(
            automatically_declare_parameters_from_overrides=True
        )

        # declare default parameters
        self.default_param = {
            "cam_id": None,
            "image_topic": "camera/image",
            "reset_camera_settings": False,
            "latch_timing_interval_s": 5,
            "flip_y": False,
            "qos_image_publish_reliable": False,
            "disregard_chunkdata": False,
            "burn_timestamp": False,
            "stream_to_disk": False,
            "codec": "mjpeg",
            "encoder_args": [],
            "stream_fr": -1,
            "record_every_nth_frame": 1,
            "output_filename": ""
        }

        self.cam = Camera(self.cam_id)

        # Call method that sets camera properties
        self.set_cam_settings()

        # disregard chunkdata is a flag to deal with an issue where chunkdata are garbled
        self.last_latch_time = time.time()
        # self.offset_nanosec = 0
        if not self.disregard_chunkdata:
            # latch a timestamp to find the offset betweem computer and camera clock
            self.latch_timing_offset()
            self.latch_timer_period = self.latch_timing_interval_s
        else: 
            self.print_warning(f"Disregarding chunkdata, using timestamp at arrival, and counter as frameid")
            self.latch_timer_period = float('infinity')

        # setup image publisher
        self.pub_stream = self.register_publisher(
            msg_type=Image,
            topic=self.image_topic,
            reliable=self.qos_image_publish_reliable
        )

        # set up video streamer if enabled
        if self.stream_to_disk:
            self.buffer = Queue(1)
            self.spawn_thread(self.grab_and_save_frame, daemon=True)

        # bridge translates Image messages to cv2 images and vice versa
        self.bridge = CvBridge()
        self.count_published_images = 0

    def set_cam_settings(self):
        """Apply camera_settings (e.g. AcquisitionFrameRate)."""
        # Camera Reset, this will turn the camera off and on, resetting all parameters to the default
        if self.reset_camera_settings:
            self.cam.reset_settings()
            self.print_warning("Camera restarted")

        # cam_dict parameters are divided into ones found in the priority set order,
        # and those that are not out of order setting of parameters can result in error
        # e.g. if attempting to set AcquisitionFrameRate setting
        # before AcquisitionFrameRateAuto is turned off, the setting is ignored.
        ordered_cam_dict = {}
        for k in CAMERA_PRIORITY_SET_ORDER:
            if k in list(self.camera_settings.keys()):
                ordered_cam_dict[k] = self.camera_settings.pop(k)

        # merge priority camera dict and unset cam dict.
        # Prioritized cam dict will be applied first, and is sorted to ensure they are not ignored
        self.camera_settings = {**ordered_cam_dict, **self.camera_settings}

        # attempt to apply setting to camera
        # failure to apply setting will log warning, but does not cause node failure
        self.cam.set_cam_settings(self.camera_settings)
        self.cam.set_chunk_settings(self.camera_chunkdata)

    def latch_timing_offset(self):
        """Obtain offset between camera and computer clocks
        This offset will be added to timestamp that comes with image
        More on latching:
            https://www.flir.com/support-center/iis/machine-vision/application-note/synchronizing-a-blackfly-or-grasshopper3-gige-cameras-time-to-pc-time/
        """
        # get computer to call a timestamp
        time_nanosec = self.get_clock().now().nanoseconds
        # request timestamp it stored from the camera
        timestamp = self.cam.get_timestamp()
        
        # compute offset
        offset_nanosec = time_nanosec - timestamp
        if hasattr(self, 'offset_nanosec') and (abs(offset_nanosec - self.offset_nanosec) > 5e8):
            self.print_error(
                f"""\n\n
                Offset between computer and camera clock changed by more than 0.5 seconds: 
                {abs(self.offset_nanosec - offset_nanosec)/1e9} seconds. If this happens frequently, 
                you may have a clock issue. If you are synchronizing your clock to a master clock, 
                the system clock may be regularly jumping due to resynchronization 
                with an online network clock other than the PTP grandmaster! 
                Disable Automatic Date & Time in your Date & Time settings, 
                or use a service to make sure that never happens again!!! 
                See https://github.com/maimonlab/ptp_tools for more info -- SCT
                \n"""
            )
        self.offset_nanosec = offset_nanosec

        # set last_latch_time as now
        self.last_latch_time = time.time()

    def add_timestamp(self, img_cv, frame_id, stamp):
        """Burn timestamp in bottom left of the image"""
        height = len(img_cv)
        timestamp = int(stamp.sec * 1e9 + stamp.nanosec)
        timestamp = datetime.datetime.fromtimestamp(timestamp/1e9).strftime("%y/%m/%d %H:%M:%S")

        img_cv = cv2.rectangle(img_cv.copy(), (0, height - 17), (165, height), 0, -1)
        cv2.putText(
            img_cv, timestamp,
            (0, height - 5), cv2.FONT_HERSHEY_PLAIN,
            1,(255, 255, 255), 1,
        )
        cv2.rectangle(img_cv, (0, height - 34), (66, height - 17), 0, -1)
        cv2.putText(
            img_cv,
            f"id: {frame_id}",
            (0, height - 22), cv2.FONT_HERSHEY_PLAIN,
            1, (255, 255, 255), 1,
        )

    def grab_and_save_frame(self):
        cmd = [
            'ffmpeg', '-y',
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-s', f'{self.cam.size[0]}x{self.cam.size[1]}',
            '-r', str(self.stream_fr) if self.stream_fr > 0 else str(self.cam.get_attr('AcquisitionFrameRate')),
            '-pix_fmt', 'gray',
            '-i', '-', '-an',
            '-vcodec', self.codec,
        ]
        if 'nvenc' in self.codec:
            cmd = cmd[:2] + ['-hwaccel', 'cuda', '-hwaccel_output_format', 'cuda'] + cmd[2:]
            if '-rc' not in self.encoder_args:
                cmd.extend(['-rc', 'constqp', '-qp', '18'])
        cmd.extend(self.encoder_args)
        cmd.append(f'{self.output_filename}.mp4')
        self.pipe = subprocess.Popen(
            cmd, stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )

        self.stamps = open(self.output_filename + '_timestamps.csv', 'w')
        self.stamps.write('frame_id,timestamp\n')

        while True:
            img, frame_id, timestamp = self.buffer.get(block=True)
            if img is None:
                break
            self.stamps.write(f'{frame_id},{timestamp}\n')
            self.pipe.stdin.write(img.astype(np.uint8).tobytes())
            self.pipe.stdin.flush()

        return

    def grab_and_publish_frame(self):
        """Grab image and metadata, convert to image message, and publish to topic"""
        self.cam.start()
        while rclpy.ok():

            img_cv, chunk_data = self.cam.get_new_frame(get_chunk=~self.disregard_chunkdata)
            if self.flip_y:
                img_cv = cv2.flip(img_cv, 0)

            # fill in header from camera chunk data
            # disregard chunkdata is a temporary fix for errors in fictrac
            if not self.disregard_chunkdata:
                # offset is computed during latching
                frame_id = str(chunk_data.GetFrameID())
                timestamp = chunk_data.GetTimestamp() + self.offset_nanosec
                stamp = Time(nanoseconds=timestamp).to_msg()
            else:
                frame_id = str(self.count_published_images)
                timestamp = self.get_clock().now().nanoseconds
                stamp = Time(nanoseconds=timestamp).to_msg()

            if self.burn_timestamp:
                self.add_timestamp(img_cv, frame_id, stamp)

            if self.stream_to_disk:
                if self.count_published_images % self.record_every_nth_frame == 0:
                    self.buffer.put((img_cv, frame_id, timestamp))

            # convert image to ros2 Image message type
            # this will initialize an empty header
            img_msg = self.bridge.cv2_to_imgmsg(img_cv)
            img_msg.header.frame_id = frame_id
            img_msg.header.stamp = stamp
            self.pub_stream.publish(img_msg)
            self.count_published_images += 1

            # if latching is past its timer period, redo latching
            if (time.time() - self.last_latch_time) > self.latch_timer_period:
                self.latch_timing_offset()

    def on_destroy(self):
        self.cam.destroy()
        if self.stream_to_disk:
            self.buffer.put((None, None, None))
            with self.buffer.mutex:
                self.buffer.queue.clear()
                self.buffer.all_tasks_done.notify_all()
                self.buffer.unfinished_tasks = 0
            self.buffer.join()
            self.stamps.close()
            self.pipe.stdin.close()
            self.pipe.wait()


def main():
    rclpy.init()
    SpinnakerCameraNode().run('grab_and_publish_frame')


if __name__ == "__main__":
    main()
