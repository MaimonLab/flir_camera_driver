# flir_camera_driver

This ROS2 package publishes images from FLIR cameras to an image topic. You can set general node settings (e.g. image_topic) and specific camera settings (e.g. AcquisitionFrameRate) by supplying a config file during launch of the node.

Content:

- [Example](#example)
- [Installation](#installation)
- [Gotchas](#gotchas)
- [Release Notes](#release)

<a name=example></a>

# Example

The basic example opens a camera publisher and an rqt_image_viewer:

    ros2 launch flir_camera_driver example.launch.py

In this exmaple the parameters from `/config/example_config.yaml` under the header `basic_example_config` are passed to the camera node.

<a name=installation></a>

# Installation

Use the installation script:

    ./install_description.sh -get_from=YOUR_MAIMONDATA_MNT_LOCATION/lab_resources/software

This will install all dependencies, including the Spinnaker SDK (v3.2.0.62) via its install script.
If a FLIR Chameleon camera is connected, it will also attempt to update the firmware (v1.24.2-01).
Firmware and SDK install scripts are stored on the lab's data server.
Use the `-get_from` flag to specify the mount location of the data server (`/mnt/maimondata01/` by default).


You may be prompted for options, most importantly:

- Do you want to add a user to the **udev permission**? Here you want to say [yes], and add the username (`maimon` in my case). If you do not add this setting, your PointGrey USB cameras will not be detected in spinview and the spinnaker SDK.
- You need an increased [usbfs](https://www.flir.com/support-center/iis/machine-vision/application-note/using-linux-with-usb-3.1/) memory. Ubuntu has it set to 16 Mb/s by default, but FLIR recommends setting this to 1000 Mb.

Instead of the script, you can follow the [install description](./install_description.md)

# Gotchas

<a name=gotchas></a>

- **PARAMETER ORDER OF APPLICATION:** If you have worked with FLIR's Spinview before you might have noticed that some setting options are blocked until another setting is in the right state. An example is that the "AcquisitionFrameRateAuto" needs to be set to "Off" before you can set "AcquisitionFrameRate" to a specific value. Spinnaker also has this issue, and when we run the `publish_camera` node and we have a dictionary of settings we want to apply to the camera, we cannot guarrantee that the dictionary entries are applied in the right order to prevent settings to be blocked. This happens frequently, and by default a blocked setting will be skipped while logging a warning.

  **Hacky solution:** To circumvent this unknown settings order the publish_camera uses a list of settings named `CAMERA_PRIORITY_SET_ORDER`. The node will first go through this list and set parameters according to this order, before setting parameters not in this list. The `CAMERA_PRIORITY_SET_ORDER` is by no means exhaustive, it merely reflects parameters frequently used in the maimon lab.

- **YAML TYPE CONVERSION:** Yaml has the unfortunate property of interpreting Off without quotes as False. Since we need to set the parameter as the string "Off", this will lead to an error if unhandled.
  **Hacky Solution:** In case the setter experiences a type error, it will attempt a type conversion to boolean and try again. 

- **PREMATURE EXIT:** When using `stream_to_disk`, in order for the saved MPEG video file to be decoded and read properly in the future, the container must include a *moov atom* that acts as a sort of table of contents. This is generally added at the end of the file, so exiting an active stream prematurely can result in an incomplete container. This can be remedied by copying the atom from a video with similar structure. This can be achieved using a library called `untrunc`:
  ```bash
  sudo snap install --edge untrunc-anthwlock
  untrunc-anthwlock -s GOOD_VIDEO BROKEN_VIDEO
  ```


**What parameters are available?** The simple_pyspin package comes with [example scripts](https://klecknerlab.github.io/simple_pyspin/) and a [list of settings for the Chameleon cameras](https://klecknerlab.github.io/simple_pyspin/cameras/Point_Grey_Research_Chameleon3_CM3-U3-13Y3M.html).

# License and reuse

Currently this repository is only accessible for maimon lab members. We plan to release this repository in the future under the LGPLv3 license.

<a name=release></a>

# Release Notes

**0.2.0** Update drivers to PySpin and deprecate Simple-PySpin. Update compatibility up to Spinnaker-3.2.0.57. Add direct streaming to disk using an FFMPEG subprocess (instead of going through ROS middleware).

**0.1.1** Update readme to include installation instructions

**0.1.0** Tested on Chameleon CM3-U3-13Y3M
