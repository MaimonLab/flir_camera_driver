# flir_camera_driver

This ROS2 package publishes images from FLIR cameras to an image topic. You can set general node settings (e.g. image_topic) and specific camera settings (e.g. AcquisitionFrameRate) by supplying a config file during launch of the node.

Content:

- [Example](#example)
- [Installation](#installation)
- [Gotchas](#gotchas)
- [Release Notes](#release)

<a name=example></a>

## Example

The basic example opens a camera publisher and an rqt_image_viewer:

    ros2 launch flir_camera_driver example.launch.py

In this exmaple the parameters from `/config/example_config.yaml` under the header `basic_example_config` are passed to the camera node.

<a name=installation></a>

## Installation

Use the installation script:

    ./install_description.sh

This will install all dependencies and call the spinnaker install script. You will be prompted for options, most importantly:

- Do you want to add a user to the **udev permission**? Here you want to say [yes], and add the username (`maimon` in my case). If you do not add this setting, your PointGrey USB cameras will not be detected in spinview and the spinnaker SDK.
- You need an increased [usbfs](https://www.flir.com/support-center/iis/machine-vision/application-note/using-linux-with-usb-3.1/) memory. Ubuntu has it set to 16 Mb/s by default, but FLIR recommends setting this to 1000 Mb.

Instead of the script, you can follow the [install description](./install_description.md)

## Gotchas

<a name=gotchas></a>

- **PARAMETER ORDER OF APPLICATION** If you have worked with FLIR's Spinview before you might have noticed that some setting options are blocked until another setting is in the right state. An example is that the "Acquisition Frame Rate Auto" needs to be set to "Off" before you can set "Acquisition Frame Rate" to a specific value. Spinnaker also has this issue, and when we run the `publish_camera` node and we have a dictionary of settings we want to apply to the camera, we cannot guarantee that the dictionary entries are applied in the right order to prevent settings to be blocked. This happens frequently, and by default a blocked setting will be skipped while logging a warning.

  **Hacky solution:** To circumvent this unknown settings order the publish_camera uses a list of settings named `CAMERA_PRIORITY_SET_ORDER`. The node will first go through this list and set parameters according to this order, before setting parameters not in this list. The `CAMERA_PRIORITY_SET_ORDER` is by no means exhaustive, it merely reflects parameters frequently used in the Maimon lab.

- **YAML TYPE CONVERSION** Yaml has the unfortunate property of interpreting Off without quotes as False. Since we need to set the parameter as the string "Off", this will lead to an error if unhandled.
  **Hacky Solution:** In case the setter experiences a type error, it will look up in the BOOLEAN_STRING_DICT to turn True to "On" and False to "Off" and retry.

**What parameters are available?** The simple_pyspin package comes with [example scripts](https://klecknerlab.github.io/simple_pyspin/) and a [list of settings for the Chameleon cameras](https://klecknerlab.github.io/simple_pyspin/cameras/Point_Grey_Research_Chameleon3_CM3-U3-13Y3M.html).

## License and reuse

Currently this repository is only accessible for maimon lab members. We plan to release this repository in the future under the LGPLv3 license.

<a name=release></a>

## Release Notes

**0.1.1** Update readme to include installation instructions

**0.1.0** Tested on Chameleon CM3-U3-13Y3M
