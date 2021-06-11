# flir_camera_driver

This ROS2 package interfaces with the FLIR spinnaker SDK through Pyspin and the [simple_pyspin](https://github.com/klecknerlab/simple_pyspin) package.

The simple_pyspin package comes with [example scripts](https://klecknerlab.github.io/simple_pyspin/) and a [list of settings for the Chameleon cameras](https://klecknerlab.github.io/simple_pyspin/cameras/Point_Grey_Research_Chameleon3_CM3-U3-13Y3M.html). Note that not all settings are implemented but that they can easily be added to the `CAMERA_PRIORITY_SET_ORDER` in `/flir_camera_driver/publish_pyspin_simple.py`

# Example

The basic example opens a camera publisher and an rqt_image_viewer:

    ros2 launch flir_camera_driver example.launch.py

The name given in the launch file _basic_example_camera_ links the parameters from the `config/example_config.yaml` with the camera node.

# Gotchas

If you have worked with FLIR's Spinview before you might have noticed that some setting options are blocked until another setting is in the right state. An example is that the "Acqusition Frame Rate Auto" needs to be set to "Off" before you can set "Acqusition Frame Rate" to a specific value. Spinnaker also has this issue, and when we run the `publish_pyspin_simple` node and we have a dictionary of settings we want to apply to the camera, we cannot guarrantee that the dictionary entries are applied in the right order to prevent settings to be blocked (and thus skipped).

The solution currently in placed is to have a list of settings named `CAMERA_PRIORITY_SET_ORDER` in which the settings will be applied. If a setting you're trying to apply is not on this list, it will be set after any entry from this list.

# License and reuse

Currently this repository is only accessible for maimon lab members. We plan to release this repository in the future under the LGPLv3 license.
