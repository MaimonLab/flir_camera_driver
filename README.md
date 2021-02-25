# flir_camera_driver 

This ROS2 package interfaces with the FLIR spinnaker SDK through Pyspin and the simple_pyspin package. 

# Example

The `launch/example.launch.py` loads the parameters from `config/example_config.yaml` and opens the corresponding cameras. It also opens two `rqt_image_view` instances to preview the topics each camera publishes on.


# Gotchas

In the config file, the serial number must be **string**. If not, it is interpreted as an index. You can use an integer to index the camera, but that would then be 0,1,2,....n to the number of cameras you have plugged in. 