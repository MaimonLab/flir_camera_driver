from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

import ruamel.yaml

ruamel_yaml = ruamel.yaml.YAML(typ="safe")
ruamel_yaml.default_flow_style = False


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory("flir_camera_driver"),
        "config",
        "example_config.yaml",
    )

    with open(config, "r") as yaml_file:
        standard_config = ruamel_yaml.load(yaml_file)

    cam_config = standard_config["camera_default"]["ros__parameters"]

    camera1 = Node(
        package="flir_camera_driver",
        name="camera_default",
        executable="publish_camera",
        parameters=[cam_config],
    )
    ld.add_action(camera1)

    rqt_viewer = Node(
        package="rqt_image_view",
        name="image_preview",
        executable="rqt_image_view",
        arguments=["/camera/image_mono"],
    )
    ld.add_action(rqt_viewer)

    return ld