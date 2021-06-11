from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    workspace = get_package_share_directory("flir_camera_driver").split("/install")[0]
    config = f"{workspace}/src/flir_camera_driver/config/example_config.yaml"

    flir_camera = Node(
        package="flir_camera_driver",
        executable="publish_camera",
        name="basic_example_camera",
        parameters=[config],
    )
    ld.add_action(flir_camera)

    rqt_viewer = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="image_preview",
        arguments=["/camera/image_mono"],
    )
    ld.add_action(rqt_viewer)

    return ld