from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory("flircam_driver"),
        "config",
        "example_config.yaml",
    )

    camera1 = Node(
        package="flircam_driver",
        name="camera1",
        executable="publish_camera",
        parameters=[config],
    )
    ld.add_action(camera1)

    rqt_viewer = Node(
        package="rqt_image_view",
        name="image_preview",
        executable="rqt_image_view",
        arguments=["/camera/rig1_ball/image_mono"],
    )
    ld.add_action(rqt_viewer)

    return ld