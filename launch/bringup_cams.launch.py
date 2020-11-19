from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    camera_1 = Node(
        package="flircam_driver",
        name="camera_1",
        executable="pyspin_stream",
        parameters=[{"cam_identifier": "18518215"}],
    )
    camera_2 = Node(
        package="flircam_driver",
        name="camera_2",
        executable="pyspin_stream",
        parameters=[{"cam_identifier": "18517933"}],
    )
    images_preview = Node(package="flircam_driver", executable="preview_camera_topics")

    ld.add_action(camera_1)
    ld.add_action(camera_2)
    ld.add_action(images_preview)

    return ld