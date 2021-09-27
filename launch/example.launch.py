from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import ruamel.yaml

ruamel_yaml = ruamel.yaml.YAML(typ="safe")
ruamel_yaml.default_flow_style = False


def generate_launch_description():
    ld = LaunchDescription()

    # get config file directory
    ros_package_path = get_package_share_directory("flir_camera_driver")
    workspace = ros_package_path.split("/install")[0]
    config_file = f"{workspace}/src/flir_camera_driver/config/example_config.yaml"

    # load config file as a dictionary
    with open(config_file, "r") as yaml_file:
        example_config = ruamel_yaml.load(yaml_file)

    # prompt user to select one node out of the config file
    print(f"\nSelect node name")
    option_dict = {}
    default_idx = 0
    for idx, (node_name, _) in enumerate(example_config.items()):
        if idx == default_idx:
            print(f"   {idx}. {node_name} (default)")
        else:
            print(f"   {idx}. {node_name}")
        option_dict[str(idx)] = node_name
    input_str = input("Select config: ").strip()
    print(f"")

    # parse user input, exiting if it cannot be parsed. This part is fragile
    try:
        if input_str == "":
            input_str = str(default_idx)
        config_key = option_dict[input_str]
        cam_config = example_config[config_key]["ros__parameters"]
        print(f"Launching with config_name: '{config_key}'\n ")
    except:
        print(f"Error parsing your input, exiting")
        exit()

    # add nodes with parameters to the launch description
    camera = Node(
        package="flir_camera_driver",
        name="basic_example_camera",
        executable="publish_camera",
        parameters=[cam_config],
    )
    ld.add_action(camera)

    rqt_viewer = Node(
        package="rqt_image_view",
        name="image_preview",
        executable="rqt_image_view",
        arguments=["/camera/image"],
    )
    ld.add_action(rqt_viewer)

    return ld