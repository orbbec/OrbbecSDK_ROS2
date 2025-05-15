import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.descriptions import ComposableNode


def load_yaml(file_path):
    with open(file_path, "r") as f:
        return yaml.safe_load(f)


def merge_params(default_params, yaml_params):
    for key, value in yaml_params.items():
        if key in default_params:
            default_params[key] = value
    return default_params


def convert_value(value):
    if isinstance(value, str):
        try:
            return int(value)
        except ValueError:
            pass
        try:
            return float(value)
        except ValueError:
            pass
        if value.lower() == "true":
            return True
        elif value.lower() == "false":
            return False
    return value


def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory("orbbec_camera")
    launch_file_dir = os.path.join(
        package_dir, "examples/multi_camera_synced_verification_tool"
    )
    config_file_dir = os.path.join(package_dir, "config")
    config_file_path = os.path.join(config_file_dir, "camera_params.yaml")
    secondary_config_file_path = os.path.join(config_file_dir, "camera_secondary_params.yaml")

    attach_to_shared_component_container_arg = LaunchConfiguration(
        "attach_to_shared_component_container", default=False
    )
    component_container_name_arg = LaunchConfiguration(
        "component_container_name", default="shared_orbbec_container"
    )

    shared_orbbec_container = Node(
        name=component_container_name_arg,
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        condition=UnlessCondition(attach_to_shared_component_container_arg),
    )

    save_rgbir = LoadComposableNodes(
        target_container=component_container_name_arg,
        composable_node_descriptions=[
            ComposableNode(
                namespace="save_rgbir",
                name="save_rgbir",
                package="orbbec_camera",
                plugin="orbbec_camera::tools::MultiCameraSubscriber",
            )
        ],
    )

    attach_to_shared_component_container_arg = TextSubstitution(text="true")

    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_synced_verify.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_01",
            "usb_port": "2-1",
            "device_num": "2",
            "sync_mode": "primary",
            "config_file_path": config_file_path,
            "trigger_out_enabled": "true",
            "attach_to_shared_component_container": attach_to_shared_component_container_arg,
            "component_container_name": component_container_name_arg,
        }.items(),
    )

    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_synced_verify.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_02",
            "usb_port": "2-3",
            "device_num": "2",
            "sync_mode": "secondary_synced",
            "config_file_path": secondary_config_file_path,
            "trigger_out_enabled": "false",
            "attach_to_shared_component_container": attach_to_shared_component_container_arg,
            "component_container_name": component_container_name_arg,
        }.items(),
    )

    # Launch description
    ld = LaunchDescription(
        [
            shared_orbbec_container,
            TimerAction(period=0.0, actions=[GroupAction([launch2_include])]),
            TimerAction(period=2.0, actions=[GroupAction([launch1_include])]),
            # The primary camera should be launched at last
            TimerAction(period=6.0, actions=[GroupAction([save_rgbir])]),
            # save_rgbir is synced verification tool
        ]
    )

    return ld
