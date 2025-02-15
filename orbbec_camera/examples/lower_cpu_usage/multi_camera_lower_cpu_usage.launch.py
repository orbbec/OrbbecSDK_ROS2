import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node,LoadComposableNodes
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.descriptions import ComposableNode

def load_yaml(file_path):
    with open(file_path, 'r') as f:
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
        if value.lower() == 'true':
            return True
        elif value.lower() == 'false':
            return False
    return value


def load_parameters(context, args):
    default_params = {arg.name: LaunchConfiguration(arg.name).perform(context) for arg in args}
    config_file_path = LaunchConfiguration('config_file_path').perform(context)
    if config_file_path:
        yaml_params = load_yaml(config_file_path)
        default_params = merge_params(default_params, yaml_params)
    skip_convert = {'config_file_path', 'usb_port', 'serial_number'}
    return {
        key: (value if key in skip_convert else convert_value(value))
        for key, value in default_params.items()
    }
def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory("orbbec_camera")
    launch_file_dir = os.path.join(package_dir, "examples/lower_cpu_usage")

    attach_to_shared_component_container_arg = LaunchConfiguration('attach_to_shared_component_container', default=False)
    component_container_name_arg = LaunchConfiguration('component_container_name', default='shared_orbbec_container')

    shared_orbbec_container = Node(
        name=component_container_name_arg,
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        condition=UnlessCondition(attach_to_shared_component_container_arg),
    )

    attach_to_shared_component_container_arg = TextSubstitution(text="true")

    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_lower_cpu_usage.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_01",
            "usb_port": "2-7",
            "device_num": "4",
            "sync_mode": "standalone",
            "attach_to_shared_component_container": attach_to_shared_component_container_arg,
            "component_container_name": component_container_name_arg,
        }.items(),
    )
    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_lower_cpu_usage.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_02",
            "usb_port": "2-2",
            "device_num": "4",
            "sync_mode": "standalone",
            "attach_to_shared_component_container": attach_to_shared_component_container_arg,
            "component_container_name": component_container_name_arg,
        }.items(),
    )
    launch3_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_lower_cpu_usage.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_03",
            "usb_port": "2-6",
            "device_num": "4",
            "sync_mode": "standalone",
            "attach_to_shared_component_container": attach_to_shared_component_container_arg,
            "component_container_name": component_container_name_arg,
        }.items(),
    )
    launch4_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_lower_cpu_usage.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_04",
            "usb_port": "2-3",
            "device_num": "4",
            "sync_mode": "standalone",
            "attach_to_shared_component_container": attach_to_shared_component_container_arg,
            "component_container_name": component_container_name_arg,
        }.items(),
    )

    delayed_left_camera = TimerAction(
        period=2.0,
        actions=[launch1_include],
    )
    delayed_right_camera = TimerAction(
        period=4.0,
        actions=[launch2_include],
    )
    delayed_rear_camera = TimerAction(
        period=6.0,
        actions=[launch3_include],
    )
    delayed_front_camera = TimerAction(
        period=8.0,
        actions=[launch4_include],
    )
    ld = LaunchDescription(
        [
            shared_orbbec_container,
            TimerAction(period=0.0, actions=[GroupAction([delayed_left_camera])]),
            TimerAction(period=2.0, actions=[GroupAction([delayed_right_camera])]),
            TimerAction(period=4.0, actions=[GroupAction([delayed_rear_camera])]),
            TimerAction(period=6.0, actions=[GroupAction([delayed_front_camera])]),
        ]
    )

    return ld
