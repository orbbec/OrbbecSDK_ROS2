import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, ComposableNodeContainer, Node
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
    args = [
        DeclareLaunchArgument('device_type', default_value='lidar'),
        DeclareLaunchArgument('camera_name', default_value='lidar'),
        DeclareLaunchArgument('device_num', default_value='1'),
        DeclareLaunchArgument('upgrade_firmware', default_value=''),
        DeclareLaunchArgument('connection_delay', default_value='10'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('tf_publish_rate', default_value='0.0'),
        DeclareLaunchArgument('lidar_format', default_value='ANY'),#LIDAR_POINT, LIDAR_SPHERE_POINT, LIDAR_SCAN
        DeclareLaunchArgument('lidar_rate', default_value='20'),
        DeclareLaunchArgument('enable_scan_to_point', default_value='false'),
        DeclareLaunchArgument('repetitive_scan_mode', default_value='-1'),
        DeclareLaunchArgument('filter_level', default_value='-1'),
        DeclareLaunchArgument('vertical_fov', default_value='-1.0'),
        DeclareLaunchArgument('min_angle', default_value='-135.0'),
        DeclareLaunchArgument('max_angle', default_value='135.0'),
        DeclareLaunchArgument('min_range', default_value='0.05'),
        DeclareLaunchArgument('max_range', default_value='30.0'),
        DeclareLaunchArgument('echo_mode', default_value='single channel'),
        DeclareLaunchArgument('point_cloud_qos', default_value='default'),
        DeclareLaunchArgument('enumerate_net_device', default_value='true'),
        DeclareLaunchArgument('net_device_ip', default_value=''),
        DeclareLaunchArgument('net_device_port', default_value='0'),
        DeclareLaunchArgument('log_level', default_value='none'),
        DeclareLaunchArgument('time_domain', default_value='device'),# global, device, system
        DeclareLaunchArgument('config_file_path', default_value=''),
        DeclareLaunchArgument('enable_heartbeat', default_value='false'),
    ]

    def get_params(context, args):
        return [load_parameters(context, args)]

    def create_node_action(context, args):
        params = get_params(context, args)
        ros_distro = os.environ.get("ROS_DISTRO", "humble")
        if ros_distro == "foxy":
            return [
                Node(
                    package="orbbec_camera",
                    executable="orbbec_camera_node",
                    name="ob_camera_node",
                    namespace=LaunchConfiguration("camera_name"),
                    parameters=params,
                    output="screen",
                )
            ]
        else:
            return [
                GroupAction([
                    PushRosNamespace(LaunchConfiguration("camera_name")),
                    ComposableNodeContainer(
                        name="camera_container",
                        namespace="",
                        package="rclcpp_components",
                        executable="component_container",
                        composable_node_descriptions=[
                            ComposableNode(
                                package="orbbec_camera",
                                plugin="orbbec_camera::OBCameraNodeDriver",
                                name=LaunchConfiguration("camera_name"),
                                parameters=params,
                            ),
                        ],
                        # prefix=["xterm -e gdb -ex run --args"],
                        output="screen",
                    )
                ])
            ]

    return LaunchDescription(
        args + [
            OpaqueFunction(function=lambda context: create_node_action(context, args))
        ]
    )
