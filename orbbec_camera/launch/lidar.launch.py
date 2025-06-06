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
        DeclareLaunchArgument('lidar_format', default_value='ANY'),
        DeclareLaunchArgument('scan_rate', default_value='0'),
        DeclareLaunchArgument('echo_mode', default_value='single channel'),
        DeclareLaunchArgument('point_cloud_qos', default_value='default'),
        # Network device settings: default enumerate_net_device is set to true, which will automatically enumerate network devices
        # If you do not want to automatically enumerate network devices,
        # you can set enumerate_net_device to true, net_device_ip to the device's IP address, and net_device_port to the default value of 8090
        DeclareLaunchArgument('enumerate_net_device', default_value='true'),
        DeclareLaunchArgument('net_device_ip', default_value=''),
        DeclareLaunchArgument('net_device_port', default_value='0'),
        DeclareLaunchArgument('log_level', default_value='none'),
        DeclareLaunchArgument('time_domain', default_value='global'),# global, device, system
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
                        output="screen",
                    )
                ])
            ]

    return LaunchDescription(
        args + [
            OpaqueFunction(function=lambda context: create_node_action(context, args))
        ]
    )
