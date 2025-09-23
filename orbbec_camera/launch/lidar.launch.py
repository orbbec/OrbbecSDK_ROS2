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
        DeclareLaunchArgument(
            'device_type',
            default_value='lidar',
            description='Device type to start: lidar or camera. Set lidar to start LiDAR; set camera to start a camera device.'
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='lidar',
            description='Node namespace/device name. Used as prefix for topics, parameters, and TF.'
        ),
        DeclareLaunchArgument(
            'device_num',
            default_value='1',
            description='Number of devices to start. Required when launching multiple devices.'
        ),
        DeclareLaunchArgument(
            'upgrade_firmware',
            default_value='',
            description='Firmware file path. If set, attempts firmware upgrade on startup; empty means no upgrade.'
        ),
        DeclareLaunchArgument(
            'connection_delay',
            default_value='10',
            description='Reopen delay after hot-plug (milliseconds). Prevents firmware issues caused by immediate reconnect.'
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Whether to publish TF frames. true to enable, false to disable.'
        ),
        DeclareLaunchArgument(
            'tf_publish_rate',
            default_value='0.0',
            description='TF publish frequency in Hz. <= 0 means use default or publish on demand.'
        ),
        DeclareLaunchArgument(
            'lidar_format',
            default_value='ANY',  # LIDAR_POINT, LIDAR_SPHERE_POINT, LIDAR_SCAN
            description='LiDAR output format: LIDAR_POINT, LIDAR_SPHERE_POINT, or LIDAR_SCAN. ANY lets device/config decide.'
        ),
        DeclareLaunchArgument(
            'lidar_rate',
            default_value='20',
            description='LiDAR scan/publish rate in Hz. Supported range depends on device model.'
        ),
        DeclareLaunchArgument(
            'publish_n_pkts',
            default_value='1',
            description='Number of frames to accumulate before publishing. Range: 1-12000. Used for multi-frame data merging.'
        ),
        DeclareLaunchArgument(
            'enable_scan_to_point',
            default_value='false',
            description='Convert LaserScan to PointCloud2 and publish. true to enable, false to disable.'
        ),
        DeclareLaunchArgument(
            'repetitive_scan_mode',
            default_value='-1',
            description='Repetitive scan mode. -1 uses device default; other values depend on device capability.'
        ),
        DeclareLaunchArgument(
            'filter_level',
            default_value='-1',
            description='Filtering level. -1 uses default; non-negative integers increase filtering strength.'
        ),
        DeclareLaunchArgument(
            'vertical_fov',
            default_value='-1.0',
            description='Vertical field of view in degrees. -1 uses device default.'
        ),
        DeclareLaunchArgument(
            'min_angle',
            default_value='-135.0',
            description='Minimum scan angle in degrees (e.g., -135.0).'
        ),
        DeclareLaunchArgument(
            'max_angle',
            default_value='135.0',
            description='Maximum scan angle in degrees (e.g., 135.0).'
        ),
        DeclareLaunchArgument(
            'min_range',
            default_value='0.05',
            description='Minimum measurable range in meters.'
        ),
        DeclareLaunchArgument(
            'max_range',
            default_value='30.0',
            description='Maximum measurable range in meters.'
        ),
        DeclareLaunchArgument(
            'echo_mode',
            default_value='single channel',
            description='Echo mode. Examples: single channel, First Echo, Last Echo. Actual options depend on device.'
        ),
        DeclareLaunchArgument(
            'point_cloud_qos',
            default_value='default',
            description='QoS for PointCloud/LaserScan: SYSTEM_DEFAULT, DEFAULT, PARAMETER_EVENTS, SERVICES_DEFAULT, PARAMETERS, SENSOR_DATA (case-insensitive).'
        ),
        DeclareLaunchArgument(
            'enumerate_net_device',
            default_value='true',
            description='Automatically enumerate network devices. true for auto-discovery; false to use specified IP/port only.'
        ),
        DeclareLaunchArgument(
            'net_device_ip',
            default_value='',
            description='IP address of the network device. Leave empty to rely on enumeration or device defaults.'
        ),
        DeclareLaunchArgument(
            'net_device_port',
            default_value='0',
            description='Port of the network device. 0 uses device/driver defaults.'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='none',
            description='SDK log level: none, debug, info, warn, error, fatal.'
        ),
        DeclareLaunchArgument(
            'time_domain',
            default_value='device',  # global, device, system
            description='Timestamp domain: device (device clock), global (time sync), or system (host clock).'
        ),
        DeclareLaunchArgument(
            'config_file_path',
            default_value='',
            description='Path to a YAML config file. If provided, overrides same-name launch arguments; otherwise defaults apply.'
        ),
        DeclareLaunchArgument(
            'enable_heartbeat',
            default_value='false',
            description='Send heartbeat to device. Enable when hardware logging/online monitoring is needed.'
        ),
        DeclareLaunchArgument(
            'enable_imu',
            default_value='false',
            description='Enable IMU (accelerometer and gyroscope) data publishing.'
        ),
        DeclareLaunchArgument(
            'imu_rate',
            default_value='50hz',
            description='IMU publish rate, e.g., 50hz, 100hz. Actual options depend on device.'
        ),
        DeclareLaunchArgument(
            'accel_range',
            default_value='2g',
            description='Accelerometer range, e.g., 2g, 4g, 8g, 16g. Device-dependent.'
        ),
        DeclareLaunchArgument(
            'gyro_range',
            default_value='1000dps',
            description='Gyroscope range, e.g., 250dps, 500dps, 1000dps, 2000dps. Device-dependent.'
        ),
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
