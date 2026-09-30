import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
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
        if value.lower() == 'false':
            return False
    return value


def load_parameters(context, args):
    default_params = {arg.name: LaunchConfiguration(arg.name).perform(context) for arg in args}
    config_file_path = LaunchConfiguration('config_file_path').perform(context)
    if config_file_path:
        default_params = merge_params(default_params, load_yaml(config_file_path))
    skip_convert = {'config_file_path', 'usb_port', 'serial_number', 'bag_record_filename', 'bag_filename',
                    'depth_colorizer_mode'}

    result = {}
    for key, value in default_params.items():
        if key in skip_convert:
            result[key] = value
        elif 'enable_pub_plugins' in key:
            if isinstance(value, str):
                if value.startswith('[') and value.endswith(']'):
                    try:
                        result[key] = yaml.safe_load(value)
                    except Exception:
                        result[key] = [value]
                else:
                    result[key] = [value]
            elif isinstance(value, list):
                result[key] = value
            else:
                result[key] = [str(value)]
        else:
            result[key] = convert_value(value)
    return result


def generate_launch_description():
    args = [
        DeclareLaunchArgument('device_type', default_value='camera'),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('serial_number', default_value=''),
        DeclareLaunchArgument('usb_port', default_value=''),
        DeclareLaunchArgument('device_num', default_value='1'),
        DeclareLaunchArgument('bag_record_filename', default_value=''),
        DeclareLaunchArgument('bag_filename', default_value=''),
        DeclareLaunchArgument('bag_loop', default_value='false'),
        DeclareLaunchArgument('connection_delay', default_value='10'),
        DeclareLaunchArgument('uvc_backend', default_value='libuvc'),
        DeclareLaunchArgument('load_config_json_file_path', default_value=''),
        DeclareLaunchArgument('export_config_json_file_path', default_value=''),

        DeclareLaunchArgument('color_frame_queue_max_frames', default_value='1'),

        DeclareLaunchArgument('color_qos', default_value='default'),

        DeclareLaunchArgument('color_qos_history', default_value='default'),

        DeclareLaunchArgument('color_qos_depth', default_value='-1'),
        DeclareLaunchArgument('color_camera_info_qos', default_value='default'),

        DeclareLaunchArgument('depth_qos', default_value='default'),

        DeclareLaunchArgument('depth_qos_history', default_value='default'),

        DeclareLaunchArgument('depth_qos_depth', default_value='-1'),
        DeclareLaunchArgument('depth_camera_info_qos', default_value='default'),

        DeclareLaunchArgument('left_ir_qos', default_value='default'),

        DeclareLaunchArgument('left_ir_qos_history', default_value='default'),

        DeclareLaunchArgument('left_ir_qos_depth', default_value='-1'),
        DeclareLaunchArgument('left_ir_camera_info_qos', default_value='default'),

        DeclareLaunchArgument('right_ir_qos', default_value='default'),

        DeclareLaunchArgument('right_ir_qos_history', default_value='default'),

        DeclareLaunchArgument('right_ir_qos_depth', default_value='-1'),
        DeclareLaunchArgument('right_ir_camera_info_qos', default_value='default'),

        DeclareLaunchArgument('enable_sync_output_accel_gyro', default_value='false'),
        DeclareLaunchArgument('linear_accel_cov', default_value='0.01'),
        DeclareLaunchArgument('angular_vel_cov', default_value='0.01'),

        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('tf_publish_rate', default_value='0.0'),
        DeclareLaunchArgument('ir_info_url', default_value=''),
        DeclareLaunchArgument('color_info_url', default_value=''),
        DeclareLaunchArgument('enable_publish_extrinsic', default_value='false'),

        DeclareLaunchArgument('point_cloud_qos', default_value='default'),
        DeclareLaunchArgument('cloud_frame_id', default_value=''),
        DeclareLaunchArgument('ordered_pc', default_value='false'),
        DeclareLaunchArgument('enable_depth_scale', default_value='true'),

        DeclareLaunchArgument('time_domain', default_value='global'),
        DeclareLaunchArgument('enable_frame_drop_log', default_value='false'),
        DeclareLaunchArgument('frame_timestamp_csv_file', default_value=''),
        DeclareLaunchArgument('enable_d2c_viewer', default_value='false'),
        DeclareLaunchArgument('depth_colorizer_mode', default_value='none'),
        DeclareLaunchArgument('show_fps_enable', default_value='false'),

        DeclareLaunchArgument('enumerate_net_device', default_value='true'),
        DeclareLaunchArgument('net_device_ip', default_value=''),
        DeclareLaunchArgument('net_device_port', default_value='0'),
        DeclareLaunchArgument('device_access_mode', default_value='Default'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('log_file_name', default_value=''),
        DeclareLaunchArgument('config_file_path', default_value=''),
        DeclareLaunchArgument('monitor_poll_interval_sec', default_value='-1'),

        DeclareLaunchArgument('force_ip_enable', default_value='false'),
        DeclareLaunchArgument('force_ip_mac', default_value=''),
        DeclareLaunchArgument('force_ip_address', default_value='192.168.1.10'),
        DeclareLaunchArgument('force_ip_subnet_mask', default_value='255.255.255.0'),
        DeclareLaunchArgument('force_ip_gateway', default_value='192.168.1.1'),

        DeclareLaunchArgument(
            'color.image_raw.enable_pub_plugins',
            default_value='["image_transport/compressed", "image_transport/raw", "image_transport/theora"]'),
        DeclareLaunchArgument(
            'depth.image_raw.enable_pub_plugins',
            default_value='["image_transport/compressedDepth", "image_transport/raw"]'),
        DeclareLaunchArgument(
            'left_ir.image_raw.enable_pub_plugins',
            default_value='["image_transport/compressed", "image_transport/raw", "image_transport/theora"]'),
        DeclareLaunchArgument(
            'right_ir.image_raw.enable_pub_plugins',
            default_value='["image_transport/compressed", "image_transport/raw", "image_transport/theora"]'),
    ]

    def get_params(context, args):
        return [load_parameters(context, args)]

    def create_node_action(context, args):
        params = get_params(context, args)
        ros_distro = os.environ.get('ROS_DISTRO', 'humble')
        if ros_distro == 'foxy':
            return [
                Node(
                    package='orbbec_camera',
                    executable='orbbec_camera_node',
                    name='ob_camera_node',
                    namespace=LaunchConfiguration('camera_name'),
                    parameters=params,
                    output='log',
                )
            ]
        return [
            GroupAction([
                PushRosNamespace(LaunchConfiguration('camera_name')),
                ComposableNodeContainer(
                    name='camera_container',
                    namespace='',
                    package='rclcpp_components',
                    executable='component_container',
                    composable_node_descriptions=[
                        ComposableNode(
                            package='orbbec_camera',
                            plugin='orbbec_camera::OBCameraNodeDriver',
                            name=LaunchConfiguration('camera_name'),
                            parameters=params,
                        ),
                    ],
                    output='log',
                )
            ])
        ]

    return LaunchDescription(
        args + [
            OpaqueFunction(function=lambda context: create_node_action(context, args))
        ]
    )
