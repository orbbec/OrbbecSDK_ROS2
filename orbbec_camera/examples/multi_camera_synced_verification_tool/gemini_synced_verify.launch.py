import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import UnlessCondition
from launch_ros.actions import LoadComposableNodes

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
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('depth_registration', default_value='true'),
        DeclareLaunchArgument('serial_number', default_value=''),
        DeclareLaunchArgument('usb_port', default_value=''),
        DeclareLaunchArgument('device_num', default_value='1'),
        DeclareLaunchArgument('uvc_backend', default_value='libuvc'),#libuvc or v4l2
        DeclareLaunchArgument('product_id', default_value=''),
        DeclareLaunchArgument('enable_point_cloud', default_value='true'),
        DeclareLaunchArgument('cloud_frame_id', default_value=''),
        DeclareLaunchArgument('enable_colored_point_cloud', default_value='false'),
        DeclareLaunchArgument('point_cloud_qos', default_value='default'),
        DeclareLaunchArgument('connection_delay', default_value='100'),
        DeclareLaunchArgument('color_width', default_value='0'),
        DeclareLaunchArgument('color_height', default_value='0'),
        DeclareLaunchArgument('color_fps', default_value='0'),
        DeclareLaunchArgument('color_format', default_value='ANY'),
        DeclareLaunchArgument('enable_color', default_value='true'),
        DeclareLaunchArgument('color_flip', default_value='false'),
        DeclareLaunchArgument('color_qos', default_value='default'),
        DeclareLaunchArgument('color_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('enable_color_auto_exposure', default_value='true'),
        DeclareLaunchArgument('color_ae_max_exposure', default_value='-1'),
        DeclareLaunchArgument('color_ae_roi_left', default_value='-1'),
        DeclareLaunchArgument('color_ae_roi_right', default_value='-1'),
        DeclareLaunchArgument('color_ae_roi_top', default_value='-1'),
        DeclareLaunchArgument('color_ae_roi_bottom', default_value='-1'),
        DeclareLaunchArgument('color_exposure', default_value='-1'),
        DeclareLaunchArgument('color_gain', default_value='-1'),
        DeclareLaunchArgument('enable_color_auto_white_balance', default_value='true'),
        DeclareLaunchArgument('color_white_balance', default_value='-1'),
        DeclareLaunchArgument('color_brightness', default_value='-1'),
        DeclareLaunchArgument('depth_width', default_value='0'),
        DeclareLaunchArgument('depth_height', default_value='0'),
        DeclareLaunchArgument('depth_fps', default_value='0'),
        DeclareLaunchArgument('depth_format', default_value='ANY'),
        DeclareLaunchArgument('enable_depth', default_value='true'),
        DeclareLaunchArgument('depth_flip', default_value='false'),
        DeclareLaunchArgument('depth_qos', default_value='default'),
        DeclareLaunchArgument('depth_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('depth_ae_roi_left', default_value='-1'),
        DeclareLaunchArgument('depth_ae_roi_right', default_value='-1'),
        DeclareLaunchArgument('depth_ae_roi_top', default_value='-1'),
        DeclareLaunchArgument('depth_ae_roi_bottom', default_value='-1'),
        DeclareLaunchArgument('ir_width', default_value='0'),
        DeclareLaunchArgument('ir_height', default_value='0'),
        DeclareLaunchArgument('ir_fps', default_value='0'),
        DeclareLaunchArgument('ir_format', default_value='ANY'),
        DeclareLaunchArgument('enable_ir', default_value='true'),
        DeclareLaunchArgument('flip_ir', default_value='false'),
        DeclareLaunchArgument('ir_qos', default_value='default'),
        DeclareLaunchArgument('ir_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('enable_ir_auto_exposure', default_value='true'),
        DeclareLaunchArgument('ir_ae_max_exposure', default_value='-1'),
        DeclareLaunchArgument('ir_exposure', default_value='-1'),
        DeclareLaunchArgument('ir_gain', default_value='-1'),
        DeclareLaunchArgument('ir_brightness', default_value='-1'),
        DeclareLaunchArgument('config_file_path', default_value=''),
        DeclareLaunchArgument('enable_sync_output_accel_gyro', default_value='true'),
        DeclareLaunchArgument('enable_accel', default_value='false'),
        DeclareLaunchArgument('accel_rate', default_value='100hz'),
        DeclareLaunchArgument('accel_range', default_value='4g'),
        DeclareLaunchArgument('enable_gyro', default_value='false'),
        DeclareLaunchArgument('gyro_rate', default_value='100hz'),
        DeclareLaunchArgument('gyro_range', default_value='1000dps'),
        DeclareLaunchArgument('linear_accel_cov', default_value='0.01'),
        DeclareLaunchArgument('angular_vel_cov', default_value='0.01'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('tf_publish_rate', default_value='0.0'),
        DeclareLaunchArgument('ir_info_url', default_value=''),
        DeclareLaunchArgument('color_info_url', default_value=''),
        DeclareLaunchArgument('log_level', default_value='none'),
        DeclareLaunchArgument('enable_publish_extrinsic', default_value='false'),
        DeclareLaunchArgument('enable_d2c_viewer', default_value='false'),
        DeclareLaunchArgument('enable_ldp', default_value='true'),
        DeclareLaunchArgument('enable_decimation_filter', default_value='false'),
        DeclareLaunchArgument('decimation_filter_scale', default_value='-1'),
        # Configure the path for depth filter file, for example: /config/depthfilter/Gemini2_v1.7.json
        DeclareLaunchArgument('depth_filter_config', default_value=''),
        # Depth work mode support is as follows:
        # Unbinned Dense Default
        # Unbinned Sparse Default
        # Binned Sparse Default
        # Obstacle Avoidance
        DeclareLaunchArgument('depth_work_mode', default_value=''),
        DeclareLaunchArgument('sync_mode', default_value='standalone'),
        DeclareLaunchArgument('depth_delay_us', default_value='0'),
        DeclareLaunchArgument('color_delay_us', default_value='0'),
        DeclareLaunchArgument('trigger2image_delay_us', default_value='0'),
        DeclareLaunchArgument('trigger_out_delay_us', default_value='0'),
        DeclareLaunchArgument('trigger_out_enabled', default_value='false'),
        DeclareLaunchArgument('enable_frame_sync', default_value='true'),
        DeclareLaunchArgument('ordered_pc', default_value='false'),
        DeclareLaunchArgument('enable_depth_scale', default_value='true'),
        DeclareLaunchArgument('align_mode', default_value='SW'),
        DeclareLaunchArgument('retry_on_usb3_detection_failure', default_value='false'),
        DeclareLaunchArgument('laser_energy_level', default_value='-1'),
        DeclareLaunchArgument('enable_heartbeat', default_value='false'),
        DeclareLaunchArgument('time_domain', default_value='global'),
        DeclareLaunchArgument('use_intra_process_comms', default_value='false'),
        DeclareLaunchArgument('attach_component_container_enable', default_value='false'),
        DeclareLaunchArgument('attach_component_container_name', default_value='orbbec_container'),
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
            attach_to_shared_component_container_arg = LaunchConfiguration('attach_to_shared_component_container', default=False)
            component_container_name_arg = LaunchConfiguration('component_container_name', default='orbbec_container')

            orbbec_container = Node(
                name=component_container_name_arg,
                package='rclcpp_components',
                executable='component_container_mt',
                output='screen',
                condition=UnlessCondition(attach_to_shared_component_container_arg)
            )
            return [
                orbbec_container,
                LoadComposableNodes(
                  target_container=component_container_name_arg,
                  composable_node_descriptions=[
                    ComposableNode(
                      namespace=LaunchConfiguration("camera_name"),
                      name=LaunchConfiguration("camera_name"),
                      package='orbbec_camera',
                      plugin='orbbec_camera::OBCameraNodeDriver',
                      parameters=params,
                      extra_arguments=[{'use_intra_process_comms': LaunchConfiguration("use_intra_process_comms")}],
                    )
                  ]
                )
            ]

    return LaunchDescription(
        args + [
            OpaqueFunction(function=lambda context: create_node_action(context, args))
        ]
    )
