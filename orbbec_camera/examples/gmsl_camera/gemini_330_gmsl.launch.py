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
        DeclareLaunchArgument('upgrade_firmware', default_value=''),
        DeclareLaunchArgument('preset_firmware_path', default_value=''),
        DeclareLaunchArgument('load_config_json_file_path', default_value=''),
        DeclareLaunchArgument('export_config_json_file_path', default_value=''),
        DeclareLaunchArgument('uvc_backend', default_value='libuvc'),#libuvc or v4l2
        DeclareLaunchArgument('point_cloud_qos', default_value='default'),
        DeclareLaunchArgument('enable_point_cloud', default_value='true'),
        DeclareLaunchArgument('point_cloud_decimation_filter_factor', default_value='1'),
        DeclareLaunchArgument('enable_colored_point_cloud', default_value='false'),
        DeclareLaunchArgument('cloud_frame_id', default_value=''),
        DeclareLaunchArgument('connection_delay', default_value='10'),
        DeclareLaunchArgument('color_width', default_value='0'),
        DeclareLaunchArgument('color_height', default_value='0'),
        DeclareLaunchArgument('color_fps', default_value='0'),
        DeclareLaunchArgument('color_format', default_value='ANY'),
        DeclareLaunchArgument('enable_color', default_value='true'),
        DeclareLaunchArgument('color_qos', default_value='default'),
        DeclareLaunchArgument('color_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('enable_color_auto_exposure_priority', default_value='false'),
        DeclareLaunchArgument('color_rotation', default_value='-1'),#color rotation degree : 0, 90, 180, 270
        DeclareLaunchArgument('color_flip', default_value='false'),
        DeclareLaunchArgument('color_mirror', default_value='false'),
        DeclareLaunchArgument('color_ae_roi_left', default_value='-1'),
        DeclareLaunchArgument('color_ae_roi_right', default_value='-1'),
        DeclareLaunchArgument('color_ae_roi_top', default_value='-1'),
        DeclareLaunchArgument('color_ae_roi_bottom', default_value='-1'),
        DeclareLaunchArgument('color_exposure', default_value='-1'),
        DeclareLaunchArgument('color_gain', default_value='-1'),
        DeclareLaunchArgument('enable_color_auto_white_balance', default_value='true'),
        DeclareLaunchArgument('color_white_balance', default_value='-1'),
        DeclareLaunchArgument('enable_color_auto_exposure', default_value='true'),
        DeclareLaunchArgument('color_ae_max_exposure', default_value='-1'),
        DeclareLaunchArgument('color_brightness', default_value='-1'),
        DeclareLaunchArgument('color_sharpness', default_value='-1'),
        DeclareLaunchArgument('color_gamma', default_value='-1'),
        DeclareLaunchArgument('color_saturation', default_value='-1'),
        DeclareLaunchArgument('color_contrast', default_value='-1'),
        DeclareLaunchArgument('color_hue', default_value='-1'),
        DeclareLaunchArgument('color_backlight_compensation', default_value='-1'),#range: 0 - 6, default: 3
        DeclareLaunchArgument('color_powerline_freq', default_value=''),#disable ,50hz ,60hz ,auto
        DeclareLaunchArgument('enable_color_decimation_filter', default_value='false'),
        DeclareLaunchArgument('color_decimation_filter_scale', default_value='-1'),
        DeclareLaunchArgument('depth_width', default_value='0'),
        DeclareLaunchArgument('depth_height', default_value='0'),
        DeclareLaunchArgument('depth_fps', default_value='0'),
        DeclareLaunchArgument('depth_format', default_value='ANY'),
        DeclareLaunchArgument('enable_depth', default_value='true'),
        DeclareLaunchArgument('depth_qos', default_value='default'),
        DeclareLaunchArgument('depth_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('enable_depth_auto_exposure_priority', default_value='false'),
        DeclareLaunchArgument('depth_precision', default_value=''),
        DeclareLaunchArgument('depth_rotation', default_value='-1'),#depth rotation degree : 0, 90, 180, 270
        DeclareLaunchArgument('depth_flip', default_value='false'),
        DeclareLaunchArgument('depth_mirror', default_value='false'),
        DeclareLaunchArgument('depth_ae_roi_left', default_value='-1'),
        DeclareLaunchArgument('depth_ae_roi_right', default_value='-1'),
        DeclareLaunchArgument('depth_ae_roi_top', default_value='-1'),
        DeclareLaunchArgument('depth_ae_roi_bottom', default_value='-1'),
        DeclareLaunchArgument('mean_intensity_set_point', default_value='-1'),
        DeclareLaunchArgument('left_ir_width', default_value='0'),
        DeclareLaunchArgument('left_ir_height', default_value='0'),
        DeclareLaunchArgument('left_ir_fps', default_value='0'),
        DeclareLaunchArgument('left_ir_format', default_value='ANY'),
        DeclareLaunchArgument('enable_left_ir', default_value='false'),
        DeclareLaunchArgument('left_ir_qos', default_value='default'),
        DeclareLaunchArgument('left_ir_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('left_ir_rotation', default_value='-1'),#left_ir rotation degree : 0, 90, 180, 270
        DeclareLaunchArgument('left_ir_flip', default_value='false'),
        DeclareLaunchArgument('left_ir_mirror', default_value='false'),
        DeclareLaunchArgument('enable_left_ir_sequence_id_filter', default_value='false'),
        DeclareLaunchArgument('left_ir_sequence_id_filter_id', default_value='-1'),
        DeclareLaunchArgument('right_ir_width', default_value='0'),
        DeclareLaunchArgument('right_ir_height', default_value='0'),
        DeclareLaunchArgument('right_ir_fps', default_value='0'),
        DeclareLaunchArgument('right_ir_format', default_value='ANY'),
        DeclareLaunchArgument('enable_right_ir', default_value='false'),
        DeclareLaunchArgument('right_ir_qos', default_value='default'),
        DeclareLaunchArgument('right_ir_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('right_ir_rotation', default_value='-1'),#right_ir rotation degree : 0, 90, 180, 270
        DeclareLaunchArgument('right_ir_flip', default_value='false'),
        DeclareLaunchArgument('right_ir_mirror', default_value='false'),
        DeclareLaunchArgument('enable_right_ir_sequence_id_filter', default_value='false'),
        DeclareLaunchArgument('right_ir_sequence_id_filter_id', default_value='-1'),
        DeclareLaunchArgument('enable_ir_auto_exposure', default_value='true'),
        DeclareLaunchArgument('ir_exposure', default_value='-1'),
        DeclareLaunchArgument('ir_gain', default_value='-1'),
        DeclareLaunchArgument('ir_ae_max_exposure', default_value='-1'),
        DeclareLaunchArgument('ir_brightness', default_value='-1'),
        DeclareLaunchArgument('enable_sync_output_accel_gyro', default_value='false'),
        DeclareLaunchArgument('enable_accel', default_value='false'),
        DeclareLaunchArgument('enable_accel_data_correction', default_value='true'),
        DeclareLaunchArgument('accel_rate', default_value='200hz'),
        DeclareLaunchArgument('accel_range', default_value='4g'),
        DeclareLaunchArgument('enable_gyro', default_value='false'),
        DeclareLaunchArgument('enable_gyro_data_correction', default_value='true'),
        DeclareLaunchArgument('gyro_rate', default_value='200hz'),
        DeclareLaunchArgument('gyro_range', default_value='1000dps'),
        DeclareLaunchArgument('linear_accel_cov', default_value='0.01'),
        DeclareLaunchArgument('angular_vel_cov', default_value='0.01'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('tf_publish_rate', default_value='0.0'),
        DeclareLaunchArgument('ir_info_url', default_value=''),
        DeclareLaunchArgument('color_info_url', default_value=''),
        # Network device settings: default enumerate_net_device is set to true, which will automatically enumerate network devices
        # If you do not want to automatically enumerate network devices,
        # you can set enumerate_net_device to false, net_device_ip to the device's IP address, and net_device_port to the default value of 8090
        DeclareLaunchArgument('enumerate_net_device', default_value='false'),
        DeclareLaunchArgument('net_device_ip', default_value=''),
        DeclareLaunchArgument('net_device_port', default_value='0'),
        DeclareLaunchArgument('exposure_range_mode', default_value='default'),#default, ultimate or regular
        DeclareLaunchArgument('log_level', default_value='none'),
        DeclareLaunchArgument('log_file_name', default_value=''),
        DeclareLaunchArgument('enable_publish_extrinsic', default_value='false'),
        DeclareLaunchArgument('enable_d2c_viewer', default_value='false'),
        DeclareLaunchArgument('disparity_to_depth_mode', default_value='HW'),
        DeclareLaunchArgument('enable_ldp', default_value='true'),
        DeclareLaunchArgument('ldp_power_level', default_value='-1'),
        DeclareLaunchArgument('sync_mode', default_value='standalone'),
        DeclareLaunchArgument('depth_delay_us', default_value='0'),
        DeclareLaunchArgument('color_delay_us', default_value='0'),
        DeclareLaunchArgument('trigger2image_delay_us', default_value='0'),
        DeclareLaunchArgument('trigger_out_delay_us', default_value='0'),
        DeclareLaunchArgument('trigger_out_enabled', default_value='true'),
        DeclareLaunchArgument('frames_per_trigger', default_value='2'),
        DeclareLaunchArgument('software_trigger_period', default_value='33'),  # ms
        DeclareLaunchArgument('enable_ptp_config', default_value='false'),#Only for Gemini 335Le
        DeclareLaunchArgument('enable_frame_sync', default_value='true'),
        DeclareLaunchArgument('ordered_pc', default_value='false'),
        DeclareLaunchArgument('enable_depth_scale', default_value='true'),
        DeclareLaunchArgument('enable_decimation_filter', default_value='false'),
        DeclareLaunchArgument('enable_hdr_merge', default_value='false'),
        DeclareLaunchArgument('enable_sequence_id_filter', default_value='false'),
        DeclareLaunchArgument('enable_threshold_filter', default_value='false'),
        DeclareLaunchArgument('enable_hardware_noise_removal_filter', default_value='false'),
        DeclareLaunchArgument('enable_noise_removal_filter', default_value='true'),
        DeclareLaunchArgument('enable_spatial_filter', default_value='false'),
        DeclareLaunchArgument('enable_temporal_filter', default_value='false'),
        DeclareLaunchArgument('enable_disparity_to_depth', default_value='true'),
        DeclareLaunchArgument('enable_hole_filling_filter', default_value='false'),
        DeclareLaunchArgument('decimation_filter_scale', default_value='-1'),
        DeclareLaunchArgument('sequence_id_filter_id', default_value='-1'),
        DeclareLaunchArgument('threshold_filter_max', default_value='-1'),
        DeclareLaunchArgument('threshold_filter_min', default_value='-1'),
        DeclareLaunchArgument('hardware_noise_removal_filter_threshold', default_value='-1.0'),
        DeclareLaunchArgument('noise_removal_filter_min_diff', default_value='256'),
        DeclareLaunchArgument('noise_removal_filter_max_size', default_value='80'),
        DeclareLaunchArgument('spatial_filter_alpha', default_value='-1.0'),
        DeclareLaunchArgument('spatial_filter_diff_threshold', default_value='-1'),
        DeclareLaunchArgument('spatial_filter_magnitude', default_value='-1'),
        DeclareLaunchArgument('spatial_filter_radius', default_value='-1'),
        DeclareLaunchArgument('temporal_filter_diff_threshold', default_value='-1.0'),
        DeclareLaunchArgument('temporal_filter_weight', default_value='-1.0'),
        DeclareLaunchArgument('hole_filling_filter_mode', default_value=''),
        DeclareLaunchArgument('hdr_merge_exposure_1', default_value='-1'),
        DeclareLaunchArgument('hdr_merge_gain_1', default_value='-1'),
        DeclareLaunchArgument('hdr_merge_exposure_2', default_value='-1'),
        DeclareLaunchArgument('hdr_merge_gain_2', default_value='-1'),
        DeclareLaunchArgument('align_mode', default_value='SW'),
        DeclareLaunchArgument('align_target_stream', default_value='COLOR'),# COLOR or DEPTH
        DeclareLaunchArgument('diagnostic_period', default_value='1.0'),
        DeclareLaunchArgument('enable_laser', default_value='true'),
        DeclareLaunchArgument('depth_precision', default_value=''),
        DeclareLaunchArgument('device_preset', default_value='Default'),
        DeclareLaunchArgument('retry_on_usb3_detection_failure', default_value='false'),
        DeclareLaunchArgument('laser_energy_level', default_value='-1'),
        DeclareLaunchArgument('enable_sync_host_time', default_value='true'),
        DeclareLaunchArgument('time_domain', default_value='global'),# global, device, system
        DeclareLaunchArgument('enable_color_undistortion', default_value='false'),
        DeclareLaunchArgument('config_file_path', default_value=''),
        DeclareLaunchArgument('enable_heartbeat', default_value='false'),
        DeclareLaunchArgument('gmsl_trigger_fps', default_value='3000'),
        DeclareLaunchArgument('enable_gmsl_trigger', default_value='false'),
        DeclareLaunchArgument('disparity_range_mode', default_value='-1'),
        DeclareLaunchArgument('disparity_search_offset', default_value='-1'),
        DeclareLaunchArgument('disparity_offset_config', default_value='false'),
        DeclareLaunchArgument('offset_index0', default_value='-1'),
        DeclareLaunchArgument('offset_index1', default_value='-1'),
        DeclareLaunchArgument('frame_aggregate_mode', default_value='ANY'), # full_frame, color_frame, ANY or disable
        DeclareLaunchArgument('interleave_ae_mode', default_value='laser'), # 'hdr' or 'laser'
        DeclareLaunchArgument('interleave_frame_enable', default_value='false'),
        DeclareLaunchArgument('interleave_skip_enable', default_value='false'),
        DeclareLaunchArgument('interleave_skip_index', default_value='1'), # 0:skip pattern ir  1: skip flood ir

        DeclareLaunchArgument('hdr_index1_laser_control', default_value='1'),#interleave_hdr_param
        DeclareLaunchArgument('hdr_index1_depth_exposure', default_value='1'),
        DeclareLaunchArgument('hdr_index1_depth_gain', default_value='16'),
        DeclareLaunchArgument('hdr_index1_ir_brightness', default_value='30'),
        DeclareLaunchArgument('hdr_index1_ir_ae_max_exposure', default_value='30458'),
        DeclareLaunchArgument('hdr_index0_laser_control', default_value='1'),
        DeclareLaunchArgument('hdr_index0_depth_exposure', default_value='7500'),
        DeclareLaunchArgument('hdr_index0_depth_gain', default_value='16'),
        DeclareLaunchArgument('hdr_index0_ir_brightness', default_value='90'),
        DeclareLaunchArgument('hdr_index0_ir_ae_max_exposure', default_value='30458'),

        DeclareLaunchArgument('laser_index1_laser_control', default_value='0'),#interleave_laser_param
        DeclareLaunchArgument('laser_index1_depth_exposure', default_value='3000'),
        DeclareLaunchArgument('laser_index1_depth_gain', default_value='16'),
        DeclareLaunchArgument('laser_index1_ir_brightness', default_value='60'),
        DeclareLaunchArgument('laser_index1_ir_ae_max_exposure', default_value='17000'),
        DeclareLaunchArgument('laser_index0_laser_control', default_value='1'),
        DeclareLaunchArgument('laser_index0_depth_exposure', default_value='3000'),
        DeclareLaunchArgument('laser_index0_depth_gain', default_value='16'),
        DeclareLaunchArgument('laser_index0_ir_brightness', default_value='60'),
        DeclareLaunchArgument('laser_index0_ir_ae_max_exposure', default_value='30000'),
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
