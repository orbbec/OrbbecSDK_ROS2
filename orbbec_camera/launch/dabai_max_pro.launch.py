from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Declare arguments
    args = [
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('depth_registration', default_value='false'),
        DeclareLaunchArgument('serial_number', default_value=''),
        DeclareLaunchArgument('usb_port', default_value=''),
        DeclareLaunchArgument('device_num', default_value='1'),
        DeclareLaunchArgument('bag_record_filename', default_value=''),
        DeclareLaunchArgument('bag_filename', default_value=''),
        DeclareLaunchArgument('bag_loop', default_value='false'),
        DeclareLaunchArgument('vendor_id', default_value='0x2bc5'),
        DeclareLaunchArgument('product_id', default_value=''),
        DeclareLaunchArgument('enable_point_cloud', default_value='true'),
        DeclareLaunchArgument('point_cloud_decimation_filter_factor', default_value='1'),
        DeclareLaunchArgument('enable_colored_point_cloud', default_value='false'),
        DeclareLaunchArgument('cloud_frame_id', default_value=''),
        DeclareLaunchArgument('point_cloud_qos', default_value='default'),
        DeclareLaunchArgument('connection_delay', default_value='100'),
        DeclareLaunchArgument('diagnostic_period', default_value='0.0'),
        DeclareLaunchArgument('color_frame_queue_max_frames', default_value='1'),
        DeclareLaunchArgument('color_width', default_value='640'),
        DeclareLaunchArgument('color_height', default_value='480'),
        DeclareLaunchArgument('color_fps', default_value='25'),
        DeclareLaunchArgument('color_format', default_value='MJPG'),
        DeclareLaunchArgument('enable_color', default_value='true'),
        DeclareLaunchArgument('color_flip', default_value='false'),
        DeclareLaunchArgument('color_mirror', default_value='false'),
        DeclareLaunchArgument('color_rotation', default_value='-1'),
        DeclareLaunchArgument('color_qos', default_value='default'),
        DeclareLaunchArgument('color_qos_history', default_value='default'),
        DeclareLaunchArgument('color_qos_depth', default_value='-1'),
        DeclareLaunchArgument('color_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('enable_color_auto_exposure_priority', default_value='false'),
        DeclareLaunchArgument('enable_color_auto_exposure', default_value='true'),
        DeclareLaunchArgument('color_exposure', default_value='-1'),
        DeclareLaunchArgument('color_gain', default_value='-1'),
        DeclareLaunchArgument('color_brightness', default_value='-1'),
        DeclareLaunchArgument('enable_color_auto_white_balance', default_value='true'),
        DeclareLaunchArgument('color_white_balance', default_value='-1'),
        DeclareLaunchArgument('color_sharpness', default_value='-1'),
        DeclareLaunchArgument('color_gamma', default_value='-1'),
        DeclareLaunchArgument('color_saturation', default_value='-1'),
        DeclareLaunchArgument('color_contrast', default_value='-1'),
        DeclareLaunchArgument('color_backlight_compensation', default_value='-1'),
        DeclareLaunchArgument('color_powerline_freq', default_value=''),
        DeclareLaunchArgument('depth_width', default_value='640'),
        DeclareLaunchArgument('depth_height', default_value='400'),
        DeclareLaunchArgument('depth_fps', default_value='10'),
        DeclareLaunchArgument('depth_format', default_value='Y12'),
        DeclareLaunchArgument('enable_depth', default_value='true'),
        DeclareLaunchArgument('depth_flip', default_value='false'),
        DeclareLaunchArgument('depth_mirror', default_value='false'),
        DeclareLaunchArgument('depth_rotation', default_value='-1'),
        DeclareLaunchArgument('depth_exposure', default_value='-1'),
        DeclareLaunchArgument('depth_gain', default_value='-1'),
        DeclareLaunchArgument('depth_qos', default_value='default'),
        DeclareLaunchArgument('depth_qos_history', default_value='default'),
        DeclareLaunchArgument('depth_qos_depth', default_value='-1'),
        DeclareLaunchArgument('depth_camera_info_qos', default_value='default'),
        # /config/depthfilter/Openni_device.json，need config path.
        DeclareLaunchArgument('depth_filter_config', default_value=''),
        DeclareLaunchArgument('ir_width', default_value='640'),
        DeclareLaunchArgument('ir_height', default_value='400'),
        DeclareLaunchArgument('ir_fps', default_value='10'),
        DeclareLaunchArgument('ir_format', default_value='Y10'),
        DeclareLaunchArgument('enable_ir', default_value='true'),
        DeclareLaunchArgument('ir_flip', default_value='false'),
        DeclareLaunchArgument('ir_qos', default_value='default'),
        DeclareLaunchArgument('ir_qos_history', default_value='default'),
        DeclareLaunchArgument('ir_qos_depth', default_value='-1'),
        DeclareLaunchArgument('ir_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('enable_ir_auto_exposure', default_value='true'),
        DeclareLaunchArgument('ir_exposure', default_value='-1'),
        DeclareLaunchArgument('ir_gain', default_value='-1'),
        DeclareLaunchArgument('enable_ir_long_exposure', default_value='false'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('tf_publish_rate', default_value='0.0'),
        DeclareLaunchArgument('ir_info_url', default_value=''),
        DeclareLaunchArgument('color_info_url', default_value=''),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('log_file_name', default_value=''),
        DeclareLaunchArgument('enable_publish_extrinsic', default_value='false'),
        DeclareLaunchArgument('enable_d2c_viewer', default_value='false'),
        DeclareLaunchArgument('depth_colorizer_mode', default_value='none'),
        DeclareLaunchArgument('enable_noise_removal_filter', default_value='true'),
        DeclareLaunchArgument('noise_removal_filter_min_diff', default_value='256'),
        DeclareLaunchArgument('noise_removal_filter_max_size', default_value='80'),
        DeclareLaunchArgument('enable_mgc_noise_removal_filter', default_value='false'),
        DeclareLaunchArgument('enable_lut_noise_removal_filter', default_value='false'),
        DeclareLaunchArgument('enable_disp_outliers_filter', default_value='false'),
        DeclareLaunchArgument('disp_outliers_filter_search_mode', default_value=''),  # FULL or OFFSET_80
        DeclareLaunchArgument('enable_edge_noise_removal_filter', default_value='false'),
        DeclareLaunchArgument('enable_spatial_fast_filter', default_value='false'),
        DeclareLaunchArgument('spatial_fast_filter_radius', default_value='-1'),
        DeclareLaunchArgument('enable_spatial_moderate_filter', default_value='false'),
        DeclareLaunchArgument('spatial_moderate_filter_diff_threshold', default_value='-1'),
        DeclareLaunchArgument('spatial_moderate_filter_magnitude', default_value='-1'),
        DeclareLaunchArgument('spatial_moderate_filter_radius', default_value='-1'),
        DeclareLaunchArgument('enable_spatial_filter', default_value='false'),
        DeclareLaunchArgument('spatial_filter_alpha', default_value='-1.0'),
        DeclareLaunchArgument('spatial_filter_diff_threshold', default_value='-1'),
        DeclareLaunchArgument('spatial_filter_magnitude', default_value='-1'),
        DeclareLaunchArgument('spatial_filter_radius', default_value='-1'),
        DeclareLaunchArgument('enable_temporal_filter', default_value='false'),
        DeclareLaunchArgument('temporal_filter_diff_threshold', default_value='-1.0'),
        DeclareLaunchArgument('temporal_filter_weight', default_value='-1.0'),
        DeclareLaunchArgument('enable_hole_filling_filter', default_value='false'),
        DeclareLaunchArgument('hole_filling_filter_mode', default_value=''),
        DeclareLaunchArgument('enable_threshold_filter', default_value='true'),
        DeclareLaunchArgument('threshold_filter_max', default_value='-1'),
        DeclareLaunchArgument('threshold_filter_min', default_value='-1'),
        DeclareLaunchArgument('use_hardware_time', default_value='false'),
        DeclareLaunchArgument('timestamp_clock_type', default_value=''),  # realtime or monotonic, default is realtime.
        DeclareLaunchArgument('enable_frame_drop_log', default_value='false'),
        DeclareLaunchArgument('frame_timestamp_csv_file', default_value=''),
        DeclareLaunchArgument('enable_color_undistortion', default_value='false'),
        DeclareLaunchArgument('enable_depth_undistortion', default_value='false'),
        DeclareLaunchArgument('enable_ir_undistortion', default_value='false'),
        DeclareLaunchArgument('align_mode', default_value='HW'),
        DeclareLaunchArgument('laser_energy_level', default_value='-1'),
        DeclareLaunchArgument('enable_ldp', default_value='true'),
        DeclareLaunchArgument('enable_heartbeat', default_value='false'),
        DeclareLaunchArgument('enable_firmware_log', default_value='false'),
        DeclareLaunchArgument('monitor_poll_interval_sec', default_value='-1'),
    ]

    # Node configuration
    parameters = [{arg.name: LaunchConfiguration(arg.name)} for arg in args]
    # get  ROS_DISTRO
    ros_distro = os.environ["ROS_DISTRO"]
    if ros_distro == "foxy":
        return LaunchDescription(
            args
            + [
                Node(
                    package="orbbec_camera",
                    executable="orbbec_camera_node",
                    name="ob_camera_node",
                    namespace=LaunchConfiguration("camera_name"),
                    parameters=parameters,
                    output="log",
                )
            ]
        )
    # Define the ComposableNode
    else:
        # Define the ComposableNode
        compose_node = ComposableNode(
            package="orbbec_camera",
            plugin="orbbec_camera::OBCameraNodeDriver",
            name=LaunchConfiguration("camera_name"),
            namespace="",
            parameters=parameters,
        )
        # Define the ComposableNodeContainer
        container = ComposableNodeContainer(
            name="camera_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                compose_node,
            ],
            output="log",
        )
        # Launch description
        ld = LaunchDescription(
            args
            + [
                GroupAction(
                    [PushRosNamespace(LaunchConfiguration("camera_name")), container]
                )
            ]
        )
        return ld
