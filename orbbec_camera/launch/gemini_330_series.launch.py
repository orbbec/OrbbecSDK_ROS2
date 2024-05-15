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
        DeclareLaunchArgument('depth_registration', default_value='true'),
        DeclareLaunchArgument('serial_number', default_value=''),
        DeclareLaunchArgument('usb_port', default_value=''),
        DeclareLaunchArgument('device_num', default_value='1'),
        DeclareLaunchArgument('point_cloud_qos', default_value='default'),
        DeclareLaunchArgument('enable_point_cloud', default_value='true'),
        DeclareLaunchArgument('enable_colored_point_cloud', default_value='true'),
        DeclareLaunchArgument('connection_delay', default_value='100'),
        DeclareLaunchArgument('color_width', default_value='0'),
        DeclareLaunchArgument('color_height', default_value='0'),
        DeclareLaunchArgument('color_fps', default_value='0'),
        DeclareLaunchArgument('color_format', default_value='ANY'),
        DeclareLaunchArgument('enable_color', default_value='true'),
        DeclareLaunchArgument('color_qos', default_value='default'),
        DeclareLaunchArgument('color_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('enable_color_auto_exposure', default_value='true'),
        DeclareLaunchArgument('depth_width', default_value='0'),
        DeclareLaunchArgument('depth_height', default_value='0'),
        DeclareLaunchArgument('depth_fps', default_value='0'),
        DeclareLaunchArgument('depth_format', default_value='ANY'),
        DeclareLaunchArgument('enable_depth', default_value='true'),
        DeclareLaunchArgument('depth_qos', default_value='default'),
        DeclareLaunchArgument('depth_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('left_ir_width', default_value='0'),
        DeclareLaunchArgument('left_ir_height', default_value='0'),
        DeclareLaunchArgument('left_ir_fps', default_value='0'),
        DeclareLaunchArgument('left_ir_format', default_value='ANY'),
        DeclareLaunchArgument('enable_left_ir', default_value='false'),
        DeclareLaunchArgument('left_ir_qos', default_value='default'),
        DeclareLaunchArgument('left_ir_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('right_ir_width', default_value='0'),
        DeclareLaunchArgument('right_ir_height', default_value='0'),
        DeclareLaunchArgument('right_ir_fps', default_value='0'),
        DeclareLaunchArgument('right_ir_format', default_value='ANY'),
        DeclareLaunchArgument('enable_right_ir', default_value='false'),
        DeclareLaunchArgument('right_ir_qos', default_value='default'),
        DeclareLaunchArgument('right_ir_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('enable_ir_auto_exposure', default_value='true'),
        DeclareLaunchArgument('enable_sync_output_accel_gyro', default_value='true'),
        DeclareLaunchArgument('enable_accel', default_value='false'),
        DeclareLaunchArgument('accel_rate', default_value='200hz'),
        DeclareLaunchArgument('accel_range', default_value='4g'),
        DeclareLaunchArgument('enable_gyro', default_value='false'),
        DeclareLaunchArgument('gyro_rate', default_value='200hz'),
        DeclareLaunchArgument('gyro_range', default_value='1000dps'),
        DeclareLaunchArgument('liner_accel_cov', default_value='0.01'),
        DeclareLaunchArgument('angular_vel_cov', default_value='0.01'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('tf_publish_rate', default_value='10.0'),
        DeclareLaunchArgument('ir_info_url', default_value=''),
        DeclareLaunchArgument('color_info_url', default_value=''),
        DeclareLaunchArgument('log_level', default_value='none'),
        DeclareLaunchArgument('enable_publish_extrinsic', default_value='false'),
        DeclareLaunchArgument('enable_d2c_viewer', default_value='false'),
        DeclareLaunchArgument('enable_ldp', default_value='true'),
        DeclareLaunchArgument('enable_soft_filter', default_value='true'),
        DeclareLaunchArgument('soft_filter_max_diff', default_value='-1'),
        DeclareLaunchArgument('soft_filter_speckle_size', default_value='-1'),
        DeclareLaunchArgument('sync_mode', default_value='standalone'),
        DeclareLaunchArgument('depth_delay_us', default_value='0'),
        DeclareLaunchArgument('color_delay_us', default_value='0'),
        DeclareLaunchArgument('trigger2image_delay_us', default_value='0'),
        DeclareLaunchArgument('trigger_out_delay_us', default_value='0'),
        DeclareLaunchArgument('trigger_out_enabled', default_value='false'),
        DeclareLaunchArgument('enable_frame_sync', default_value='true'),
        DeclareLaunchArgument('ordered_pc', default_value='false'),
        DeclareLaunchArgument('use_hardware_time', default_value='true'),
        DeclareLaunchArgument('enable_depth_scale', default_value='true'),
        DeclareLaunchArgument('enable_decimation_filter', default_value='false'),
        DeclareLaunchArgument('enable_hdr_merge', default_value='false'),
        DeclareLaunchArgument('enable_sequence_id_filter', default_value='false'),
        DeclareLaunchArgument('enable_threshold_filter', default_value='false'),
        DeclareLaunchArgument('enable_noise_removal_filter', default_value='true'),
        DeclareLaunchArgument('enable_spatial_filter', default_value='false'),
        DeclareLaunchArgument('enable_temporal_filter', default_value='false'),
        DeclareLaunchArgument('enable_hole_filling_filter', default_value='false'),
        DeclareLaunchArgument('decimation_filter_scale_', default_value='-1'),
        DeclareLaunchArgument('sequence_id_filter_id', default_value='-1'),
        DeclareLaunchArgument('threshold_filter_max', default_value='-1'),
        DeclareLaunchArgument('threshold_filter_min', default_value='-1'),
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
        DeclareLaunchArgument('diagnostic_period', default_value='1.0'),
        DeclareLaunchArgument('enable_laser', default_value='true'),
        DeclareLaunchArgument('depth_precision', default_value=''),
        DeclareLaunchArgument('device_preset', default_value='Default'),
        # Laser on/off alternate mode, 0: off, 1: on-off alternate, 2: off-on alternate.
        DeclareLaunchArgument('laser_on_off_mode', default_value='0'),
        DeclareLaunchArgument('retry_on_usb3_detection_failure', default_value='false'),
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
                    output="screen",
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
            output="screen",
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
