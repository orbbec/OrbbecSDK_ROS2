from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Declare arguments
    args = [
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('depth_registration', default_value='false'),
        DeclareLaunchArgument('serial_number', default_value=''),
        DeclareLaunchArgument('usb_port', default_value=''),
        DeclareLaunchArgument('device_num', default_value='1'),
        DeclareLaunchArgument('vendor_id', default_value='0x2bc5'),
        DeclareLaunchArgument('product_id', default_value=''),
        DeclareLaunchArgument('point_cloud_qos', default_value='default'),
        DeclareLaunchArgument('connection_delay', default_value='100'),
        DeclareLaunchArgument('color_width', default_value='1280'),
        DeclareLaunchArgument('color_height', default_value='800'),
        DeclareLaunchArgument('color_fps', default_value='10'),
        DeclareLaunchArgument('color_format', default_value='MJPG'),
        DeclareLaunchArgument('enable_color', default_value='true'),
        DeclareLaunchArgument('flip_color', default_value='false'),
        DeclareLaunchArgument('color_qos', default_value='default'),
        DeclareLaunchArgument('color_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('enable_color_auto_exposure', default_value='true'),
        DeclareLaunchArgument('left_ir_width', default_value='1280'),
        DeclareLaunchArgument('left_ir_height', default_value='800'),
        DeclareLaunchArgument('left_ir_fps', default_value='10'),
        DeclareLaunchArgument('left_ir_format', default_value='Y8'),
        DeclareLaunchArgument('enable_left_ir', default_value='true'),
        DeclareLaunchArgument('flip_left_ir', default_value='false'),
        DeclareLaunchArgument('left_ir_qos', default_value='default'),
        DeclareLaunchArgument('left_ir_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('right_ir_width', default_value='1280'),
        DeclareLaunchArgument('right_ir_height', default_value='800'),
        DeclareLaunchArgument('right_ir_fps', default_value='10'),
        DeclareLaunchArgument('right_ir_format', default_value='Y8'),
        DeclareLaunchArgument('enable_right_ir', default_value='true'),
        DeclareLaunchArgument('flip_right_ir', default_value='false'),
        DeclareLaunchArgument('right_ir_qos', default_value='default'),
        DeclareLaunchArgument('right_ir_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('enable_ir_auto_exposure', default_value='true'),
        DeclareLaunchArgument('enable_accel', default_value='true'),
        DeclareLaunchArgument('accel_rate', default_value='100hz'),
        DeclareLaunchArgument('accel_range', default_value='4g'),
        DeclareLaunchArgument('enable_gyro', default_value='true'),
        DeclareLaunchArgument('gyro_rate', default_value='1KHZ'),
        DeclareLaunchArgument('gyro_range', default_value='500dps'),
        DeclareLaunchArgument('liner_accel_cov', default_value='0.01'),
        DeclareLaunchArgument('angular_vel_cov', default_value='0.01'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('tf_publish_rate', default_value='10.0'),
        DeclareLaunchArgument('ir_info_url', default_value=''),
        DeclareLaunchArgument('color_info_url', default_value=''),
        DeclareLaunchArgument('log_level', default_value='none'),
        DeclareLaunchArgument('enable_publish_extrinsic', default_value='false'),
        DeclareLaunchArgument('enable_d2c_viewer', default_value='false'),
        DeclareLaunchArgument('enable_soft_filter', default_value='true'),
        DeclareLaunchArgument('enable_ldp', default_value='true'),
        DeclareLaunchArgument('enable_soft_filter', default_value='true'),
        DeclareLaunchArgument('soft_filter_max_diff', default_value='-1'),
        DeclareLaunchArgument('soft_filter_speckle_size', default_value='-1'),
        DeclareLaunchArgument('sync_mode', default_value='free_run'),
        DeclareLaunchArgument('depth_delay_us', default_value='0'),
        DeclareLaunchArgument('color_delay_us', default_value='0'),
        DeclareLaunchArgument('trigger2image_delay_us', default_value='0'),
        DeclareLaunchArgument('trigger_signal_output_delay_us', default_value='0'),
        DeclareLaunchArgument('trigger_signal_output_enabled', default_value='false'),
        DeclareLaunchArgument('enable_frame_sync', default_value='true'),
    ]

    # Node configuration
    parameters = [{arg.name: LaunchConfiguration(arg.name)} for arg in args]
    # Define the ComposableNode
    compose_node = ComposableNode(
        package='orbbec_camera',
        plugin='orbbec_camera::OBCameraNodeDriver',
        name=LaunchConfiguration('camera_name'),
        namespace='',
        parameters=parameters,
    )
    # Define the ComposableNodeContainer
    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            compose_node,
        ],
        output='screen',
    )

    # Launch description
    ld = LaunchDescription(
        args +
        [
            GroupAction([
                PushRosNamespace(LaunchConfiguration('camera_name')),
                container
            ])
        ]
    )
    return ld
