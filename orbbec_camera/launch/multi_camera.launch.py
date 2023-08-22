from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    camera_name = DeclareLaunchArgument('camera_name', default_value='camera')
    d_sensor = DeclareLaunchArgument('3d_sensor', default_value='gemini2')
    camera1_prefix = DeclareLaunchArgument('camera1_prefix', default_value='01')
    camera2_prefix = DeclareLaunchArgument('camera2_prefix', default_value='02')
    camera1_usb_port = DeclareLaunchArgument('camera1_usb_port', default_value='2-3.3')
    camera2_usb_port = DeclareLaunchArgument('camera2_usb_port', default_value='1-4.4')
    device_num = DeclareLaunchArgument('device_num', default_value='2')

    # Node configuration
    cleanup_node = Node(
        package='orbbec_camera',
        executable='ob_cleanup_shm_node',
        name='camera',
        output='screen'
    )

    # Include launch files
    launch_file_dir = FindPackageShare('orbbec_camera').find('orbbec_camera') + '/launch'
    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_dir + '/' + (LaunchConfiguration('3d_sensor') + '.launch.py')),
        launch_arguments={
            'camera_name': 'camera_' + LaunchConfiguration('camera1_prefix'),
            'usb_port': LaunchConfiguration('camera1_usb_port'),
            'device_num': LaunchConfiguration('device_num')
        }.items()
    )

    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_dir + '/' + (LaunchConfiguration('3d_sensor') + '.launch.py')),
        launch_arguments={
            'camera_name': 'camera_' + LaunchConfiguration('camera2_prefix'),
            'usb_port': LaunchConfiguration('camera2_usb_port'),
            'device_num': LaunchConfiguration('device_num')
        }.items()
    )

    # Static TF publisher
    tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'camera01_link', 'camera02_link']
    )

    # Launch description
    ld = LaunchDescription([
        camera_name,
        d_sensor,
        camera1_prefix,
        camera2_prefix,
        camera1_usb_port,
        camera2_usb_port,
        device_num,
        cleanup_node,
        GroupAction([launch1_include]),
        GroupAction([launch2_include]),
        tf_publisher
    ])

    return ld
