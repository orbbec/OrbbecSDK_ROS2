from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory('orbbec_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')
    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'lidar.launch.py')
        ),
        launch_arguments={
            'camera_name': 'lidar_01',
            'device_num': '2',
            'net_device_ip': '192.168.1.100',
            'net_device_port': '2228',
        }.items()
    )

    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'lidar.launch.py')
        ),
        launch_arguments={
            'camera_name': 'lidar_02',
            'device_num': '2',
            'net_device_ip': '192.168.1.101',
            'net_device_port': '2228',
        }.items()
    )

    # If you need more lidar, just add more launch_include here, and change the net_device_ip and net_device_port

    # Launch description
    ld = LaunchDescription([
        GroupAction([launch1_include]),
        GroupAction([launch2_include]),
    ])

    return ld
