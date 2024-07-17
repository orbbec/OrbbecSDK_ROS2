import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory('orbbec_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')
    config_file_dir = os.path.join(package_dir, 'config')
    config_file_path = os.path.join(config_file_dir, 'camera_params.yaml')
    front_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'camera_name': 'front_camera',
            'usb_port': '2-1.1',
            'device_num': '4',
            'sync_mode': 'software_triggering',
            'config_file_path': config_file_path,
        }.items()
    )

    left_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'camera_name': 'left_camera',
            'usb_port': '2-1.2.1',
            'device_num': '4',
            'sync_mode': 'hardware_triggering',
            'config_file_path': config_file_path,
        }.items()
    )
    rear_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'camera_name': 'rear_camera',
            'usb_port': '2-1.2.1',
            'device_num': '4',
            'sync_mode': 'hardware_triggering',
            'config_file_path': config_file_path,
        }.items()
    )
    right_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'camera_name': 'right_camera',
            'usb_port': '2-1.2.1',
            'device_num': '4',
            'sync_mode': 'hardware_triggering',
            'config_file_path': config_file_path,
        }.items()
    )

    # If you need more cameras, just add more launch_include here, and change the usb_port and device_num

    # Launch description
    ld = LaunchDescription([
        GroupAction([rear_camera]),
        GroupAction([left_camera]),
        GroupAction([right_camera]),
        TimerAction(period=3.0, actions=[GroupAction([front_camera])]), # The primary camera should be launched at last

    ])

    return ld