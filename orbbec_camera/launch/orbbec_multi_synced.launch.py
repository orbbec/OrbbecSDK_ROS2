from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory('orbbec_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')
    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'orbbec_camera.launch.py')
        ),
        launch_arguments={
            "camera_model": "gemini330_series",
            'camera_name': 'camera_01',
            "usb_port": "4-1",
            "device_num": "3",
            "sync_mode": "primary",
        }.items()
    )

    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'orbbec_camera.launch.py')
        ),
        launch_arguments={
            "camera_model": "gemini330_series",
            "attach_component_container_enable": "true",
            'camera_name': 'camera_02',
            'usb_port': '4-2',
            'device_num': '3',
            "sync_mode": "secondary_synced",
        }.items()
    )

    launch3_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'orbbec_camera.launch.py')
        ),
        launch_arguments={
            "camera_model": "gemini330_series",
            "attach_component_container_enable": "true",
            'camera_name': 'camera_03',
            'usb_port': '4-3',
            'device_num': '3',
            "sync_mode": "secondary_synced",
        }.items()
    )

    # If you need more cameras, just add more launch_include here, and change the usb_port and device_num

    # Launch description
    ld = LaunchDescription(
        [
            TimerAction(period=3.0, actions=[GroupAction([launch1_include])]),
            TimerAction(period=0.0, actions=[GroupAction([launch2_include])]),
            TimerAction(period=1.0, actions=[GroupAction([launch3_include])]),
            # The primary camera should be launched at last
        ]
    )

    return ld