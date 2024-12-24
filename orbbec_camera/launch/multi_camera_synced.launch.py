import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory("orbbec_camera")
    launch_file_dir = os.path.join(package_dir, "launch")
    config_file_dir = os.path.join(package_dir, "config")
    config_file_path = os.path.join(config_file_dir, "camera_params.yaml")

    G330_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series.launch.py")
        ),
        launch_arguments={
            "camera_name": "G330_0",
            "usb_port": "2-7",
            "device_num": "2",
            "sync_mode": "primary",
            "config_file_path": config_file_path,
        }.items(),
    )

    G330_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series.launch.py")
        ),
        launch_arguments={
            "camera_name": "G330_1",
            "usb_port": "2-1",
            "device_num": "2",
            "sync_mode": "secondary_synced",
            "config_file_path": config_file_path,
        }.items(),
    )


    # If you need more cameras, just add more launch_include here, and change the usb_port and device_num

    # Launch description
    ld = LaunchDescription(
        [

            TimerAction(period=0.0, actions=[GroupAction([G330_1])]),
            TimerAction(period=2.0, actions=[GroupAction([G330_0])]),
            # The primary camera should be launched at last
        ]
    )

    return ld

