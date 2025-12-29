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
    secondary_config_file_path = os.path.join(config_file_dir, "camera_secondary_params.yaml")

    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_01",
            "usb_port": "2-1",
            "device_num": "2",
            "sync_mode": "primary",
            "config_file_path": config_file_path,
            "trigger_out_enabled": "true",
            "log_level": "none",
            "log_file_name": "camera_01.log",
        }.items(),
    )

    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_02",
            "usb_port": "2-3",
            "device_num": "2",
            "sync_mode": "secondary_synced",
            "config_file_path": secondary_config_file_path,
            "trigger_out_enabled": "false",
            "log_level": "none",
            "log_file_name": "camera_02.log",
        }.items(),
    )



    # Launch description
    ld = LaunchDescription(
        [

            TimerAction(period=0.0, actions=[GroupAction([launch2_include])]),
            TimerAction(period=2.0, actions=[GroupAction([launch1_include])]),
            # The primary camera should be launched at last
        ]
    )

    return ld

