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

    front_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_interleave_laser_g335L.launch.py")
        ),
        launch_arguments={
            "camera_name": "front_camera",
            "usb_port": "2-1",
            "device_num": "4",
            "sync_mode": "primary",
            "config_file_path": config_file_path,
        }.items(),
    )

    left_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_interleave_laser_g335L.launch.py")
        ),
        launch_arguments={
            "camera_name": "left_camera",
            "usb_port": "2-3.2",
            "device_num": "4",
            "sync_mode": "secondary_synced",
            "config_file_path": config_file_path,
        }.items(),
    )
    right_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_interleave_laser_g335L.launch.py")
        ),
        launch_arguments={
            "camera_name": "right_camera",
            "usb_port": "2-3.4",
            "device_num": "4",
            "sync_mode": "secondary_synced",
            "config_file_path": config_file_path,
        }.items(),
    )
    rear_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_interleave_laser_g335L.launch.py")
        ),
        launch_arguments={
            "camera_name": "rear_camera",
            "usb_port": "2-2",
            "device_num": "4",
            "sync_mode": "secondary_synced",
            "config_file_path": config_file_path,
        }.items(),
    )

    # Launch description
    ld = LaunchDescription(
      [
          TimerAction(period=0.5, actions=[GroupAction([left_camera])]),
          TimerAction(period=0.5, actions=[GroupAction([right_camera])]),
          TimerAction(period=0.5, actions=[GroupAction([rear_camera])]),
          TimerAction(period=0.5, actions=[GroupAction([front_camera])]),
      ]
    )

    return ld