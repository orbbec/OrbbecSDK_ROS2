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

    G0_51 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_interleave_laser_g335L.launch.py")
        ),
        launch_arguments={
            "camera_name": "G0_51",
            "usb_port": "2-2",
            "device_num": "4",
            "sync_mode": "primary",
            "enable_left_ir":"true",
            "config_file_path": config_file_path,
        }.items(),
    )

    G1_54 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_interleave_laser_g335L.launch.py")
        ),
        launch_arguments={
            "camera_name": "G1_54",
            "usb_port": "2-3.3",
            "device_num": "4",
            "sync_mode": "secondary_synced",
            "enable_left_ir":"true",
            "config_file_path": config_file_path,
        }.items(),
    )
    G2_5Y = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_interleave_laser_g335L.launch.py")
        ),
        launch_arguments={
            "camera_name": "G2_5Y",
            "usb_port": "2-3.1",
            "device_num": "4",
            "sync_mode": "secondary_synced",
            "enable_left_ir":"true",
            "config_file_path": config_file_path,
        }.items(),
    )
    G3_47 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_interleave_laser_g335L.launch.py")
        ),
        launch_arguments={
            "camera_name": "G3_47",
            "usb_port": "2-1",
            "device_num": "4",
            "sync_mode": "secondary_synced",
            "enable_left_ir":"true",
            "config_file_path": config_file_path,
        }.items(),
    )

    multi_save_rgbir_node = Node(
        package="orbbec_camera",
        executable="multi_save_rgbir_node",
        name="multi_save_rgbir_node",
    )

    # If you need more cameras, just add more launch_include here, and change the usb_port and device_num

    # Launch description
    ld = LaunchDescription(
        [
            GroupAction([multi_save_rgbir_node]),
            TimerAction(
                period=2.0,
                actions=[
                    TimerAction(period=0.5, actions=[GroupAction([G1_54])]),
                    TimerAction(period=0.5, actions=[GroupAction([G2_5Y])]),
                    TimerAction(period=0.5, actions=[GroupAction([G3_47])]),
                    TimerAction(period=0.5, actions=[GroupAction([G0_51])]),
                ],
            ),
            # The primary camera should be launched at last
        ]
    )

    return ld
