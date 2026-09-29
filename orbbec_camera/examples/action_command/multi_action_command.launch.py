import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("orbbec_camera")
    camera_launch = os.path.join(
        package_dir, "launch", "gemini_330_series.launch.py"
    )

    camera_01 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch),
        launch_arguments={
            "camera_name": "camera_01",
            "enumerate_net_device": "false",
            "net_device_ip": "192.168.1.10",
            "net_device_port": "8090",
            "sync_mode": "group_actions",
            "enable_ptp_config": "true",
            "log_file_name": "camera_01.log",
        }.items(),
    )

    camera_02 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch),
        launch_arguments={
            "camera_name": "camera_02",
            "enumerate_net_device": "false",
            "net_device_ip": "192.168.1.11",
            "net_device_port": "8090",
            "sync_mode": "group_actions",
            "enable_ptp_config": "true",
            "log_file_name": "camera_02.log",
        }.items(),
    )

    action_command_node = Node(
        package="orbbec_camera",
        executable="action_command_node",
        name="action_command_node",
        output="screen",
    )

    return LaunchDescription(
        [
            action_command_node,
            TimerAction(period=0.0, actions=[GroupAction([camera_01])]),
            TimerAction(period=2.0, actions=[GroupAction([camera_02])]),
        ]
    )
