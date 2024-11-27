import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory("orbbec_camera")
    launch_file_dir = os.path.join(package_dir, "launch")
    config_file_dir = os.path.join(package_dir, "config")
    config_file_path = os.path.join(config_file_dir, "camera_params.yaml")

    use_intra_process_comms_arg = DeclareLaunchArgument(
        'use_intra_process_comms', default_value='true',
    )
    attach_to_shared_component_container_arg = DeclareLaunchArgument(
        'attach_to_shared_component_container', default_value='true',
    )

    shared_container_name = "shared_orbbec_container"
    shared_container = Node(
        name=shared_container_name,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        condition=IfCondition(LaunchConfiguration('attach_to_shared_component_container'))
    )

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
            'use_intra_process_comms': LaunchConfiguration("use_intra_process_comms"),
            'attach_to_shared_component_container': LaunchConfiguration("attach_to_shared_component_container"),
            'component_container_name': shared_container_name,
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
            'use_intra_process_comms': LaunchConfiguration("use_intra_process_comms"),
            'attach_to_shared_component_container': LaunchConfiguration("attach_to_shared_component_container"),
            'component_container_name': shared_container_name,
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
            'use_intra_process_comms': LaunchConfiguration("use_intra_process_comms"),
            'attach_to_shared_component_container': LaunchConfiguration("attach_to_shared_component_container"),
            'component_container_name': shared_container_name,
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
            'use_intra_process_comms': LaunchConfiguration("use_intra_process_comms"),
            'attach_to_shared_component_container': LaunchConfiguration("attach_to_shared_component_container"),
            'component_container_name': shared_container_name,
        }.items(),
    )

    # Launch description
    # ld = LaunchDescription(
    #   [
    #       TimerAction(period=0.5, actions=[GroupAction([left_camera])]),
    #       TimerAction(period=0.5, actions=[GroupAction([right_camera])]),
    #       TimerAction(period=0.5, actions=[GroupAction([rear_camera])]),
    #       TimerAction(period=0.5, actions=[GroupAction([front_camera])]),
    #   ]
    # )
    delayed_left_camera = TimerAction(
        period=0.5,
        actions=[left_camera],
    )
    delayed_right_camera = TimerAction(
        period=0.5,
        actions=[right_camera],
    )
    delayed_rear_camera = TimerAction(
        period=0.5,
        actions=[rear_camera],
    )
    delayed_front_camera = TimerAction(
        period=0.5,
        actions=[front_camera],
    )
    ld = LaunchDescription(
      [
        use_intra_process_comms_arg,
        attach_to_shared_component_container_arg,
        shared_container,
        delayed_left_camera,
        delayed_right_camera,
        delayed_rear_camera,
        delayed_front_camera,
      ]
    )

    return ld