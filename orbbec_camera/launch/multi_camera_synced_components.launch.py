from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction
def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory("orbbec_camera")
    launch_file_dir = os.path.join(package_dir, "launch")
    config_file_dir = os.path.join(package_dir, "config")
    config_file_335_path = os.path.join(config_file_dir, "camera_335_params.yaml")
    config_file_335L_path = os.path.join(config_file_dir, "camera_335L_params.yaml")

    use_intra_process_comms_arg = DeclareLaunchArgument(
        'use_intra_process_comms', default_value='false',
    )
    attach_to_shared_component_container_arg = DeclareLaunchArgument(
        'attach_to_shared_component_container', default_value='false',
    )

    shared_container_name = "shared_orbbec_container"
    shared_container = Node(
        name=shared_container_name,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        condition=IfCondition(LaunchConfiguration('attach_to_shared_component_container'))
    )

    G0_6D = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_interleave_laser_g335_components.launch.py")
        ),
        launch_arguments={
            "camera_name": "G0_6D",
            "usb_port": "2-3.2",
            "device_num": "2",
            "sync_mode": "primary",
            'use_intra_process_comms': LaunchConfiguration("use_intra_process_comms"),
            'attach_to_shared_component_container': LaunchConfiguration("attach_to_shared_component_container"),
            'component_container_name': shared_container_name,
            "config_file_path": config_file_335_path,
        }.items(),
    )

    G1_6H = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_interleave_laser_g335_components.launch.py")
        ),
        launch_arguments={
            "camera_name": "G1_6H",
            "usb_port": "2-3.1",
            "device_num": "2",
            "sync_mode": "secondary_synced",
            'use_intra_process_comms': LaunchConfiguration("use_intra_process_comms"),
            'attach_to_shared_component_container': LaunchConfiguration("attach_to_shared_component_container"),
            'component_container_name': shared_container_name,
            "config_file_path": config_file_335_path,
        }.items(),
    )


    # If you need more cameras, just add more launch_include here, and change the usb_port and device_num



    ld = LaunchDescription([
        use_intra_process_comms_arg,
        attach_to_shared_component_container_arg,
        shared_container,
        GroupAction([G0_6D]),
        TimerAction(period=0.5, actions=[GroupAction([G1_6H])]),
    ])

    return ld

