import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory("orbbec_camera")
    launch_file_dir = os.path.join(package_dir, "launch")
    config_file_dir = os.path.join(package_dir, "config")
    config_file_335_path = os.path.join(config_file_dir, "camera_335_params.yaml")
    config_file_335L_path = os.path.join(config_file_dir, "camera_335L_params.yaml")

    use_intra_process_comms_declare = DeclareLaunchArgument(
        'use_intra_process_comms', default_value='false',
    )
    attach_to_shared_component_container_declare = DeclareLaunchArgument(
        'attach_to_shared_component_container', default_value='false',
    )
    component_container_name_declare = DeclareLaunchArgument(
      'component_container_name', default_value='shared_orbbec_container'
    )

    attach_to_shared_component_container_arg = LaunchConfiguration('attach_to_shared_component_container', default=False)
    component_container_name_arg = LaunchConfiguration('component_container_name', default='shared_orbbec_container')

    shared_orbbec_container = Node(
        name=component_container_name_arg,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        condition=UnlessCondition(attach_to_shared_component_container_arg)
    )

    attach_to_shared_component_container_arg = TextSubstitution(text='true')

    G0_41 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_interleave_laser_g335_components.launch.py")
        ),
        launch_arguments={
            "camera_name": "G0_41",
            "usb_port": "2-7",
            "device_num": "2",
            "sync_mode": "primary",
            'use_intra_process_comms': LaunchConfiguration("use_intra_process_comms"),
            'attach_to_shared_component_container': attach_to_shared_component_container_arg,
            'component_container_name': component_container_name_arg,
            "config_file_path": config_file_335_path,
        }.items(),
    )

    G1_50 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_interleave_laser_g335_components.launch.py")
        ),
        launch_arguments={
            "camera_name": "G1_50",
            "usb_port": "2-1",
            "device_num": "2",
            "sync_mode": "secondary_synced",
            'use_intra_process_comms': LaunchConfiguration("use_intra_process_comms"),
            'attach_to_shared_component_container': attach_to_shared_component_container_arg,
            'component_container_name': component_container_name_arg,
            "config_file_path": config_file_335_path,
        }.items(),
    )


    # If you need more cameras, just add more launch_include here, and change the usb_port and device_num

    ld = LaunchDescription([
        use_intra_process_comms_declare,
        attach_to_shared_component_container_declare,
        component_container_name_declare,
        shared_orbbec_container,
        G1_50,
        TimerAction(period=3.0, actions=[G0_41]),
    ])

    return ld

