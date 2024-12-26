import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
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
    config_file_path = os.path.join(config_file_dir, "camera_params.yaml")

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

    front_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series.launch.py")
        ),
        launch_arguments={
            "camera_name": "front_camera",
            "usb_port": "2-1",
            # "usb_port": "gmsl2-4",
            "device_num": "4",
            "sync_mode": "primary",
            # "sync_mode": "secondary_synced",
            "config_file_path": config_file_path,
            'use_intra_process_comms': LaunchConfiguration("use_intra_process_comms"),
            'attach_to_shared_component_container': attach_to_shared_component_container_arg,
            'component_container_name': component_container_name_arg,
            # 'gmsl_trigger_fps': "5990",
            # 'enable_gmsl_trigger': "true",
        }.items(),
    )
    left_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series.launch.py")
        ),
        launch_arguments={
            "camera_name": "left_camera",
            "usb_port": "2-3.1",
            # "usb_port": "gmsl2-3",
            "device_num": "4",
            "sync_mode": "secondary_synced",
            "config_file_path": config_file_path,
            'use_intra_process_comms': LaunchConfiguration("use_intra_process_comms"),
            'attach_to_shared_component_container': attach_to_shared_component_container_arg,
            'component_container_name': component_container_name_arg,
        }.items(),
    )
    right_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series.launch.py")
        ),
        launch_arguments={
            "camera_name": "right_camera",
            "usb_port": "2-3.3",
            # "usb_port": "gmsl2-1",
            "device_num": "4",
            "sync_mode": "secondary_synced",
            "config_file_path": config_file_path,
            'use_intra_process_comms': LaunchConfiguration("use_intra_process_comms"),
            'attach_to_shared_component_container': attach_to_shared_component_container_arg,
            'component_container_name': component_container_name_arg,
        }.items(),
    )
    rear_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series.launch.py")
        ),
        launch_arguments={
            "camera_name": "rear_camera",
            "usb_port": "2-2",
            # "usb_port": "gmsl2-7",
            "device_num": "4",
            "sync_mode": "secondary_synced",
            "config_file_path": config_file_path,
            'use_intra_process_comms': LaunchConfiguration("use_intra_process_comms"),
            'attach_to_shared_component_container': attach_to_shared_component_container_arg,
            'component_container_name': component_container_name_arg,
        }.items(),
    )

    delayed_left_camera = TimerAction(
        period=0.0,
        actions=[left_camera],
    )
    delayed_right_camera = TimerAction(
        period=2.0,
        actions=[right_camera],
    )
    delayed_rear_camera = TimerAction(
        period=4.0,
        actions=[rear_camera],
    )
    delayed_front_camera = TimerAction(
        period=6.0,
        actions=[front_camera],
    )
    ld = LaunchDescription(
      [
        use_intra_process_comms_declare,
        attach_to_shared_component_container_declare,
        component_container_name_declare,
        shared_orbbec_container,
        delayed_left_camera,
        delayed_right_camera,
        delayed_rear_camera,
        delayed_front_camera,
      ]
    )

    return ld