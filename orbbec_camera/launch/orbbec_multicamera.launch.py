import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory('orbbec_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')

    G330_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'orbbec_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'gemini330_series',
            'attach_component_container_enable': 'true',
            'camera_name': 'G330_0',
            'usb_port': '2-1',
            'device_num': '4',
            # 'sync_mode': 'primary',
        }.items()
    )

    G330_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'orbbec_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'gemini330_series',
            'attach_component_container_enable': 'false',
            'camera_name': 'G330_1',
            'usb_port': '2-4',
            'device_num': '4',
            # 'sync_mode': 'secondary_synced',
            # 'trigger_out_enabled': 'false',
        }.items()
    )

    G330_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'orbbec_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'gemini330_series',
            'attach_component_container_enable': 'true',
            'camera_name': 'G330_2',
            'usb_port': '2-7',
            'device_num': '4',
            # 'sync_mode': 'secondary_synced',
            # 'trigger_out_enabled': 'false',
        }.items()
    )

    G330_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'orbbec_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'gemini330_series',
            'attach_component_container_enable': 'true',
            'camera_name': 'G330_3',
            'usb_port': '2-3',
            'device_num': '4',
            # 'sync_mode': 'secondary_synced',
            # 'trigger_out_enabled': 'false',
        }.items()
    )

    # If you need more cameras, just add more launch_include here, and change the usb_port and device_num

    # Launch description
    ld = LaunchDescription(
        [
            TimerAction(period=0.0, actions=[GroupAction([G330_1])]),
            TimerAction(period=2.0, actions=[GroupAction([G330_2])]),
            TimerAction(period=4.0, actions=[GroupAction([G330_3])]),
            TimerAction(period=6.0, actions=[GroupAction([G330_0])]),
            # The primary camera should be launched at last
        ]
    )

    return ld