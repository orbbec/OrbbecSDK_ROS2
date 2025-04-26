#!/bin/bash

# Define the vendor ID for Orbbec devices
VID="2bc5"

# Function to get USB port for a given serial number
get_usb_port() {
    local serial=$1
    for dev in /sys/bus/usb/devices/*; do
        if [ -e "$dev/idVendor" ]; then
            vid=$(cat "$dev/idVendor")
            if [ "$vid" == "${VID}" ]; then
                dev_serial=$(cat "$dev/serial" 2>/dev/null)
                if [ "$dev_serial" == "$serial" ]; then
                    echo $(basename $dev)
                    return
                fi
            fi
        fi
    done
    echo "Unknown"
}

# Get USB ports for each camera
front_port=$(get_usb_port "CP3S34D00051") # primary camera
rear_port=$(get_usb_port "CP3L44P00047") # secondary-synced camera
left_port=$(get_usb_port "CP3L44P0005Y") # secondary-synced camera
right_port=$(get_usb_port "CP3L44P00054") #  secondary-synced camera

# Generate the launch file
cat << EOF > multi_camera_synced.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('orbbec_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')
    config_file_dir = os.path.join(package_dir, 'config')
    config_file_path = os.path.join(config_file_dir, 'camera_params.yaml')

    front_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'camera_name': 'front_camera',
            'usb_port': '$front_port',
            'device_num': '4',
            'sync_mode': 'software_triggering',
            'config_file_path': config_file_path

        }.items()
    )

    rear_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'camera_name': 'rear_camera',
            'usb_port': '$rear_port',
            'device_num': '4',
            'sync_mode': 'hardware_triggering',
            'config_file_path': config_file_path

        }.items()
    )

    left_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'camera_name': 'left_camera',
            'usb_port': '$left_port',
            'device_num': '4',
            'sync_mode': 'hardware_triggering',
            'config_file_path': config_file_path

        }.items()
    )

    right_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'camera_name': 'right_camera',
            'usb_port': '$right_port',
            'device_num': '4',
            'sync_mode': 'hardware_triggering',
            'config_file_path': config_file_path

        }.items()
    )

    ld = LaunchDescription([
        GroupAction([rear_camera]),
        GroupAction([left_camera]),
        GroupAction([right_camera]),
        TimerAction(period=3.0, actions=[GroupAction([front_camera])]), # The primary camera should be launched at last

    ])

    return ld
EOF

echo "Launch file 'multi_camera_synced.launch.py' has been generated."
