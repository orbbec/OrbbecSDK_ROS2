import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from utils_yaml import *


default_package_name = 'orbbec_camera'
default_launch_file = 'orbbec_camera.launch.py'
default_multi_camera_config_file = 'multicamera.yaml' #'multi_camera_synced.ymal'
default_shared_container_name = 'shared_orbbec_container'


def generate_launch_arguments():
    launch_arguments = [
        #general config
        DeclareLaunchArgument('config_file_path', default_value=default_multi_camera_config_file),
        #instra-process demo set
        DeclareLaunchArgument('use_intra_process_comms', default_value='false'), #false
        DeclareLaunchArgument('attach_component_container_enable', default_value='false'), #false
        DeclareLaunchArgument('attach_component_container_name', default_value=default_shared_container_name),
    ]
    return launch_arguments


def generate_camera_node(args, launch_file_dir, camera_params, launch_file=default_launch_file):
    config_params = {key: value for key, value in camera_params.items()}
    #load launch_arguments and meger to yaml params
    args_params = {arg.name: arg.default_value for arg in args}
    #print_params(args_params)
    default_params = update_params(args_params, config_params)
    #print_params(default_params)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, launch_file)
        ),
        launch_arguments=default_params.items()
    )


def load_yaml_params(config_file_path):
    config_yaml_path = config_file_path or os.path.join(get_package_share_directory(default_package_name), 'config', config_file_path)
    config_yaml_path = get_absolute_path(config_yaml_path, package_name=default_package_name)
    print(f"load convert Absolute config_yaml_path: {config_yaml_path}")
    return load_yaml(config_yaml_path)


def generate_launch_description():
    args=generate_launch_arguments()
    package_dir = get_package_share_directory(default_package_name)
    launch_file_dir = os.path.join(package_dir, "launch")


    def create_share_container(attach_component_container_name, attach_enable_condition):
        shared_container = Node(
            name=attach_component_container_name,
            package='rclcpp_components',
            executable='component_container_mt',
            output='screen',
            condition=IfCondition(attach_enable_condition)
        )
        return shared_container


    def create_node_action(context, args):
        attach_enable_condition = LaunchConfiguration("attach_component_container_enable", default=False).perform(context)
        attach_component_container_name = LaunchConfiguration("attach_component_container_name", default=default_shared_container_name).perform(context)
        use_intra_process_comms = LaunchConfiguration("use_intra_process_comms", default=False).perform(context)
        print(f"attach_enable_condition: {attach_enable_condition}", f"attach_component_container_name: {attach_component_container_name}")
        print(f"use_intra_process_comms: {use_intra_process_comms}")
        shared_container_node = create_share_container(attach_component_container_name, attach_enable_condition)

        config_file_path = LaunchConfiguration("config_file_path", default=default_multi_camera_config_file).perform(context)
        print(f"config_file_path: {config_file_path}")
        #camera_config_file = os.path.join(package_dir, "config", config_file_path)
        config_params = load_yaml_params(config_file_path)
        if not config_params:
            raise Exception("Failed to load multi-camera configuration.")

        camera_params = config_params['orbbec_ros']['multi_camera_parameters']
        camera_actions = []
        for camera_key in camera_params:
            if camera_key.startswith('launch_device_camera'):
                camera_config = camera_params[camera_key]
                #print_params(camera_config)
                period_value = float(camera_config.pop('period', '0.0'))
                print(f"period_value: {period_value}", f"camera_config: {camera_config}")

                camera_node = generate_camera_node(args, launch_file_dir, camera_config)
                camera_action = TimerAction(period=period_value, actions=[camera_node])
                camera_actions.append(camera_action)

        nodes = [
                shared_container_node,
                *camera_actions,
            ]
        return nodes

    return LaunchDescription(
        args + [
            OpaqueFunction(function=lambda context: create_node_action(context, args))
        ]
    )