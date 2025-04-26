import os
import yaml
import logging
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import UnlessCondition
from launch_ros.actions import LoadComposableNodes
from ament_index_python.packages import get_package_share_directory
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from utils_yaml import *

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


default_package_name = 'orbbec_camera'
default_camera_model = 'gemini330_series'
default_common_yaml  = 'common.yaml'
default_component_container_name = 'orbbec_container'

def generate_launch_arguments():
    launch_arguments = [
        #general config
        DeclareLaunchArgument('camera_model', default_value=default_camera_model),
        DeclareLaunchArgument('config_file_path', default_value=''),
        #multi-device sync param
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('usb_port', default_value=''),
        DeclareLaunchArgument('device_num', default_value='1'),
        DeclareLaunchArgument('sync_mode', default_value='standalone'),
        #instra-process demo set
        DeclareLaunchArgument('use_intra_process_comms', default_value='false'),
        DeclareLaunchArgument('attach_component_container_enable', default_value='false'),
        DeclareLaunchArgument('attach_component_container_name', default_value=default_component_container_name),
    ]
    return launch_arguments


def load_common_yaml_params():
    common_yaml_path = os.path.join(
        get_package_share_directory(default_package_name), 'config', default_common_yaml
    )
    common_yaml_path = get_absolute_path(common_yaml_path, package_name=default_package_name)
    logger.info(f"load convert Absolute path: {common_yaml_path}")
    return load_and_parse_yaml(common_yaml_path)


def load_config_yaml_params(config_file_path, camera_model):
    config_yaml_path = config_file_path or os.path.join(
        get_package_share_directory(default_package_name), 'config', f"{camera_model}.yaml"
    )
    config_yaml_path = get_absolute_path(config_yaml_path, package_name=default_package_name)
    print(f"load convert Absolute config_yaml_path: {config_yaml_path}")
    return load_and_parse_yaml(config_yaml_path)


def load_parameters(context, args):
    camera_model = LaunchConfiguration("camera_model").perform(context)
    config_file_path = LaunchConfiguration("config_file_path").perform(context)
    usb_port = LaunchConfiguration("usb_port").perform(context)
    print(f"camera_model: {camera_model}", f"config_file_path: {config_file_path}", f"usb_port: {usb_port}")

    try:
        #1.load ymal params
        yaml_common_params = load_common_yaml_params()
        yaml_params = load_config_yaml_params(config_file_path,camera_model)
        #print_params(yaml_common_params)
        #print_params(yaml_params)
        yaml_params = update_params(yaml_common_params,yaml_params)
        #print_params(yaml_params)

        #2.load args params
        args_params = {arg.name: LaunchConfiguration(arg.name).perform(context) for arg in args}
        #print_params(args_params)

        #3.meger params args
        if usb_port is None or usb_port == "": #meger yaml->args params
            print("usb_port param null")
            default_params = update_params(args_params, yaml_params)
        else:
            print("usb_port param not null value:", usb_port) #meger args->yaml params
            default_params = update_params(yaml_params, args_params)
        #print_params(default_params)

        #4.load cmd params & meger params cmd->yaml
        cmd_params = dict([a for a in [a.split(':=') for a in sys.argv] if len(a) == 2])
        default_params = update_params(default_params,cmd_params)
        #print_params(args_params)

    except (FileNotFoundError, yaml.YAMLError) as e:
        logger.error(f"Error loading YAML file: {e}")
        return {}

    #5.Skip conversion for specific parameters
    SKIP_CONVERT = {'config_file_path', 'usb_port', 'serial_number', 'attach_component_container_name'}
    return {
        key: (value if key in SKIP_CONVERT else convert_value(value))
        for key, value in default_params.items()
    }


def find_camera_name(params):
    return find_param(params, 'camera_name', 'camera')

def find_use_intra_process_comms(params):
    return find_param(params, 'use_intra_process_comms', False)


def generate_launch_description():
    args = generate_launch_arguments()
    def get_params(context, args):
        return [load_parameters(context, args)]

    def create_composable_node(camera_name, params, use_intra_process):
        common_arguments = [{'use_intra_process_comms': use_intra_process}]
        composable_node = ComposableNode(
                namespace=camera_name,
                name=camera_name,
                package=default_package_name,
                plugin='orbbec_camera::OBCameraNodeDriver',
                parameters=params,
                #extra_arguments=[{'use_intra_process_comms': LaunchConfiguration("use_intra_process_comms")}],
                extra_arguments=common_arguments,
            )
        return composable_node


    def create_orbbec_container(attach_component_container_name, attach_component_container_enable):
        orbbec_container = Node(
            name=attach_component_container_name,
            package='rclcpp_components',
            executable='component_container_mt',
            output='screen',
            condition=UnlessCondition(attach_component_container_enable)
        )
        return orbbec_container


    def create_node_action(context, args):
        params = get_params(context, args)
        camera_name = find_camera_name(params)

        use_intra_process = find_use_intra_process_comms(params)
        #use_intra_process = LaunchConfiguration("use_intra_process_comms", default=False).perform(context)
        print(f"camera_name: {camera_name}", f"use_intra_process: {use_intra_process}")

        attach_enable_condition = LaunchConfiguration("attach_component_container_enable", default=False).perform(context)
        attach_component_container_name = LaunchConfiguration("attach_component_container_name", default='orbbec_container').perform(context)
        print(f"attach_enable_condition: {attach_enable_condition}", f"attach_component_container_name: {attach_component_container_name}")

        ros_distro = os.environ.get("ROS_DISTRO", "humble")
        if ros_distro == "foxy":
            return [
                Node(
                    package=default_package_name,
                    executable="orbbec_camera_node",
                    name="ob_camera_node",
                    namespace=camera_name,
                    parameters=params,
                    output="screen",
                    #condition=UnlessCondition(attach_enable_condition)
                    #prefix=['xterm -e gdb -ex run --args'],
                ),
            ]
        else:
            composable_node = create_composable_node(camera_name, params, use_intra_process)
            orbbec_container= create_orbbec_container(attach_component_container_name, attach_enable_condition)
            return [
                orbbec_container,
                LoadComposableNodes(
                    target_container=attach_component_container_name,
                    composable_node_descriptions=[composable_node]
                ),
            ]

    return LaunchDescription(
        args + [
            OpaqueFunction(function=lambda context: create_node_action(context, args))
        ]
    )