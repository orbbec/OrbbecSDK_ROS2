import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node,LoadComposableNodes
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.descriptions import ComposableNode

def load_yaml(file_path):
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def merge_params(default_params, yaml_params):
    for key, value in yaml_params.items():
        if key in default_params:
            default_params[key] = value
    return default_params


def convert_value(value):
    if isinstance(value, str):
        try:
            return int(value)
        except ValueError:
            pass
        try:
            return float(value)
        except ValueError:
            pass
        if value.lower() == 'true':
            return True
        elif value.lower() == 'false':
            return False
    return value


def load_parameters(context, args):
    default_params = {arg.name: LaunchConfiguration(arg.name).perform(context) for arg in args}
    config_file_path = LaunchConfiguration('config_file_path').perform(context)
    if config_file_path:
        yaml_params = load_yaml(config_file_path)
        default_params = merge_params(default_params, yaml_params)
    skip_convert = {'config_file_path', 'usb_port', 'serial_number'}
    return {
        key: (value if key in skip_convert else convert_value(value))
        for key, value in default_params.items()
    }
def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory("orbbec_camera")
    launch_file_dir = os.path.join(package_dir, "examples/benchmark")

    attach_to_shared_component_container_arg = LaunchConfiguration('attach_to_shared_component_container', default=False)
    component_container_name_arg = LaunchConfiguration('component_container_name', default='shared_orbbec_container')

    shared_orbbec_container = Node(
        name=component_container_name_arg,
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        condition=UnlessCondition(attach_to_shared_component_container_arg),
    )
    start_benchmark = LoadComposableNodes(
        target_container=component_container_name_arg,
        composable_node_descriptions=[
        ComposableNode(
            namespace="start_benchmark",
            name="start_benchmark",
            package='orbbec_camera',
            plugin='orbbec_camera::tools::StartBenchmark',
            )
        ]
    )

    attach_to_shared_component_container_arg = TextSubstitution(text="true")

    args = [
        DeclareLaunchArgument('uvc_backend', default_value='libuvc'),
        DeclareLaunchArgument('depth_registration', default_value='false'),
        DeclareLaunchArgument('color_width', default_value='848'),
        DeclareLaunchArgument('color_height', default_value='480'),
        DeclareLaunchArgument('color_fps', default_value='30'),
        DeclareLaunchArgument('color_format', default_value='MJPG'),
        DeclareLaunchArgument('enable_color', default_value='true'),
        DeclareLaunchArgument('depth_width', default_value='848'),
        DeclareLaunchArgument('depth_height', default_value='480'),
        DeclareLaunchArgument('depth_fps', default_value='30'),
        DeclareLaunchArgument('depth_format', default_value='ANY'),
        DeclareLaunchArgument('enable_depth', default_value='true'),
        DeclareLaunchArgument('left_ir_width', default_value='848'),
        DeclareLaunchArgument('left_ir_height', default_value='480'),
        DeclareLaunchArgument('left_ir_fps', default_value='30'),
        DeclareLaunchArgument('left_ir_format', default_value='ANY'),
        DeclareLaunchArgument('enable_left_ir', default_value='true'),
        DeclareLaunchArgument('right_ir_width', default_value='848'),
        DeclareLaunchArgument('right_ir_height', default_value='480'),
        DeclareLaunchArgument('right_ir_fps', default_value='30'),
        DeclareLaunchArgument('right_ir_format', default_value='ANY'),
        DeclareLaunchArgument('enable_right_ir', default_value='true'),
        DeclareLaunchArgument('enable_point_cloud', default_value='false'),
        DeclareLaunchArgument('enable_colored_point_cloud', default_value='false'),
        DeclareLaunchArgument('enable_decimation_filter', default_value='false'),
        DeclareLaunchArgument('enable_hdr_merge', default_value='false'),
        DeclareLaunchArgument('enable_sequence_id_filter', default_value='false'),
        DeclareLaunchArgument('enable_threshold_filter', default_value='false'),
        DeclareLaunchArgument('enable_hardware_noise_removal_filter', default_value='false'),
        DeclareLaunchArgument('enable_noise_removal_filter', default_value='false'),
        DeclareLaunchArgument('enable_spatial_filter', default_value='false'),
        DeclareLaunchArgument('enable_temporal_filter', default_value='false'),
        DeclareLaunchArgument('enable_hole_filling_filter', default_value='false'),
    ]
    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_benchmark.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_01",
            "usb_port": "2-7",
            "device_num": "4",
            "sync_mode": "standalone",
            "attach_to_shared_component_container": attach_to_shared_component_container_arg,
            "component_container_name": component_container_name_arg,
        }.items(),
    )
    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_benchmark.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_02",
            "usb_port": "2-2",
            "device_num": "4",
            "sync_mode": "standalone",
            "attach_to_shared_component_container": attach_to_shared_component_container_arg,
            "component_container_name": component_container_name_arg,
        }.items(),
    )
    launch3_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_benchmark.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_03",
            "usb_port": "2-1",
            "device_num": "4",
            "sync_mode": "standalone",
            "attach_to_shared_component_container": attach_to_shared_component_container_arg,
            "component_container_name": component_container_name_arg,
        }.items(),
    )
    launch4_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_benchmark.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_04",
            "usb_port": "2-3",
            "device_num": "4",
            "sync_mode": "standalone",
            "attach_to_shared_component_container": attach_to_shared_component_container_arg,
            "component_container_name": component_container_name_arg,
        }.items(),
    )

    delayed_left_camera = TimerAction(
        period=2.0,
        actions=args+[launch1_include],
    )
    delayed_right_camera = TimerAction(
        period=4.0,
        actions=args+[launch2_include],
    )
    delayed_rear_camera = TimerAction(
        period=6.0,
        actions=args+[launch3_include],
    )
    delayed_front_camera = TimerAction(
        period=8.0,
        actions=args+[launch4_include],
    )
    ld = LaunchDescription(
        [
            shared_orbbec_container,
            start_benchmark,
            delayed_left_camera,
            delayed_right_camera,
            delayed_rear_camera,
            delayed_front_camera,
        ]
    )

    return ld
