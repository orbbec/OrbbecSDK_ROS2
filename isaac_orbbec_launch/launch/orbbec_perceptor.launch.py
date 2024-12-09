
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction
from launch.actions import GroupAction

def generate_launch_description():
    orbbec_camera_bringup_dir = get_package_share_directory('orbbec_camera')
    perceptor_bringup_dir = get_package_share_directory('isaac_ros_perceptor_bringup')

    perceptor_config_file = DeclareLaunchArgument(
        'perceptor_config_file', default_value='params/orbbec_perceptor_detached.yaml',
        description="Perceptor configuration")
    from_bag_arg = DeclareLaunchArgument(
        'from_bag', default_value='False',
        description='Whether to run from a bag or live realsense data')
    bag_path_arg = DeclareLaunchArgument(
        'bag_path', default_value='rosbag2*',
        description='Path of the bag (only used if from_bag == True)')

    component_container_name_arg = "shared_orbbec_container"
    shared_orbbec_container = Node(
        name=component_container_name_arg,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen')

    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            orbbec_camera_bringup_dir, 'launch', 'multi_camera_synced.launch.py')]),
        launch_arguments={
            'attach_to_shared_component_container': 'True',
            'component_container_name': component_container_name_arg}.items(),
        condition=UnlessCondition(LaunchConfiguration('from_bag')))

    perceptor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            perceptor_bringup_dir, 'launch', 'rgbd_perceptor.launch.py')]),
        launch_arguments={'config_file': LaunchConfiguration("perceptor_config_file"),
                          'attach_to_shared_component_container': 'True',
                          'component_container_name': component_container_name_arg}.items())
    delayed_perceptor_launch = TimerAction(
      period=0.5,
      actions=[
          GroupAction(actions=[perceptor_launch])
      ]
    )

    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path')],
        shell=True,
        output='screen',
        condition=IfCondition(LaunchConfiguration('from_bag')))

    return LaunchDescription([
        perceptor_config_file,
        from_bag_arg,
        bag_path_arg,
        shared_orbbec_container,
        orbbec_launch,
        perceptor_launch,
        bag_play,
      ]
    )
