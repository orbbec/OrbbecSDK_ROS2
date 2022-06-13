from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    params_config_path = (
        get_package_share_directory("orbbec_camera")
        + "/params/ob_camera_params.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="orbbec_camera",
                namespace="camera",
                name="camera",
                executable="orbbec_camera_node",
                output="screen",
                parameters=[params_config_path],
                prefix=["xterm -e gdb -ex run --args"],
            ),
        ]
    )
