from launch import LaunchDescription
import launch_ros.actions
from ament_index_python import get_package_share_directory


def generate_launch_description():
    ob_params_file1 = (
        get_package_share_directory("orbbec_camera")
        + "/params/multi_camera/ob_camera1_params.yaml"
    )
    ob_params_file2 = (
        get_package_share_directory("orbbec_camera")
        + "/params/multi_camera/ob_camera2_params.yaml"
    )
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="orbbec_camera",
                namespace="camera1",
                name="camera1",
                executable="orbbec_camera_node",
                output="screen",
                parameters=[ob_params_file1],
            ),
            launch_ros.actions.Node(
                package="orbbec_camera",
                namespace="camera2",
                name="camera2",
                executable="orbbec_camera_node",
                output="screen",
                parameters=[ob_params_file2],
            ),
            # dummy static transformation from camera1 to camera2
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "camera1_link",
                    "camera2_link",
                ],
            ),
        ]
    )
