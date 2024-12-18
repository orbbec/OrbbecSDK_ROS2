import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R
import sys
import os

def rotation_matrix_to_quaternion(rotation_matrix):
    r = R.from_matrix(rotation_matrix)
    q = r.as_quat()
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

def create_transform(matrix, parent_frame, child_frame):
    t = TransformStamped()
    t.header.stamp = rclpy.time.Time().to_msg()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame

    # Convert translation from mm to m
    t.transform.translation.x = matrix[0, 3] / 1000.0
    t.transform.translation.y = matrix[1, 3] / 1000.0
    t.transform.translation.z = matrix[2, 3] / 1000.0

    rotation_matrix = matrix[:3, :3]
    q = rotation_matrix_to_quaternion(rotation_matrix)

    t.transform.rotation = q

    return t

def convert_optical_to_vehicle_frame(optical_transform):
    # Conversion matrix from optical frame to vehicle frame
    conversion_matrix = np.array([
        [ 0,  0,  1,  0],
        [-1,  0,  0,  0],
        [ 0, -1,  0,  0],
        [ 0,  0,  0,  1]
    ])

    return np.linalg.multi_dot([conversion_matrix, optical_transform, np.linalg.inv(conversion_matrix)])

def load_yaml_to_matrices(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"YAML file not found: {file_path}")
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
        matrices = {
            key: np.array(value['matrix'])
            for key, value in data.items()
        }
    return matrices

class StaticTransformsPublisher(Node):
    def __init__(self, yaml_path):
        super().__init__('static_transforms_publisher')
        self._broadcaster = StaticTransformBroadcaster(self)

        # Load optical frame matrices from YAML file
        optical_matrices = load_yaml_to_matrices(yaml_path)

        # Identity transforms for other frames
        map_to_odom = np.eye(4)  # Assuming identity (no translation, no rotation)
        odom_to_base_link = np.eye(4)  # Assuming identity (no translation, no rotation)
        base_link_to_front_camera = np.eye(4)  # Assuming identity (no translation, no rotation)

        # Create and send static transform from 'odom' to 'base_link'
        # Create and send static transform from 'base_link' to 'front_camera_link'
        transforms = []
        transforms.append(create_transform(map_to_odom, 'map', 'odom'))
        transforms.append(create_transform(odom_to_base_link, 'odom', 'base_link'))
        transforms.append(create_transform(base_link_to_front_camera, 'base_link', 'front_camera_link'))

        # Convert optical frame matrices to vehicle frame
        vehicle_matrices = {k: convert_optical_to_vehicle_frame(v) for k, v in optical_matrices.items()}

        transforms.append(create_transform(vehicle_matrices['front_camera_link_to_left_camera_link'], 'front_camera_link', 'left_camera_link'))
        transforms.append(create_transform(vehicle_matrices['front_camera_link_to_right_camera_link'], 'front_camera_link', 'right_camera_link'))
        transforms.append(create_transform(vehicle_matrices['front_camera_link_to_rear_camera_link'], 'front_camera_link', 'rear_camera_link'))

        self._broadcaster.sendTransform(transforms)

def main():
    if len(sys.argv) < 2:
        print("Usage: python static_transforms_publisher.py <path_to_yaml>")
        sys.exit(1)

    yaml_path = sys.argv[1]

    rclpy.init()
    node = StaticTransformsPublisher(yaml_path)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
