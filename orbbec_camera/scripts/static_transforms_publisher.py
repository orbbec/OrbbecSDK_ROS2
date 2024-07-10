import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
import numpy as np
from scipy.spatial.transform import Rotation as R

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

class StaticTransformsPublisher(Node):
    def __init__(self):
        super().__init__('static_transforms_publisher')
        self._broadcaster = StaticTransformBroadcaster(self)

        optical_matrices = {
            'front_camera_link_to_left_camera_link': np.array([
                [-0.0117891, 0.0022117, -0.999928, -27.6039],
                [0.0052539, 0.999984, 0.00214988, 0.548432],
                [0.999917, -0.00522818, -0.0118005, -125.997],
                [0, 0, 0, 1]
            ]),
            'front_camera_link_to_right_camera_link': np.array([
                [-0.00612923, -0.00990235, 0.999932, 126.278],
                [0.00831223, 0.999916, 0.00995314, -0.0925262],
                [-0.999947, 0.00837267, -0.0060464, -31.6458],
                [0, 0, 0, 1]
            ]),
            'rear_camera_link_to_right_camera_link': np.array([
                [-0.00801505, -0.00150606, 0.999967, 30.108],
                [0.0144913, 0.999894, 0.0016221, 1.223],
                [-0.999863, 0.0145038, -0.00799237, 125.246],
                [0, 0, 0, 1]
            ])
        }

        # Convert optical frame matrices to vehicle frame
        vehicle_matrices = {k: convert_optical_to_vehicle_frame(v) for k, v in optical_matrices.items()}

        transforms = []
        transforms.append(create_transform(vehicle_matrices['front_camera_link_to_left_camera_link'], 'front_camera_link', 'left_camera_link'))
        transforms.append(create_transform(vehicle_matrices['front_camera_link_to_right_camera_link'], 'front_camera_link', 'right_camera_link'))
        
        # Right to rear camera transform
        right_to_rear = vehicle_matrices['rear_camera_link_to_right_camera_link']
        
        # Invert the matrix to get right_camera_link to rear_camera_link
        right_to_rear = np.linalg.inv(right_to_rear)
        
        # Apply 180-degree rotation around Z-axis to align coordinate systems
        rotation_180 = np.array([
            [-1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        right_to_rear = np.dot(right_to_rear, rotation_180)
        
        transforms.append(create_transform(right_to_rear, 'right_camera_link', 'rear_camera_link'))

        print("Right to Rear Transform:")
        print(right_to_rear)
        

        self._broadcaster.sendTransform(transforms)
        

def main():
    rclpy.init()
    node = StaticTransformsPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()