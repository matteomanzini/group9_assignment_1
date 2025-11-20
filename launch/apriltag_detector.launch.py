from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_detector',
            remappings=[
                # Usa la camera del robot
                ('image_rect', '/rgb_camera/image'),
                ('camera_info', '/rgb_camera/camera_info'),
            ],
            parameters=[{
                'family': '36h11',
                'size': 0.050,  # 5cm come da assignment
                'max_hamming': 0,
            }]
        )
    ])