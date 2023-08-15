from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_camera',
            namespace='crop_perception',
            executable='perception_tf',
        ),
        Node(
            package='ros_camera',
            namespace='crop_perception',
            executable='crop_localize',
        ),
        Node(
            package='ros_camera',
            namespace='crop_perception',
            executable='crop_harvest_pose',
        ),
    ])