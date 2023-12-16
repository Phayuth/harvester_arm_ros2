from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='onrobotsg',
            # namespace='',
            executable='onrobotsg_service',
            name='gripper'
        ),
    ])