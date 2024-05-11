import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = []
    ld.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.0", "0.0", "0.0", "-1.5708", "0.0", "0.0", "Tool_Center_Point", "gripper_adapter_link"],
        )
    )

    ld.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.02151967", "-0.07698767", "0.08906624", "0.04974107", "0.12450784", "-0.68294573", "0.71805902", "Tool_Center_Point", "camera_link"],
        )
    )

    return LaunchDescription(ld)