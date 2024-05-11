from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = []
    ld.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["-0.01692856", "-0.0914942", "0.02279194", "0.51734014", "-0.49873382", "0.49523948", "0.48822291", "tool0", "camera_link"],
        )
    )

    return LaunchDescription(ld)

# -0.01692856 -0.0914942 0.02279194 0.51734014 -0.49873382 0.49523948 0.48822291 tool0 camera_link