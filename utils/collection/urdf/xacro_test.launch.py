from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    arg1_value = "john"
    arg2_value = "cena"

    robot_description_content = Command([
        'xacro',
        ' ',
        'example_macro.urdf.xacro',
        ' ',
        'arg1_input:={}'.format(arg1_value),
        ' ',
        'arg2_input:={}'.format(arg2_value)
    ]) # run terminal command

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    robot_joint_state_pub_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="both",
    )

    rvizer = Node(
        package="rviz2",
        executable="rviz2",
        output="both",
    )

    return LaunchDescription([
        robot_state_publisher_node,
        robot_joint_state_pub_gui,
        rvizer
    ])
