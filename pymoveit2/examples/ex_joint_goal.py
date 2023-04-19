#!/usr/bin/env python3
"""
Example of moving to a joint configuration.
`ros2 run pymoveit2 ex_joint_goal.py --ros-args -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57, 0.7854]"`
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import panda
from pymoveit2.robots import ur5


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_joint_goal")

    # Declare parameter for joint positions
    node.declare_parameter(
        "joint_positions",
        [
            0.0,
            0.0,
            0.0,
            -0.7853981633974483,
            0.0,
            1.5707963267948966,
        ],
    )

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface Panda
    # moveit2 = MoveIt2(
    #     node=node,
    #     joint_names=panda.joint_names(),
    #     base_link_name=panda.base_link_name(),
    #     end_effector_name=panda.end_effector_name(),
    #     group_name=panda.MOVE_GROUP_ARM,
    #     callback_group=callback_group,
    #     follow_joint_trajectory_action_name= "/panda_arm_controller/follow_joint_trajectory",
    # )

    # Create MoveIt 2 interface UR5
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
        follow_joint_trajectory_action_name= "/joint_trajectory_controller/follow_joint_trajectory",
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameter
    joint_positions = (
        node.get_parameter("joint_positions").get_parameter_value().double_array_value
    )

    # Move to joint configuration
    node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")
    moveit2.move_to_configuration(joint_positions)
    moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
