#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def main():
    rclpy.init(args=None)
    node = rclpy.create_node('test_action_client')

    action_client = ActionClient(node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

    if not action_client.wait_for_server(timeout_sec=2.0):
        node.get_logger().error('Action server not available')
        rclpy.shutdown()
        return

    node.get_logger().info('Action server available')

    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    joint_trajectory_point = JointTrajectoryPoint()
    joint_trajectory_point.positions = [1.0, -4.0, -0.5, 0.0, 0.0, 0.0]  # joint position
    joint_trajectory_point.velocities = [1.0] * 6  # joint velocity
    joint_trajectory_point.time_from_start = rclpy.duration.Duration(seconds=3.0).to_msg()
    joint_trajectory.points.append(joint_trajectory_point)

    follow_joint_traj_goal = FollowJointTrajectory.Goal()
    follow_joint_traj_goal.trajectory = joint_trajectory

    future = action_client.send_goal_async(follow_joint_traj_goal)

    while rclpy.ok():
        rclpy.spin_once(node)
        if future.done():
            status = future.result().status
            if status == 2:
                node.get_logger().info('Action succeeded')
            else:
                node.get_logger().info('Action failed with status: ')
                node.get_logger().info(str(status))
            break

    action_client.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
