import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class PublisherJointTrajectory(Node):
    def __init__(self):
        super().__init__(node_name="publish_joint_trajectory_position_controller")
        self.controller_name = 'joint_trajectory_controller'
        self.wait_sec_between_publish = 6
        self.joints = ['shoulder_pan_joint',
                       'shoulder_lift_joint',
                       'elbow_joint',
                       'wrist_1_joint',
                       'wrist_2_joint',
                       'wrist_3_joint']

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')

        pos1 = [0.785, -1.57, 0.785, 0.785, 0.785, 0.785]
        pos2 = [0.0, -1.57, 0.0, 0.0, 0.0, 0.0]
        pos3 = [0.0, -1.57, 0.0, 0.0, -0.785, 0.0]
        pos4 = [0.0, -1.57, 0.0, 0.0, 0.0, 0.0]

        self.goals = [pos1, pos2, pos3, pos4]

        publish_topic = "/" + self.controller_name + "/" + "joint_trajectory"

        self.get_logger().info(f'Publishing goals on topic "{publish_topic}" every {self.wait_sec_between_publish} s')

        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.timer = self.create_timer(self.wait_sec_between_publish, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        traj = JointTrajectory()
        traj.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.goals[self.i]
        point.time_from_start = Duration(sec=4)

        traj.points.append(point)
        self.publisher_.publish(traj)

        self.i += 1
        self.i %= len(self.goals) # loop back to start index when finish the loop

def main(args=None):
    rclpy.init(args=args)

    publisher_joint_trajectory = PublisherJointTrajectory()

    rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
