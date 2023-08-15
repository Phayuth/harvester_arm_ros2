import sys
sys.path.append("/home/yuth/ws_yuthdev/robotics_manipulator")

import time
import numpy as np
np.random.seed(9)
# from planner_dev.copsim_rrt_star import RRTStarDev
from planner_dev.copsim_rrt_informed import RRTInformedDev
from planner_util.extract_path_class import extract_path_class_6d
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from rclpy.time import Time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PublisherJointTrajectory(Node):
    def __init__(self):
        super().__init__(node_name="publish_joint_trajectory_position_controller")
        self.controller_name = 'joint_trajectory_controller'
        self.joints = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
        publish_topic = "/" + self.controller_name + "/" + "joint_trajectory"
        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)

        # UR5e
        thetaInit = np.array([np.deg2rad(-27.68), np.deg2rad(2.95), np.deg2rad(-26.58), np.deg2rad(-15.28), np.deg2rad(87.43), np.deg2rad(0.0)]).reshape(6, 1)
        thetaGoal = np.array([np.deg2rad(124.02), np.deg2rad(-70.58), np.deg2rad(-91.21), np.deg2rad(-50.84), np.deg2rad(105.67), np.deg2rad(-0.02)]).reshape(6, 1)
        thetaApp = np.array([np.deg2rad(124.02), np.deg2rad(-56.47), np.deg2rad(-91.21), np.deg2rad(-59.58), np.deg2rad(103.13), np.deg2rad(-0.08)]).reshape(6, 1)

        planner = RRTInformedDev(thetaInit, thetaApp, thetaGoal, eta=0.3, maxIteration=100)
        path = planner.planning()
        print(f"==>> path: \n{path}")

        time.sleep(3)

        pathX, pathY, pathZ, pathP, pathQ, pathR = extract_path_class_6d(path)
        self.pathX = np.array(pathX)
        self.pathY = np.array(pathY)
        self.pathZ = np.array(pathZ)
        self.pathP = np.array(pathP)
        self.pathQ = np.array(pathQ)
        self.pathR = np.array(pathR)

        timeStep = 0.7

        self.traj = JointTrajectory()
        self.traj.joint_names = self.joints
        for i in range(len(self.pathX)):
            point = JointTrajectoryPoint()
            point.positions = [self.pathX[i], 
                            self.pathY[i], 
                            self.pathZ[i], 
                            self.pathP[i], 
                            self.pathQ[i], 
                            self.pathR[i]]
            point.time_from_start = Duration(sec=1+i, nanosec=0)
            self.traj.points.append(point)

        self.publisher_.publish(self.traj)

def main(args=None):

    rclpy.init(args=args)
    publisher_joint_trajectory = PublisherJointTrajectory()
    # rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
