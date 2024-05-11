import sys
sys.path.append("/home/yuth/ws_yuthdev/robotics_manipulator")

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
from harvest_bridge import HarvestBridge
from spatial_geometry.spatial_transformation import RigidBodyTransformation as rbt
from planner.planner_manipulator import PlannerManipulator
from simulator.sim_ur5e_api import UR5eArmCoppeliaSimAPI


class HarvesterClient(Node):

    def __init__(self):
        super().__init__('crop_planning')
        # name
        self.servTriggerPrePlan = 'trigger_pre_plan'
        self.servTriggerPlan = 'trigger_plan'
        self.servExemotionForward = 'trigger_execute_motion_forward'
        self.servExemotionReverse = 'trigger_execute_motion_reverse'
        self.topicCropHarvestPose = '/crop_harvest_pose'

        self.callbackGroup = ReentrantCallbackGroup()

        self.harvestPoseSubs = self.create_subscription(PoseArray, self.topicCropHarvestPose, self.harvest_pose_array_callback, 10, callback_group=self.callbackGroup)

        # plan and motion
        self.triggerPrePlanner = self.create_service(Trigger, self.servTriggerPrePlan, self.trigger_pre_plan, callback_group=self.callbackGroup)
        self.triggerPlanner = self.create_service(Trigger, self.servTriggerPlan, self.trigger_plan, callback_group=self.callbackGroup)
        self.triggerMoveForward = self.create_service(Trigger, self.servExemotionForward, self.trigger_execute_forward, callback_group=self.callbackGroup)
        self.triggerMoveReverse = self.create_service(Trigger, self.servExemotionReverse, self.trigger_execute_reverse, callback_group=self.callbackGroup)

        # constant value save
        self.harvestPose = None
        self.qCurrent = None
        self.qGoal = None
        self.qAux = None
        self.path = None

    def harvest_pose_array_callback(self, msg):
        self.harvestPose = HarvestBridge.get_harvest_pose(msg)

    def trigger_pre_plan(self, request, response):
        TCamToBase = self.get_transform_cam_to_base()
        TAuxToCam = self.get_transform_point_to_cam(self.pickedAuxIndex)
        TGoalToCam = self.get_transform_point_to_cam(self.pickedGoalIndex)
        self.qAux, self.qGoal, response = HarvestBridge.pre_plan_process(TCamToBase, TAuxToCam, TGoalToCam, self.ikClient, request, response)
        return response

    def trigger_plan(self, request, response):
        self.path, response = HarvestBridge.get_path_planning(self.qCurrent, self.qAux, self.qGoal, request, response)
        return response

    def trigger_execute_forward(self, request, response):
        trajMsg, response = HarvestBridge.srv_execute_motion_forward(self.path, self.jointsName, request, response)
        self.trajPublisher.publish(trajMsg)
        return response

    def trigger_execute_reverse(self, request, response):
        trajMsg, response = HarvestBridge.srv_execute_motion_reverse(self.path, self.jointsName, request, response)
        self.trajPublisher.publish(trajMsg)
        return response


def main(args=None):
    rclpy.init(args=args)
    harvester = HarvesterClient()
    executor = MultiThreadedExecutor()
    rclpy.spin(harvester, executor)
    harvester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
