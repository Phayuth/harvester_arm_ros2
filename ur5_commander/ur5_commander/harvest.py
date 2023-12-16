import sys
sys.path.append("/home/yuth/ws_yuthdev/robotics_manipulator")

import numpy as np
np.random.seed(9)

from rigid_body_transformation.rigid_trans import RigidBodyTransformation as rbt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from harvest_bridge import HarvestBridge

class HarvesterClient(Node):

    def __init__(self):
        super().__init__('crop_planning')
        # name
        self.servTriggerPrePlan = 'trigger_pre_plan'
        self.servTriggerPlan = 'trigger_plan'
        self.servExemotionForward = 'trigger_execute_motion_forward'
        self.servExemotionReverse = 'trigger_execute_motion_reverse'
        self.topicCropHarvestPose = '/crop_harvest_pose'

        # callbackgroup
        self.callbackGroup = ReentrantCallbackGroup()

        # subscriber harvest pose 
        self.harvestPoseSubs = self.create_subscription(PoseArray, self.topicCropHarvestPose, self.harvest_pose_array_callback, 10, callback_group=self.callbackGroup)
        self.pickedGoalIndex = 10  # we have to score but for now just picked it
        self.pickedAuxIndex = self.pickedGoalIndex + 1

        self.currentJointsubscription = self.create_subscription(JointTrajectoryControllerState, '/joint_trajectory_controller/state', self.joint_state_callback, 10, callback_group=self.callbackGroup)
        self.ikClient = self.create_client(GetPositionIK, 'compute_ik', callback_group=self.callbackGroup)

        # plan and motion
        self.triggerPrePlanner = self.create_service(Trigger, self.servTriggerPrePlan, self.trigger_pre_plan, callback_group=self.callbackGroup)
        self.triggerPlanner = self.create_service(Trigger, self.servTriggerPlan, self.trigger_plan, callback_group=self.callbackGroup)
        self.triggerMoveForward = self.create_service(Trigger, self.servExemotionForward, self.trigger_execute_forward, callback_group=self.callbackGroup)
        self.triggerMoveReverse = self.create_service(Trigger, self.servExemotionReverse, self.trigger_execute_reverse, callback_group=self.callbackGroup)
        self.jointsName = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.trajPublisher = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 1)

        # publish virtual joint for view
        self.virtualStartJointPub = self.create_publisher(JointState, '/vs/joint_states', 10)
        self.virtualAuxJointPub = self.create_publisher(JointState, '/va/joint_states', 10)
        self.virtualGoalJointPub = self.create_publisher(JointState, '/vg/joint_states', 10)
        self.virtualJointPubTimer = self.create_timer(0.1, self.publish_joint_states)

        # constant value save
        self.harvestPose = None
        self.qCurrent = None
        self.qGoal = None
        self.qAux = None
        self.path = None

        # tf listener
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)
        self.fromFrameRel = 'camera_link'
        self.toFrameRel = 'base_link'

        self.get_logger().info(f'\ncrop planning initialized. \
                                 \ncall {self.servTriggerPrePlan} to do preplan \
                                 \ncall {self.servTriggerPlan} to plan motion. \
                                 \ncall {self.servExemotionForward} to execute robot forward motion. \
                                 \ncall {self.servExemotionReverse} to execute robot reverse motion')

    def joint_state_callback(self, msg):
        self.qCurrent = msg.actual.positions

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

    def publish_joint_states(self):
        now = self.get_clock().now().to_msg()

        if self.path is None:
            qC = self.qCurrent
            qA = self.qCurrent
            qG = self.qCurrent
        elif self.path is not None:
            qC = self.path[0].config
            qA = self.path[-2].config
            qG = self.path[-1].config

        jointMsgCurrent, jointMsgAux, jointMsgGoal = HarvestBridge.pub_virtual_state(now, qC, qA, qG)
        self.virtualStartJointPub.publish(jointMsgCurrent)
        self.virtualAuxJointPub.publish(jointMsgAux)
        self.virtualGoalJointPub.publish(jointMsgGoal)

    def get_transform_cam_to_base(self):
        try:
            now = rclpy.time.Time()
            trans = self.tfBuffer.lookup_transform(self.toFrameRel, self.fromFrameRel, now)
            translation = trans.transform.translation
            rotation = trans.transform.rotation

            tCamToBase = np.array([translation.x, translation.y, translation.z])
            RCamToBase = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
            TCamToBase = rbt.conv_t_and_quat_to_h(tCamToBase, RCamToBase)
            return TCamToBase
        except:
            pass

    def get_transform_point_to_cam(self, pickIndex):
        tPointToCam = np.array([self.harvestPose[pickIndex, 0], self.harvestPose[pickIndex, 1], self.harvestPose[pickIndex, 2]])
        RPointToCam = np.array([self.harvestPose[pickIndex, 3], self.harvestPose[pickIndex, 4], self.harvestPose[pickIndex, 5], self.harvestPose[pickIndex, 6]])
        TPointToCam = rbt.conv_t_and_quat_to_h(tPointToCam, RPointToCam)
        return TPointToCam


def main(args=None):
    rclpy.init(args=args)
    harvester = HarvesterClient()
    executor = MultiThreadedExecutor()
    rclpy.spin(harvester, executor)
    harvester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
