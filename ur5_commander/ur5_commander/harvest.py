import sys
sys.path.append("/home/yuth/ws_yuthdev/robotics_manipulator")

import numpy as np
np.random.seed(9)

from planner_dev.copsim6d.copsim_rrt_single import RRTBaseCopSim, RRTConnectCopSim, RRTStarCopSim, RRTInformedCopSim, RRTStarConnectCopSim, RRTInformedConnectCopSim, RRTConnectAstInformedCopSim, RRTStarLocalOptCopSim, RRTConnectLocalOptCopSim
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Event
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R


class HarvesterClient(Node):

    def __init__(self):
        super().__init__('crop_planning')
        # name
        self.servTriggerGetCurrent = 'trigger_current_q'
        self.servTriggerGetAux = 'trigger_aux_q'
        self.servTriggerGetGoal = 'trigger_goal_q'
        self.servTriggerPlan = 'trigger_plan'
        self.servExemotionForward = 'trigger_execute_motion_forward'
        self.servExemotionReverse = 'trigger_execute_motion_reverse'
        self.topicCropHarvestPose = '/crop_harvest_pose'

        self.service_done_event = Event()
        self.callbackGroup = ReentrantCallbackGroup()

        # subscriber harvest pose 
        self.harvestPoseSubs = self.create_subscription(PoseArray, self.topicCropHarvestPose, self.harvest_pose_array_callback, 10, callback_group=self.callbackGroup)
        self.pickedGoalIndex = 10  # we have to score but for now just picked it
        self.pickedAuxIndex = self.pickedGoalIndex + 1

        # subscriber to current joint position with trigger to update current joint
        self.currentJointsubscription = self.create_subscription(JointTrajectoryControllerState, '/joint_trajectory_controller/state', self.joint_state_callback, 10, callback_group=self.callbackGroup)
        self.triggerCurrent = self.create_service(Trigger, self.servTriggerGetCurrent, self.trigger_update_current_joint_callback)
        self.joint = None

        # subscriber to goal and aux position with trigger to update and perform IK
        self.triggerAux = self.create_service(Trigger, self.servTriggerGetAux, self.trigger_update_aux_callback, callback_group=self.callbackGroup)
        self.triggerGoal = self.create_service(Trigger, self.servTriggerGetGoal, self.trigger_update_goal_callback, callback_group=self.callbackGroup)
        self.ikClient = self.create_client(GetPositionIK, 'compute_ik', callback_group=self.callbackGroup)

        # publish joint sequence after planning
        self.jointsName = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.trajPublisher = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 1)
        self.triggerPlanner = self.create_service(Trigger, self.servTriggerPlan, self.trigger_plan, callback_group=self.callbackGroup)
        self.triggerMoveForward = self.create_service(Trigger, self.servExemotionForward, self.trigger_execute_forward, callback_group=self.callbackGroup)
        self.triggerMoveReverse = self.create_service(Trigger, self.servExemotionReverse, self.trigger_execute_reverse, callback_group=self.callbackGroup)

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
                               \ncall {self.servTriggerGetCurrent} to update current joint. \
                               \ncall {self.servTriggerGetAux} to update auxilary joint. \
                               \ncall {self.servTriggerGetGoal} to update goal joint. \
                               \ncall {self.servTriggerPlan} to plan motion. \
                               \ncall {self.servExemotionForward} to execute robot forward motion. \
                               \ncall {self.servExemotionReverse} to execute robot reverse motion')

    def trigger_update_current_joint_callback(self, request, response):
        if request:
            self.qCurrent = self.joint
            response.success = True
            response.message = f"current joint value called success qcurrent = {str(self.qCurrent)}"
        else:
            response.success = False
            response.message = "invalid trigger request"
        return response

    def trigger_update_aux_callback(self, request, response):
        if not self.ikClient.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('No action server available')

        self.service_done_event.clear()
        event = Event()

        def done_callback(future):
            nonlocal event
            event.set()

        TCamToBase = self.get_transform_cam_to_base()
        TPointToCam = self.get_transform_point_to_cam(self.pickedAuxIndex)
        TPointToBase = TCamToBase @ TPointToCam
        tPointToBase, qPointToBase = self.transformation_to_quaternion_and_translation(TPointToBase)

        reqIKAux = self.compose_ik_request(tPointToBase, qPointToBase)
        futureAux = self.ikClient.call_async(reqIKAux)
        futureAux.add_done_callback(done_callback)
        event.wait()
        resAux = futureAux.result()

        self.qAux = resAux.solution.joint_state.position
        response.success = True
        response.message = f"auxilary joint value called success qaux = {str(self.qAux)}"
        return response

    def trigger_update_goal_callback(self, request, response):
        if not self.ikClient.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('No action server available')

        self.service_done_event.clear()
        event = Event()

        def done_callback(future):
            nonlocal event
            event.set()

        TCamToBase = self.get_transform_cam_to_base()
        TPointToCam = self.get_transform_point_to_cam(self.pickedGoalIndex)
        TPointToBase = TCamToBase @ TPointToCam
        tPointToBase, qPointToBase = self.transformation_to_quaternion_and_translation(TPointToBase)

        reqIKGoal = self.compose_ik_request(tPointToBase, qPointToBase)
        futureGoal = self.ikClient.call_async(reqIKGoal)
        futureGoal.add_done_callback(done_callback)
        event.wait()
        resGoal = futureGoal.result()

        self.qGoal = resGoal.solution.joint_state.position
        response.success = True
        response.message = f"goal joint value called success qgoal = {str(self.qGoal)}"
        return response

    def compose_ik_request(self, trans, quat):
        reqIK = GetPositionIK.Request()
        reqIK.ik_request.group_name = 'ur_manipulator'
        reqIK.ik_request.pose_stamped.header.frame_id = ''
        reqIK.ik_request.pose_stamped.pose.position.x = trans[0]
        reqIK.ik_request.pose_stamped.pose.position.y = trans[1]
        reqIK.ik_request.pose_stamped.pose.position.z = trans[2]
        reqIK.ik_request.pose_stamped.pose.orientation.x = quat[0]
        reqIK.ik_request.pose_stamped.pose.orientation.y = quat[1]
        reqIK.ik_request.pose_stamped.pose.orientation.z = quat[2]
        reqIK.ik_request.pose_stamped.pose.orientation.w = quat[3]
        return reqIK
    
    def joint_state_callback(self, msg):
        joint_positions = msg.actual.positions
        self.joint = joint_positions

    def harvest_pose_array_callback(self, msg):
        poses = msg.poses
        numPoses = len(poses)
        if numPoses != 0:
            poseList = np.zeros((numPoses, 7))
            for i, pose in enumerate(poses):
                poseList[i, 0] = pose.position.x
                poseList[i, 1] = pose.position.y
                poseList[i, 2] = pose.position.z
                poseList[i, 3] = pose.orientation.x
                poseList[i, 4] = pose.orientation.y
                poseList[i, 5] = pose.orientation.z
                poseList[i, 6] = pose.orientation.w

            self.harvestPose = poseList

    def trigger_plan(self, request, response):
        qCurrent = np.array([self.qCurrent[0], self.qCurrent[1], self.qCurrent[2], self.qCurrent[3], self.qCurrent[4], self.qCurrent[5]]).reshape(6, 1)
        qAux = np.array([self.qAux[0], self.qAux[1], self.qAux[2], self.qAux[3], self.qAux[4], self.qAux[5]]).reshape(6, 1)
        qGoal = np.array([self.qGoal[0], self.qGoal[1], self.qGoal[2], self.qGoal[3], self.qGoal[4], self.qGoal[5]]).reshape(6, 1)

        # planner = RRTBaseCopSim(qCurrent, qAux, qGoal, eta=0.1, maxIteration=3000, numDoF=6)
        # planner = RRTConnectCopSim(qCurrent, qAux, qGoal, eta=0.1, maxIteration=3000, numDoF=6)
        # planner = RRTStarCopSim(qCurrent, qAux, qGoal, eta=0.1, maxIteration=3000, numDoF=6)
        planner = RRTInformedCopSim(qCurrent, self.wrap_to_pi(qAux), self.wrap_to_pi(qGoal), eta=0.1, maxIteration=3000, numDoF=6)
        # planner = RRTStarConnectCopSim(qCurrent, qAux, qGoal, eta=0.1, maxIteration=3000, numDoF=6)
        # planner = RRTInformedConnectCopSim(qCurrent, qAux, qGoal, eta=0.1, maxIteration=3000, numDoF=6)
        # planner = RRTConnectAstInformedCopSim(qCurrent, qAux, qGoal, eta=0.1, maxIteration=3000, numDoF=6)
        # planner = RRTStarLocalOptCopSim(qCurrent, qAux, qGoal, eta=0.1, maxIteration=3000, numDoF=6)
        # planner = RRTConnectLocalOptCopSim(qCurrent, qAux, qGoal, eta=0.1, maxIteration=3000, numDoF=6)
        self.path = planner.planning()

        response.success = True
        response.message = f"trajectory ready. call trigger motion to publish. call {self.servExemotionForward} or {self.servExemotionReverse}."
        return response

    def trigger_execute_forward(self, request, response):
        self.traj = JointTrajectory()
        self.traj.joint_names = self.jointsName
        for i in range(len(self.path)):
            point = JointTrajectoryPoint()
            point.positions = [self.path[i][0, 0], 
                               self.path[i][1, 0], 
                               self.path[i][2, 0], 
                               self.path[i][3, 0], 
                               self.path[i][4, 0], 
                               self.path[i][5, 0]]
            point.time_from_start = Duration(sec=1 + i, nanosec=0)
            self.traj.points.append(point)

        self.trajPublisher.publish(self.traj)
        response.success = True
        response.message = "Robot motion is active now"
        return response

    def trigger_execute_reverse(self, request, response):
        self.traj = JointTrajectory()
        self.traj.joint_names = self.jointsName
        for i in range(len(self.path)):
            point = JointTrajectoryPoint()
            point.positions = [self.path[(i * -1) - 1][0, 0], 
                               self.path[(i * -1) - 1][1, 0], 
                               self.path[(i * -1) - 1][2, 0], 
                               self.path[(i * -1) - 1][3, 0], 
                               self.path[(i * -1) - 1][4, 0], 
                               self.path[(i * -1) - 1][5, 0]]
            point.time_from_start = Duration(sec=1 + i, nanosec=0)
            self.traj.points.append(point)

        self.trajPublisher.publish(self.traj)
        response.success = True
        response.message = "Robot motion is active now"
        return response

    def publish_joint_states(self):
        now = self.get_clock().now().to_msg()

        if self.qCurrent is None:
            qC = self.joint
        elif self.qCurrent is not None:
            qC = self.qCurrent
        joint_state_msg1 = JointState()
        joint_state_msg1.header.stamp = now
        joint_state_msg1.name = ['vs_shoulder_pan_joint', 'vs_shoulder_lift_joint', 'vs_elbow_joint', 'vs_wrist_1_joint', 'vs_wrist_2_joint', 'vs_wrist_3_joint']
        joint_state_msg1.position = [qC[0], qC[1], qC[2], qC[3], qC[4], qC[5]]

        if self.qAux is None:
            qA = self.joint
        elif self.qAux is not None:
            qA = self.qAux
        joint_state_msg2 = JointState()
        joint_state_msg2.header.stamp = now
        joint_state_msg2.name = ['va_shoulder_pan_joint', 'va_shoulder_lift_joint', 'va_elbow_joint', 'va_wrist_1_joint', 'va_wrist_2_joint', 'va_wrist_3_joint']
        joint_state_msg2.position = [qA[0], qA[1], qA[2], qA[3], qA[4], qA[5]]

        if self.qGoal is None:
            qG = self.joint
        elif self.qGoal is not None:
            qG = self.qGoal
        joint_state_msg3 = JointState()
        joint_state_msg3.header.stamp = now
        joint_state_msg3.name = ['vg_shoulder_pan_joint', 'vg_shoulder_lift_joint', 'vg_elbow_joint', 'vg_wrist_1_joint', 'vg_wrist_2_joint', 'vg_wrist_3_joint']
        joint_state_msg3.position = [qG[0], qG[1], qG[2], qG[3], qG[4], qG[5]]

        self.virtualAuxJointPub.publish(joint_state_msg2)
        self.virtualStartJointPub.publish(joint_state_msg1)
        self.virtualGoalJointPub.publish(joint_state_msg3)

    def get_transform_cam_to_base(self):
        try:
            now = rclpy.time.Time()
            trans = self.tfBuffer.lookup_transform(self.toFrameRel, self.fromFrameRel, now)
            translation = trans.transform.translation
            rotation = trans.transform.rotation

            tCamToBase = np.array([translation.x, translation.y, translation.z])
            RCamToBase = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
            TCamToBase = self.compose_transformation_matrix(tCamToBase, RCamToBase)
            return TCamToBase
        except:
            pass

    def get_transform_point_to_cam(self, pickIndex):
        tPointToCam = np.array([self.harvestPose[pickIndex, 0], self.harvestPose[pickIndex, 1], self.harvestPose[pickIndex, 2]])
        RPointToCam = np.array([self.harvestPose[pickIndex, 3], self.harvestPose[pickIndex, 4], self.harvestPose[pickIndex, 5], self.harvestPose[pickIndex, 6]])
        TPointToCam = self.compose_transformation_matrix(tPointToCam, RPointToCam)
        return TPointToCam
    
    def wrap_to_pi(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def compose_transformation_matrix(self, translation, rotation):
        rot = R.from_quat(rotation)
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rot.as_matrix()
        transformation_matrix[:3, 3] = translation
        return transformation_matrix

    def transformation_to_quaternion_and_translation(self, transformation_matrix):
        rotation_matrix = transformation_matrix[:3, :3]
        rotation = R.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()
        translation = transformation_matrix[:3, 3]
        return translation, quaternion


def main(args=None):
    rclpy.init(args=args)
    harvester = HarvesterClient()
    executor = MultiThreadedExecutor()
    rclpy.spin(harvester, executor)
    harvester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
