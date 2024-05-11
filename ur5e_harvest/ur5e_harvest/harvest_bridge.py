import sys
sys.path.append("/home/yuth/ws_yuthdev/robotics_manipulator")

import numpy as np
np.random.seed(9)

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK

from planner.planner_manipulator import PlannerManipulator
from spatial_geometry.spatial_transformation import RigidBodyTransformation as rbt
from simulator.sim_ur5e_api import UR5eArmCoppeliaSimAPI


class HarvestBridge:

    def compose_ik_request(trans, quat):
        reqIKMsg = GetPositionIK.Request()
        reqIKMsg.ik_request.group_name = 'ur_manipulator'
        reqIKMsg.ik_request.pose_stamped.header.frame_id = ''
        reqIKMsg.ik_request.pose_stamped.pose.position.x = trans[0]
        reqIKMsg.ik_request.pose_stamped.pose.position.y = trans[1]
        reqIKMsg.ik_request.pose_stamped.pose.position.z = trans[2]
        reqIKMsg.ik_request.pose_stamped.pose.orientation.x = quat[0]
        reqIKMsg.ik_request.pose_stamped.pose.orientation.y = quat[1]
        reqIKMsg.ik_request.pose_stamped.pose.orientation.z = quat[2]
        reqIKMsg.ik_request.pose_stamped.pose.orientation.w = quat[3]
        return reqIKMsg

    def get_ik_solution(ikClient, H):
        t, quat = rbt.conv_h_to_t_and_quat(H)
        reqIKMsg = HarvestBridge.compose_ik_request(t, quat)
        future = ikClient.call_async(reqIKMsg)
        res = future.result()
        q = res.solution.joint_state.position
        return q

    def pre_plan_process(TCamToBase, TAuxToCam, TGoalToCam, ikClient, request, response):
        TAuxToBase = TCamToBase @ TAuxToCam
        qAux = HarvestBridge.get_ik_solution(ikClient, TAuxToBase)
        TGoalToBase = TCamToBase @ TGoalToCam
        qGoal = HarvestBridge.get_ik_solution(ikClient, TGoalToBase)

        response.success = True
        response.message = f"done pre plan"
        return qAux, qGoal, response

    def get_harvest_pose(msg):
        poses = msg.poses
        numPoses = len(poses)
        if numPoses != 0:
            harvestPoses = np.zeros((numPoses, 7))
            for i, pose in enumerate(poses):
                harvestPoses[i, 0] = pose.position.x
                harvestPoses[i, 1] = pose.position.y
                harvestPoses[i, 2] = pose.position.z
                harvestPoses[i, 3] = pose.orientation.x
                harvestPoses[i, 4] = pose.orientation.y
                harvestPoses[i, 5] = pose.orientation.z
                harvestPoses[i, 6] = pose.orientation.w
        return harvestPoses

    def get_path_planning(qCurrent, qAux, qGoal, request, response):

        copsimConfigDualTree = {
            "planner": 5,
            "eta": 0.15,
            "subEta": 0.05,
            "maxIteration": 1500,
            "robotEnvClass": UR5eArmCoppeliaSimAPI,
            "nearGoalRadius": None,
            "rewireRadius": None,
            "endIterationID": 1,
            "print_debug": True,
            "localOptEnable": True
        }

        qCurrent = np.array([qCurrent[0], qCurrent[1], qCurrent[2], qCurrent[3], qCurrent[4], qCurrent[5]]).reshape(6, 1)
        qAux = np.array([qAux[0], qAux[1], qAux[2], qAux[3], qAux[4], qAux[5]]).reshape(6, 1)
        qGoal = np.array([qGoal[0], qGoal[1], qGoal[2], qGoal[3], qGoal[4], qGoal[5]]).reshape(6, 1)

        pa = PlannerManipulator(qCurrent, qAux, qGoal, copsimConfigDualTree)
        path = pa.planning()

        response.success = True
        response.message = f"path sequences : {path}"
        return path, response

    def srv_execute_motion_forward(path, jointName, request, response):
        trajMsg = JointTrajectory()
        trajMsg.joint_names = jointName
        for i in range(len(path)):
            point = JointTrajectoryPoint()
            point.positions = path[i].config.tolist()
            point.time_from_start = Duration(sec=1 + i, nanosec=0)
            trajMsg.points.append(point)

        response.success = True
        response.message = "Robot motion is active now"
        return trajMsg, response

    def srv_execute_motion_reverse(path, jointName, request, response):
        trajMsg = JointTrajectory()
        trajMsg.joint_names = jointName
        for i in range(len(path)):
            point = JointTrajectoryPoint()
            point.positions = path[(i * -1) - 1].config.tolist()
            point.time_from_start = Duration(sec=1 + i, nanosec=0)
            trajMsg.points.append(point)

        response.success = True
        response.message = "Robot motion is active now"
        return trajMsg, response

    def pub_virtual_state(timeNow, qCurrent, qAux, qGoal):
        jointMsgCurrent = JointState()
        jointMsgCurrent.header.stamp = timeNow
        jointMsgCurrent.name = ['vs_shoulder_pan_joint', 'vs_shoulder_lift_joint', 'vs_elbow_joint', 'vs_wrist_1_joint', 'vs_wrist_2_joint', 'vs_wrist_3_joint']
        jointMsgCurrent.position = qCurrent

        jointMsgAux = JointState()
        jointMsgAux.header.stamp = timeNow
        jointMsgAux.name = ['va_shoulder_pan_joint', 'va_shoulder_lift_joint', 'va_elbow_joint', 'va_wrist_1_joint', 'va_wrist_2_joint', 'va_wrist_3_joint']
        jointMsgAux.position = qAux

        jointMsgGoal = JointState()
        jointMsgGoal.header.stamp = timeNow
        jointMsgGoal.name = ['vg_shoulder_pan_joint', 'vg_shoulder_lift_joint', 'vg_elbow_joint', 'vg_wrist_1_joint', 'vg_wrist_2_joint', 'vg_wrist_3_joint']
        jointMsgGoal.position = qGoal

        return jointMsgCurrent, jointMsgAux, jointMsgGoal

