import sys

sys.path.append("/home/yuth/ws_yuthdev/robotics_manipulator")
import numpy as np

np.random.seed(9)

# from planner_dev.copsim_rrt_star import RRTStarDev
from planner_dev.copsim_rrt_informed import RRTInformedDev
from planner_util.extract_path_class import extract_path_class_6d
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Event
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState


class HarvesterClient(Node):

    def __init__(self):
        super().__init__('crop_planning')
        self.service_done_event = Event()
        self.callback_group = ReentrantCallbackGroup()

        # subscriber to current joint position with trigger to update current joint
        self.currentJointsubscription = self.create_subscription(JointTrajectoryControllerState, '/joint_trajectory_controller/state', self.joint_state_callback, 10, callback_group=self.callback_group)
        self.triggerCurrent = self.create_service(Trigger, 'trigger_current_q', self.trigger_update_current_joint_callback)
        self.joint = None
        self.qCurrent = None

        # subscriber to goal and aux position with trigger to update and perform IK
        self.goalPoseSubs = self.create_subscription(PoseStamped, '/crop_harvest_goal_picked', self.goal_pose_array_callback, 10, callback_group=self.callback_group)
        self.auxiPoseSubs = self.create_subscription(PoseStamped, '/crop_harvest_aux_picked', self.auxi_pose_array_callback, 10, callback_group=self.callback_group)
        self.triggerAux = self.create_service(Trigger, 'trigger_aux_q', self.trigger_update_aux_callback, callback_group=self.callback_group)
        self.triggerGoal = self.create_service(Trigger, 'trigger_goal_q', self.trigger_update_goal_callback, callback_group=self.callback_group)
        self.goalPose = None
        self.auxPose = None
        self.qGoal = None
        self.qAux = None
        self.client = self.create_client(GetPositionIK, 'compute_ik', callback_group=self.callback_group)

        # publish joint sequence after planning
        self.jointsName = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.trajPublisher = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 1)
        self.triggerPlanner = self.create_service(Trigger, 'trigger_plan', self.trigger_plan, callback_group=self.callback_group)
        self.triggerMoveForward = self.create_service(Trigger, 'trigger_execute_motion_forward', self.trigger_execute_forward, callback_group=self.callback_group)
        self.triggerMoveReverse = self.create_service(Trigger, 'trigger_execute_motion_reverse', self.trigger_execute_reverse, callback_group=self.callback_group)

        # publish virtual joint for view
        self.publisher_joint1 = self.create_publisher(JointState, '/vs/joint_states', 10)
        self.publisher_joint2 = self.create_publisher(JointState, '/va/joint_states', 10)
        self.publisher_joint3 = self.create_publisher(JointState, '/vg/joint_states', 10)
        self.timer_ = self.create_timer(0.1, self.publish_joint_states)

        self.get_logger().info('Initialize Ready')

    def trigger_update_current_joint_callback(self, request, response):
        if request:
            self.qCurrent = self.joint
            print(f"==>> self.qCurrent: \n{self.qCurrent}")
            response.success = True
            response.message = str(self.qCurrent)
        else:
            response.success = False
            response.message = "Invalid trigger request."
        return response

    def trigger_update_aux_callback(self, request, response):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('No action server available')

        self.service_done_event.clear()
        event = Event()

        def done_callback(future):
            nonlocal event
            event.set()

        reqIKAux = GetPositionIK.Request()
        reqIKAux.ik_request.group_name = 'ur_manipulator'
        reqIKAux.ik_request.pose_stamped.header.frame_id = ''
        reqIKAux.ik_request.pose_stamped.pose.position.x = self.auxPose[0]
        reqIKAux.ik_request.pose_stamped.pose.position.y = self.auxPose[1]
        reqIKAux.ik_request.pose_stamped.pose.position.z = self.auxPose[2]
        reqIKAux.ik_request.pose_stamped.pose.orientation.x = self.auxPose[3]
        reqIKAux.ik_request.pose_stamped.pose.orientation.y = self.auxPose[4]
        reqIKAux.ik_request.pose_stamped.pose.orientation.z = self.auxPose[5]
        reqIKAux.ik_request.pose_stamped.pose.orientation.w = self.auxPose[6]
        futureAux = self.client.call_async(reqIKAux)
        futureAux.add_done_callback(done_callback)
        event.wait()
        resAux = futureAux.result()

        self.qAux = resAux.solution.joint_state.position
        print(f"==>> self.qAux: \n{self.qAux}")

        response.success = True
        response.message = f"Done Call"
        return response

    def trigger_update_goal_callback(self, request, response):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('No action server available')

        self.service_done_event.clear()
        event = Event()

        def done_callback(future):
            nonlocal event
            event.set()

        reqIKGoal = GetPositionIK.Request()
        reqIKGoal.ik_request.group_name = 'ur_manipulator'
        reqIKGoal.ik_request.pose_stamped.header.frame_id = ''
        reqIKGoal.ik_request.pose_stamped.pose.position.x = self.goalPose[0]
        reqIKGoal.ik_request.pose_stamped.pose.position.y = self.goalPose[1]
        reqIKGoal.ik_request.pose_stamped.pose.position.z = self.goalPose[2]
        reqIKGoal.ik_request.pose_stamped.pose.orientation.x = self.goalPose[3]
        reqIKGoal.ik_request.pose_stamped.pose.orientation.y = self.goalPose[4]
        reqIKGoal.ik_request.pose_stamped.pose.orientation.z = self.goalPose[5]
        reqIKGoal.ik_request.pose_stamped.pose.orientation.w = self.goalPose[6]
        futureGoal = self.client.call_async(reqIKGoal)
        futureGoal.add_done_callback(done_callback)
        event.wait()
        resGoal = futureGoal.result()

        self.qGoal = resGoal.solution.joint_state.position
        print(f"==>> self.qGoal: \n{self.qGoal}")

        response.success = True
        response.message = f"Done Call"
        return response

    def joint_state_callback(self, msg):
        joint_positions = msg.actual.positions
        self.joint = joint_positions

    def goal_pose_array_callback(self, msg):
        pose = msg.pose
        self.goalPose = np.array([pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

    def auxi_pose_array_callback(self, msg):
        pose = msg.pose
        self.auxPose = np.array([pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

    def trigger_plan(self, request, response):

        self.get_logger().info('Planning Start')

        qCurrent = np.array([self.qCurrent[0], self.qCurrent[1], self.qCurrent[2], self.qCurrent[3], self.qCurrent[4], self.qCurrent[5]]).reshape(6, 1)
        qAux = np.array([self.qAux[0], self.qAux[1], self.qAux[2], self.qAux[3], self.qAux[4], self.qAux[5]]).reshape(6, 1)
        qGoal = np.array([self.qGoal[0], self.qGoal[1], self.qGoal[2], self.qGoal[3], self.qGoal[4], self.qGoal[5]]).reshape(6, 1)

        planner = RRTInformedDev(qCurrent, self.wrap_to_pi(qAux), self.wrap_to_pi(qGoal), eta=0.3, maxIteration=100)
        path = planner.planning()

        pathX, pathY, pathZ, pathP, pathQ, pathR = extract_path_class_6d(path)
        self.pathX = np.array(pathX)
        self.pathY = np.array(pathY)
        self.pathZ = np.array(pathZ)
        self.pathP = np.array(pathP)
        self.pathQ = np.array(pathQ)
        self.pathR = np.array(pathR)

        response.success = True
        response.message = "Trajectory Ready. Call Trigger to Publish"

        return response

    def trigger_execute_forward(self, request, response):
        self.traj = JointTrajectory()
        self.traj.joint_names = self.jointsName
        for i in range(len(self.pathX)):
            point = JointTrajectoryPoint()
            point.positions = [self.pathX[i], 
                               self.pathY[i], 
                               self.pathZ[i], 
                               self.pathP[i], 
                               self.pathQ[i], 
                               self.pathR[i]]
            point.time_from_start = Duration(sec=1 + i, nanosec=0)
            self.traj.points.append(point)

        self.trajPublisher.publish(self.traj)

        response.success = True
        response.message = "Robot motion is active now"

        return response

    def trigger_execute_reverse(self, request, response):
        self.traj = JointTrajectory()
        self.traj.joint_names = self.jointsName
        for i in range(len(self.pathX)):
            point = JointTrajectoryPoint()
            point.positions = [self.pathX[(i * -1) - 1], 
                               self.pathY[(i * -1) - 1], 
                               self.pathZ[(i * -1) - 1], 
                               self.pathP[(i * -1) - 1], 
                               self.pathQ[(i * -1) - 1], 
                               self.pathR[(i * -1) - 1]]
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

        self.publisher_joint2.publish(joint_state_msg2)
        self.publisher_joint1.publish(joint_state_msg1)
        self.publisher_joint3.publish(joint_state_msg3)

    def wrap_to_pi(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi


def main(args=None):
    rclpy.init(args=args)
    harvester = HarvesterClient()
    executor = MultiThreadedExecutor()
    rclpy.spin(harvester, executor)
    harvester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
