"The solution is random since we don't have init joint value, must investigate further"
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from std_srvs.srv import Trigger
from rclpy.executors import MultiThreadedExecutor
from threading import Event
from rclpy.callback_groups import ReentrantCallbackGroup

class IKClient(Node):
    def __init__(self):
        super().__init__('ik_client')
        self.service_done_event = Event()
        self.callback_group = ReentrantCallbackGroup()

        self.client = self.create_client(GetPositionIK, 'compute_ik', callback_group=self.callback_group)
        self.trigger_pose = self.create_service(Trigger, 'trigger_pose', self.trigger_pose_callback, callback_group=self.callback_group)

    def trigger_pose_callback(self, request, response):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('No action server available')
            
        self.service_done_event.clear()
        event = Event()

        def done_callback(future):
            nonlocal event
            event.set()

        reqIK = GetPositionIK.Request()
        reqIK.ik_request.group_name = 'ur_manipulator'  # Set the MoveIt planning group name
        reqIK.ik_request.pose_stamped.header.frame_id = ''  # Set the desired frame ID
        reqIK.ik_request.pose_stamped.pose.position.x = 0.5  # Set the desired pose position
        reqIK.ik_request.pose_stamped.pose.position.y = 0.0
        reqIK.ik_request.pose_stamped.pose.position.z = 0.3
        reqIK.ik_request.pose_stamped.pose.orientation.x = 0.0  # Set the desired pose orientation
        reqIK.ik_request.pose_stamped.pose.orientation.y = 0.0  # Set the desired pose orientation
        reqIK.ik_request.pose_stamped.pose.orientation.z = 0.0  # Set the desired pose orientation
        reqIK.ik_request.pose_stamped.pose.orientation.w = 1.0  # Set the desired pose orientation
        future = self.client.call_async(reqIK)
        future.add_done_callback(done_callback)
        # Wait for action to be done
        # self.service_done_event.wait()
        event.wait()
        res = future.result()
        response.success = True
        response.message = f"{res.solution.joint_state.position}"
        return response
    
    def send_request(self):
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'ur_manipulator'  # Set the MoveIt planning group name
        request.ik_request.pose_stamped.header.frame_id = ''  # Set the desired frame ID
        request.ik_request.pose_stamped.pose.position.x = 0.5
        request.ik_request.pose_stamped.pose.position.y = 0.0
        request.ik_request.pose_stamped.pose.position.z = 0.3
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 0.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 1.0
    
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    ik_client = IKClient()
    # ik_client.send_request()
    # respone = ik_client.send_request()
    # print(f"==>> respone: \n{respone}")

    # joint1 = respone.solution.joint_state.position[0]
    # joint2 = respone.solution.joint_state.position[1]
    # joint3 = respone.solution.joint_state.position[2]
    # joint4 = respone.solution.joint_state.position[3]
    # joint5 = respone.solution.joint_state.position[4]
    # joint6 = respone.solution.joint_state.position[5]
    # print(f"Joint Pose = {[joint1, joint2, joint3, joint4, joint5, joint6]}")
    executor = MultiThreadedExecutor()
    rclpy.spin(ik_client, executor)
    ik_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
