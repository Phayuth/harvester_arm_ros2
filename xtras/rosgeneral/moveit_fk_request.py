import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionFK


class FKClient(Node):

    def __init__(self):
        super().__init__('fk_client')
        self.client = self.create_client(GetPositionFK, 'compute_fk')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GetPositionFK service not available, waiting...')

    def send_request(self):
        request = GetPositionFK.Request()
        request.header.frame_id = 'base_link'  # Set the desired frame ID
        request.fk_link_names = ['tool0']  # Set the link for which you want to compute FK
        request.robot_state.joint_state.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        request.robot_state.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Set the joint positions

        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    fk_client = FKClient()
    respone = fk_client.send_request()
    # print(f"==>> respone: \n{respone}")

    x = respone.pose_stamped[0].pose.position.x
    y = respone.pose_stamped[0].pose.position.y
    z = respone.pose_stamped[0].pose.position.z

    ox = respone.pose_stamped[0].pose.orientation.x
    oy = respone.pose_stamped[0].pose.orientation.y
    oz = respone.pose_stamped[0].pose.orientation.z
    ow = respone.pose_stamped[0].pose.orientation.w

    print(f"pose = {[x, y, z]}")
    print(f"ornt = {[ox, oy, oz, ow]}")

    fk_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
