import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion


class KW(Node):

    def __init__(self):
        super().__init__(node_name='turtlebot_controller')
        self.pub = self.create_publisher(Pose, "pose", 10)
        self.timer = self.create_timer(1, self.pose_callback)

    def pose_callback(self):
        pose_msg = Pose()
        pose_msg.position = Point(x=1.0, y=2.0, z=1.0)
        pose_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    controlnode = KW()
    rclpy.spin(controlnode)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
