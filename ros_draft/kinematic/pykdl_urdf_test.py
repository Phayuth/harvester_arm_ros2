import rclpy
from rclpy.node import Node
import PyKDL
import urdf_parser_py.urdf as urdf
from std_msgs.msg import String  # Import the message type for /robot_description

class PyKDLChainReader(Node):

    def __init__(self):
        super().__init__('pykdl_chain_reader')

        self.subscription = self.create_subscription(String, '/robot_description', self.urdf_callback, 10)

    def urdf_callback(self, msg):
        urdf_string = msg.data
        robot = urdf.URDF.from_xml_string(urdf_string)

        root_link = robot.get_root()
        end_effector_link = 'gripper'

        chain = PyKDL.Chain()
        chainlist = robot.get_chain(root_link, end_effector_link)
        print(f"==>> chainlist: \n{chainlist}")

def main(args=None):
    rclpy.init(args=args)
    pykdl_chain_reader = PyKDLChainReader()
    rclpy.spin(pykdl_chain_reader)
    pykdl_chain_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
