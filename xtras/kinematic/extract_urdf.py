import rclpy
from rclpy.node import Node
import urdf_parser_py.urdf as urdf
from std_msgs.msg import String


class PyKDLChainReader(Node):

    def __init__(self):
        super().__init__("pykdl_chain_reader")
        self.subscription = self.create_subscription(String, "/ikenv/robot_description", self.urdf_callback, 10)

    def urdf_callback(self, msg):
        urdf_string = msg.data
        robot = urdf.URDF.from_xml_string(urdf_string)

        urdf_file_path_with_urdf_parser = "/home/yuth/experiment/ICRA2024HandOverComp/urdf_file.urdf"
        with open(urdf_file_path_with_urdf_parser, "w") as urdf_file_with_urdf_parser:
            urdf_file_with_urdf_parser.write(robot.to_xml_string())


def main(args=None):
    rclpy.init(args=args)
    pykdl_chain_reader = PyKDLChainReader()
    rclpy.spin_once(pykdl_chain_reader)
    pykdl_chain_reader.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
