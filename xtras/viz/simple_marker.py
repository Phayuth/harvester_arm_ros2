import sys
import rclpy
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion


def processFeedback(feedback):
    p = feedback.pose.position
    print(f'{feedback.marker_name} is now at {p.x}, {p.y}, {p.z}')


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('simple_marker')

    # create an interactive marker server on the namespace simple_marker
    server = InteractiveMarkerServer(node, 'namespace_marker')

    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = 'base_link'
    int_marker.name = 'my_marker'
    int_marker.description = 'Simple 1-DOF Control Johncena Naruto'

    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.45
    box_marker.scale.z = 0.45
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append(box_marker)

    # add the control to the interactive marker
    int_marker.controls.append(box_control)

    # create a control which will move the box
    # this control does not contain any markers,
    # which will cause RViz to insert two arrows
    rotate_control_x = InteractiveMarkerControl()
    rotate_control_x.orientation.w = 0.5
    rotate_control_x.orientation.x = 0.5
    rotate_control_x.orientation.y = 0.5
    rotate_control_x.orientation.z = 0.5
    rotate_control_x.name = 'move_x'
    rotate_control_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    rotate_control_x.orientation_mode = 0

    # add the control to the interactive marker
    int_marker.controls.append(rotate_control_x)

    # rotate_control_y = InteractiveMarkerControl()
    # rotate_control_y.name = 'move_y'
    # rotate_control_y.interaction_mode = InteractiveMarkerControl.MOVE_PLANE

    # int_marker.controls.append(rotate_control_y)

    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, feedback_callback=processFeedback)

    # 'commit' changes and send to all clients
    server.applyChanges()

    rclpy.spin(node)
    server.shutdown()
