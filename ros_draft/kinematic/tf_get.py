import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
from scipy.spatial.transform import Rotation as R

class FrameListener(Node):

    def __init__(self):
        super().__init__('frame_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        from_frame_rel = 'camera_color_frame'
        to_frame_rel = 'base_link'
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, now)
            translation = trans.transform.translation
            rotation = trans.transform.rotation

            # Extract translation and rotation as numpy arrays
            translation_array = np.array([translation.x, translation.y, translation.z])
            print(f"==>> translation_array: \n{translation_array}")
            rotation_array = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
            print(f"==>> rotation_array: \n{rotation_array}")

            # Calculate the transformation matrix
            transformation_matrix = self.compose_transformation_matrix(translation_array, rotation_array)
            print(f"==>> transformation_matrix: \n{transformation_matrix}")

        except TransformException as ex:
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')


    def compose_transformation_matrix(self, translation, rotation):
        rot = R.from_quat(rotation)
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rot.as_matrix()
        transformation_matrix[:3, 3] = translation
        return transformation_matrix


def main(args=None):
    rclpy.init(args=args)
    imageSubscriber = FrameListener()
    rclpy.spin(imageSubscriber)
    imageSubscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()