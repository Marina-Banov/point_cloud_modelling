import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from . import utils

SOURCE_FRAME_ID = 'odom'
TARGET_FRAME_ID = 'camera_depth_optical_frame'


class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        try:
            timestamp = rclpy.time.Time()
            self.tf_buffer.can_transform(TARGET_FRAME_ID, SOURCE_FRAME_ID, timestamp, Duration(seconds=2))
            trans = self.tf_buffer.lookup_transform(TARGET_FRAME_ID, SOURCE_FRAME_ID, timestamp)
            angles = utils.euler_from_quaternion(trans.transform.rotation)
            # self.get_logger().info(str(trans.transform.translation))
            self.get_logger().info(str(angles))
        except TransformException as ex:
            self.get_logger().warning('------------------------------------------')


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
