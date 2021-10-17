import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import open3d as o3d
from . import point_cloud2 as pc2
import time
from tf2_msgs.msg import TFMessage
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from rclpy.duration import Duration

CAMERA_TREE = ['camera_bottom_screw_frame', 'camera_link', 'camera_depth_frame','camera_depth_optical_frame']
source_frame = 'camera_link'
target_frame = 'camera_depth_optical_frame'

class GetPcdNode(Node):
    def __init__(self):
        super().__init__('get_pcd')
        self.points = set()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        """self.create_subscription(
            PointCloud2,
            '/depth/color/points',
            self.process_frame,
            10
        )
        self.create_subscription(
            TFMessage,
            '/tf',
            self.y,
            1
        )"""
        self.create_subscription(
            TFMessage,
            '/tf_static',
            self.get_camera_transforms,
            2
        )
        
    def get_camera_transforms(self, data):
        try:
            now = rclpy.time.Time()
            self.tf_buffer.can_transform(target_frame, source_frame, now, Duration(seconds=2))
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, now)
            self.get_logger().info(str(trans))
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {source_frame} to {target_frame}: {ex}')

    def y(self, data):
        self.get_logger().info("/tf")
        self.get_logger().info(str(data.transforms))

    def process_frame(self, data):
        #self.lock.acquire()
        self.get_logger().info(f"Processing frame...")
        pcd_data = pc2.read_points(data, field_names=['x','y','z'], skip_nans=True)
        # round xyz coordinates of each point to 3 decimal spots
        new_points = list(map(lambda t: tuple(map(lambda x: round(x, 3), t)), list(pcd_data)))
        self.points.update(new_points)
        self.get_logger().info(f"Frame processed")
        
    def save_pcd(self):
        try:
            self.get_logger().info(f"Saving pointcloud ({len(self.points)} points)...")
            out_pcd = o3d.geometry.PointCloud()
            # only take every other point
            # out_pcd.points = o3d.utility.Vector3dVector(list(self.points)[::2])
            out_pcd.points = o3d.utility.Vector3dVector(list(self.points))
            timestr = time.strftime("%Y%m%d%H%M%S")
            o3d.io.write_point_cloud(f"./{timestr}.pcd",out_pcd)
            self.get_logger().info(f"Saved pointcloud {timestr}.pcd")
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    get_pcd_node = GetPcdNode()
    
    try:
        rclpy.spin(get_pcd_node)
    except KeyboardInterrupt:
        pass
        # get_pcd_node.save_pcd()
        
    rclpy.shutdown()


if __name__ == '__main__':
    main()
