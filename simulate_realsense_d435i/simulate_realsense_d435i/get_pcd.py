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
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
import transformations as tfs


SOURCE_FRAME_ID = 'odom'
TARGET_FRAME_ID = 'camera_depth_optical_frame'


class GetPcdNode(Node):
    def __init__(self):
        super().__init__('get_pcd')
        self.points = set()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) # necessary
        self.camera_sensor_trans = TransformStamped()
        self.create_subscription(
            PointCloud2,
            '/depth/color/points',
            self.process_frame,
            10
        )
        
    def get_transform(self, source_frame, target_frame, default_value=None):
        try:
            now = rclpy.time.Time()
            self.tf_buffer.can_transform(target_frame, source_frame, now, Duration(seconds=2))
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, now)
            self.get_logger().info(f'Successfully transformed {source_frame} to {target_frame}')
            return trans
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {source_frame} to {target_frame}: {ex}')
            return default_value
            
    def get_transform_matrices(self):
        Vq = self.camera_sensor_trans.transform.rotation
        Vt = self.camera_sensor_trans.transform.translation
        Vq = [Vq.w, Vq.x, Vq.y, Vq.z]
        Vt = [Vt.x, Vt.y, Vt.z]
        # angle = tfs.euler_from_quaternion(Vq)[2]
        # return tfs.rotation_matrix(angle, [0,1,0]), tfs.translation_matrix(Vt)
        return tfs.quaternion_matrix(Vq), tfs.translation_matrix(Vt)

    def process_frame(self, data):
        #self.lock.acquire()
        self.get_logger().info(f"Processing frame...")
        
        self.camera_sensor_trans = self.get_transform(SOURCE_FRAME_ID, TARGET_FRAME_ID, self.camera_sensor_trans)
        Rq, Rt = self.get_transform_matrices()
        M = np.dot(Rq, Rt)
        
        pcd_data = np.array(list(pc2.read_points(data, field_names=['x','y','z'], skip_nans=True)))
        points = np.ones((pcd_data.shape[0], 4)) # add the fourth column
        points[:,:-1] = pcd_data
        points = np.around(np.dot(M, points.T), 3).T
        
        # tuples are hashable objects and will cause collisions when added to a set
        new_points = list(map(lambda t: tuple(tfs.euler_from_quaternion(t)), points))
        self.points.update(new_points)
        
        self.get_logger().info(f"Frame processed")
        
    def save_pcd(self):
        try:
            self.get_logger().info(f"Saving pointcloud ({len(self.points)} points)...")
            out_pcd = o3d.geometry.PointCloud()
            # only take every other point
            out_pcd.points = o3d.utility.Vector3dVector(list(self.points)[::2])
            # out_pcd.points = o3d.utility.Vector3dVector(list(self.points))
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
        get_pcd_node.save_pcd()
        
    rclpy.shutdown()


if __name__ == '__main__':
    main()
