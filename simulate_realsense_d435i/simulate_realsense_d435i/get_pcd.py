import time
import numpy as np
import open3d as o3d
import transformations as tfs
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from geometry_msgs.msg import Vector3
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

SOURCE_FRAME_ID = 'odom'
TARGET_FRAME_ID = 'camera_depth_optical_frame'


class GetPcdNode(Node):
    def __init__(self):
        super().__init__('get_pcd')
        self.points = set()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  # necessary
        self.create_subscription(
            PointCloud2,
            '/depth/color/points',
            self.process_frame,
            10
        )
        self.flag = True
        self.create_subscription(
            Bool,
            '/break',
            self.toggle_flag,
            10
        )

    def toggle_flag(self, data):
        time.sleep(1)
        self.flag = True

    def get_transform(self, source_frame, target_frame, timestamp):
        try:
            self.tf_buffer.can_transform(target_frame, source_frame, timestamp, Duration(seconds=0))
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, timestamp)
            self.get_logger().info(f'Successfully transformed {source_frame} to {target_frame}')
            return trans.transform
        except TransformException as ex:
            self.get_logger().warning(f'Could not transform {source_frame} to {target_frame}: {ex}')
            return None

    def get_transform_matrices(self, transform):
        Vq = transform.rotation
        Vq = [Vq.w, -Vq.x, -Vq.y, -Vq.z]
        Vt = transform.translation
        Vt = [Vt.x, Vt.y, Vt.z]
        return np.around(tfs.quaternion_matrix(Vq), 5), np.around(tfs.translation_matrix(Vt), 5)

    def process_frame(self, data):
        if not self.flag:
            return
        self.flag = False
        self.get_logger().info(f"Processing frame...")

        trans = self.get_transform(SOURCE_FRAME_ID, TARGET_FRAME_ID, data.header.stamp)
        if not trans:
            self.flag = True
            return
        Mr, Mt = self.get_transform_matrices(trans)
        Ms = tfs.scale_matrix(-1)
        Mr_ = np.around(tfs.quaternion_matrix([0.5, -0.5, 0.5, 0.5]), 5)

        pcd_data = np.array(list(read_points(data, field_names=['x', 'y', 'z'], skip_nans=True)))
        P = np.ones((pcd_data.shape[0], 4))  # add the fourth column
        P[:, :-1] = pcd_data
        
        # P = np.around(np.dot(Mr, P.T), 3).T  # ROTATION WORKS
        # P = np.around(tfs.concatenate_matrices(Mr, Mr_, P.T), 3).T
        # P = np.around(tfs.concatenate_matrices(Mt, Ms, P.T), 3).T  # TRANSLATION WORKS
        P = np.around(tfs.concatenate_matrices(Ms, Mt, Mr, Mr_, P.T), 3).T

        # tuples are hashable objects and will cause collisions when added to a set
        new_points = list(map(lambda t: (t[0], t[1], t[2]), P))
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
            o3d.io.write_point_cloud(f"./{timestr}.pcd", out_pcd)
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
