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


def euler_from_quaternion(q):
    angles = Vector3()

    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    angles.x = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (q.w * q.y - q.z * q.x)
    angles.y = np.arcsin(sinp)

    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    angles.z = np.arctan2(siny_cosp, cosy_cosp)

    return angles


class GetPcdNode(Node):
    def __init__(self):
        super().__init__('get_pcd')
        self.points = set()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  # necessary
        self.camera_sensor_trans = None
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
            self.tf_buffer.can_transform(target_frame, source_frame, timestamp, Duration(seconds=2))
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, timestamp)
            self.get_logger().info(f'Successfully transformed {source_frame} to {target_frame}')
            return trans
        except TransformException as ex:
            self.get_logger().warning(f'Could not transform {source_frame} to {target_frame}: {ex}')
            return None

    def get_transform_matrices(self):
        Vq = self.camera_sensor_trans.transform.rotation
        angles = euler_from_quaternion(Vq)
        Vq = [Vq.w, Vq.x, Vq.y, Vq.z]
        angles.x = 0.0
        angles.y = 0.0
        Rx = tfs.rotation_matrix(angles.x, [1, 0, 0])
        Ry = tfs.rotation_matrix(angles.y, [0, 1, 0])
        Rz = tfs.rotation_matrix(angles.z, [0, 0, 1])

        Vt = self.camera_sensor_trans.transform.translation
        Vt = [Vt.x, Vt.y, Vt.z]
        return np.around(tfs.concatenate_matrices(Rx, Ry, Rz), 5), np.around(tfs.translation_matrix(Vt), 5)
        # return tfs.quaternion_matrix(Vq), tfs.translation_matrix(Vt)

    def process_frame(self, data):
        if not self.flag:
            return
        self.flag = False
        self.get_logger().info(f"Processing frame...")

        self.camera_sensor_trans = self.get_transform(SOURCE_FRAME_ID, TARGET_FRAME_ID, data.header.stamp)
        if not self.camera_sensor_trans:
            self.flag = True
            return
        Mr, Mt = self.get_transform_matrices()

        pcd_data = np.array(list(read_points(data, field_names=['x', 'y', 'z'], skip_nans=True)))
        P = np.ones((pcd_data.shape[0], 4))  # add the fourth column
        P[:, :-1] = pcd_data
        P = np.around(tfs.concatenate_matrices(Mr, Mt, P.T), 3).T

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
