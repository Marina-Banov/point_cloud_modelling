import time
import numpy as np
import open3d as o3d
import transformations as tfs
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Transform, Vector3

POINTS = [
    [(0, -1, 5), (0, -1, 7), (0, 1, 5), (0, 1, 7), (2, -0.5, 6.5), (2, 0.5, 6.5), (2, 0, 5.5)],
    [(0, -1, 2), (0, -1, 4), (0, 1, 2), (0, 1, 4), (2, -0.5, 3.5), (2, 0.5, 3.5), (2, 0, 2.5)]
]
POSITION = [Transform(), Transform(translation=Vector3(z=3.0))]


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

class TestingNode(Node):
    def __init__(self):
        super().__init__('testing')
        self.points = set()
        self.camera_sensor_trans = None
        self.process_frame(0)
        self.process_frame(1)

    def get_transform_matrices(self):
        Vq = self.camera_sensor_trans.rotation
        angles = euler_from_quaternion(Vq)
        Vq = [Vq.w, Vq.x, Vq.y, Vq.z]
        angles.x = 0.0
        angles.y = 0.0
        angles.z = 0.0
        Rx = tfs.rotation_matrix(angles.x, [1, 0, 0])
        Ry = tfs.rotation_matrix(angles.y, [0, 1, 0])
        Rz = tfs.rotation_matrix(angles.z, [0, 0, 1])

        Vt = self.camera_sensor_trans.translation
        Vt = [Vt.x, Vt.y, Vt.z]
        return np.around(tfs.concatenate_matrices(Rx, Ry, Rz), 5), np.around(tfs.translation_matrix(Vt), 5)
        # return tfs.quaternion_matrix(Vq), tfs.translation_matrix(Vt)

    def process_frame(self, iteration):
        self.get_logger().info(f"Processing frame...")
        self.camera_sensor_trans = POSITION[iteration]
        Mr, Mt = self.get_transform_matrices()

        pcd_data = np.array(POINTS[iteration])
        P = np.ones((pcd_data.shape[0], 4))  # add the fourth column
        P[:, :-1] = pcd_data
        P = tfs.concatenate_matrices(Mr, Mt, P.T).T

        # tuples are hashable objects and will cause collisions when added to a set
        new_points = list(map(lambda t: (t[0], t[1], t[2]), P))
        self.points.update(new_points)

        self.get_logger().info(f"Frame processed")


    def save_pcd(self):
        try:
            self.get_logger().info(f"Saving pointcloud ({len(self.points)} points)...")
            out_pcd = o3d.geometry.PointCloud()
            out_pcd.points = o3d.utility.Vector3dVector(list(self.points))
            timestr = time.strftime("%Y%m%d%H%M%S")
            o3d.io.write_point_cloud(f"./{timestr}.pcd", out_pcd)
            self.get_logger().info(f"Saved pointcloud {timestr}.pcd")
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    get_pcd_node = TestingNode()
    get_pcd_node.save_pcd()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
