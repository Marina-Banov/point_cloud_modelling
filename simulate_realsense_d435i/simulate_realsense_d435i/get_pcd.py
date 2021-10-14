import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import open3d as o3d
from . import point_cloud2 as pc2
import time


class GetPcdNode(Node):
    def __init__(self):
        super().__init__('get_pcd')
        self.create_subscription(
            PointCloud2,
            '/depth/color/points',
            self.save_pcd,
            10
        )

    def save_pcd(self, data):
        # self.lock.acquire()
        pcd_data = pc2.read_points(data, field_names=['x','y','z'], skip_nans=True)
        xyz = list(pcd_data)
        rgb = [[0.5,0.5,0.5]] * len(xyz)
        
        out_pcd = o3d.geometry.PointCloud()
        out_pcd.points = o3d.utility.Vector3dVector(xyz)
        out_pcd.colors = o3d.utility.Vector3dVector(rgb)
        
        timestr = time.strftime("%Y%m%d%H%M%S")
        o3d.io.write_point_cloud(f"./{timestr}.pcd",out_pcd)
        self.get_logger().info(f"Saved frame {timestr}")


def main(args=None):
    try:
        rclpy.init(args=args)
        get_pcd_node = GetPcdNode()
        rclpy.spin(get_pcd_node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        sys.exit(0)


if __name__ == '__main__':
    main()
