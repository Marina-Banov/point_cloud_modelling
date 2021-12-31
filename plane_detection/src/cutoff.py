import argparse
import pcl
import utils
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-f", type=str, required=True, metavar="FILE",
        help="path to a .pcd file"
    )
    filename = parser.parse_args().f

    print("-------LOADING PCD-------")
    cloud = pcl.load(filename)
    cloud_points = np.full((cloud.size, 3), cloud, dtype=np.float32)
    finalpoints = np.delete(cloud_points,
                            np.where((cloud_points[:, 0] < -8.5) |
                                     (cloud_points[:, 1] > -1.6))[0],
                            axis=0)
    cloud = pcl.PointCloud(finalpoints)
    cloud.to_file("20211201114059_.pcd".encode('utf-8'), ascii=False)

    print("-------DOWNSAMPLING-------")
    vg = cloud.make_voxel_grid_filter()
    vg.set_leaf_size(0.05, 0.05, 0.05)
    cloud = vg.filter()

    utils.visualize(np.asarray(cloud))


if __name__ == '__main__':
    main()
