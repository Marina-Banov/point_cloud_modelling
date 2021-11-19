import argparse
import numpy as np
import pcl
import pcl.pcl_visualization
from utils import VisualizeType, get, visualize, setup_segmenter
import random


def remove_points(cloud, indices):
    cloud_points = np.full((cloud.size, 3), cloud, dtype=np.float32)
    finalpoints = np.delete(cloud_points, indices, axis=0)
    final = pcl.PointCloud()
    final.from_array(finalpoints)
    return final


def transform_inliers(inliers):
    color = np.array([random.random(), random.random(), random.random()], dtype=np.float32)
    res = np.zeros((inliers.size, 6))
    res[:, :3] = inliers
    res[:, 3:] = color
    return res


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", type=str, required=True, metavar="FILE", help="path to a .pcd file")
    parser.add_argument(
        "-t", type=int, default=5000, metavar="THRESHOLD",
        help="minimum number of points a segmented plane should contain, default: 5000"
    )
    filename = parser.parse_args().f
    threshold = parser.parse_args().t

    print("-------LOADING PCD-------")
    cloud = pcl.load(filename)

    print("-------DOWNSAMPLING-------")
    vg = cloud.make_voxel_grid_filter()
    vg.set_leaf_size(0.05, 0.05, 0.05)
    cloud = vg.filter()

    x, y, z = (0, 0, 1)
    seg = setup_segmenter(cloud, x, y, z)
    indices, coefficients = seg.segment()
    print(coefficients)
    inliers = get(cloud, indices, VisualizeType.ONLY_INLIERS)
    cloud = remove_points(cloud, indices)
    result_points = np.empty((0, 6), dtype=np.float32)
    result_points = np.append(result_points, transform_inliers(inliers), axis=0)

    x, y, z = (1, 0, 0)
    while True:
        seg = setup_segmenter(cloud, x, y, z)
        indices, coefficients = seg.segment()
        print(coefficients)

        if len(indices) > threshold:
            inliers = get(cloud, indices, VisualizeType.ONLY_INLIERS)
            cloud = remove_points(cloud, indices)
            result_points = np.append(result_points, transform_inliers(inliers), axis=0)
            x, y = (y, x)
        else:
            break

    visualize(result_points)


if __name__ == '__main__':
    main()
