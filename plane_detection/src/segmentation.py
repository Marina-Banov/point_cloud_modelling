import argparse
import numpy as np
import pcl
import pcl.pcl_visualization
from utils import VisualizeType, get, visualize, setup_segmenter, add_color_to_points
import random


def remove_points(cloud, indices):
    cloud_points = np.full((cloud.size, 3), cloud, dtype=np.float32)
    finalpoints = np.delete(cloud_points, indices, axis=0)
    return pcl.PointCloud(finalpoints)


def random_color():
    return np.array([random.random(), random.random(), random.random()], dtype=np.float32)


def segmentation(cloud, threshold=5000):
    x, y, z = (0, 0, 1)
    seg = setup_segmenter(cloud, x, y, z)
    indices, coefficients = seg.segment()
    print(coefficients)
    inliers = get(cloud, indices, VisualizeType.ONLY_INLIERS)
    cloud = remove_points(cloud, indices)
    result_points = np.empty((0, 6), dtype=np.float32)
    result_points = np.append(result_points, add_color_to_points(inliers, random_color()), axis=0)

    x, y, z = (1, 0, 0)
    while True:
        seg = setup_segmenter(cloud, x, y, z)
        indices, coefficients = seg.segment()

        if len(indices) > threshold:
            print(coefficients)
            inliers = get(cloud, indices, VisualizeType.ONLY_INLIERS)
            cloud = remove_points(cloud, indices)
            result_points = np.append(result_points, add_color_to_points(inliers, random_color()), axis=0)
            x, y = (y, x)
        else:
            break

    visualize(result_points)
    return result_points


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

    segmentation(cloud, threshold)


if __name__ == '__main__':
    main()
