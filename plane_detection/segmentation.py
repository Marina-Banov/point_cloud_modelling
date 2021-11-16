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
    color = np.array([random.randint(0,255), random.randint(0,255), random.randint(0,255)], dtype=np.int32)
    rgb = np.array([color[0] << 16 | color[1] << 8 | color[2]], dtype=np.float32)
    res = np.zeros((inliers.size, 4))
    res[:, :3] = inliers
    res[:, 3] = rgb
    return res


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", "-f", type=str, required=True)
    filename = parser.parse_args().file

    print("-------LOADING PCD-------")
    cloud = pcl.load(filename)

    result_points = np.empty((0,3), dtype=np.float32)

    seg = setup_segmenter(cloud, 0, 0, 1)
    indices, coefficients = seg.segment()
    print(coefficients)
    inliers = get(cloud, indices, VisualizeType.ONLY_INLIERS)
    cloud = remove_points(cloud, indices)
    result_points = np.append(result_points, inliers, axis=0)

    for i in range(2):
        seg = setup_segmenter(cloud, 1, 0, 0)
        indices, coefficients = seg.segment()
        print(coefficients)

        if len(indices) != 0:
            inliers = get(cloud, indices, VisualizeType.ONLY_INLIERS)
            cloud = remove_points(cloud, indices)
            result_points = np.append(result_points, inliers, axis=0)

    for i in range(2):
        seg = setup_segmenter(cloud, 0, 1, 0)
        indices, coefficients = seg.segment()
        print(coefficients)

        if len(indices) != 0:
            inliers = get(cloud, indices, VisualizeType.ONLY_INLIERS)
            cloud = remove_points(cloud, indices)
            result_points = np.append(result_points, inliers, axis=0)

    result = pcl.PointCloud()  # TODO XYZRGB
    result.from_array(result_points.astype(np.float32))
    visualize(result)


if __name__ == '__main__':
    main()
