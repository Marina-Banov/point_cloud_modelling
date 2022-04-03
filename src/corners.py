import argparse
import pcl
from sklearn.metrics.pairwise import manhattan_distances
import numpy as np
import matplotlib.pyplot as plt
import utils


def display(chull, corner_points, centroid):
    corner_points = np.take(chull, corner_points, axis=0)
    corner_points = utils.add_color_to_points(corner_points, [1.0, 0.0, 0.0])
    points = utils.add_color_to_points(chull, [0.85, 0.85, 0.85])
    points = np.append(corner_points, points, axis=0)
    centroid = utils.add_color_to_points(centroid, [0.0, 0.0, 1.0])
    points = np.append(points, centroid, axis=0)
    utils.visualize(points)


def get_corner_indices(diff_x):
    x_indices = np.not_equal(diff_x, None).nonzero()[0]
    cur_group = []
    res = []

    for i in range(len(x_indices) - 1):
        if x_indices[i + 1] - x_indices[i] < 20:
            cur_group.append(x_indices[i])
        else:
            if len(cur_group) > 20:
                res.append(cur_group[-1])
            cur_group = []

    if len(cur_group) > 20:
        res.append(cur_group[-1])

    return res


def corners(cloud, alpha=0.07, threshold=0.0005):
    seg = utils.setup_segmenter(cloud, 0, 0, 1)
    indices, coefficients = seg.segment()
    print(coefficients)
    chull = np.asarray(
        utils.get(cloud, indices, utils.VisualizeType.CONCAVE_HULL,
                  alpha=alpha)
    )

    print("-------CALCULATING MEAN AND DIST-------")
    centroid = chull.mean(axis=0)
    print(centroid)

    dist = manhattan_distances([centroid], chull, sum_over_features=False)
    diff_x = np.diff(dist[:, 0])
    diff_x = np.where(abs(diff_x) < threshold, diff_x, None)
    diff_y = np.diff(dist[:, 1])
    diff_y = np.where(abs(diff_y) < threshold, diff_y, None)

    x_indices = get_corner_indices(diff_x)
    y_indices = get_corner_indices(diff_y)
    corner_points = np.concatenate((x_indices, y_indices))

    for i in corner_points:
        print(i, chull[i])

    display(chull, corner_points, centroid)

    plt.plot(list(range(dist.shape[0])), dist[:, 0], 'b')
    plt.plot(list(range(dist.shape[0])), dist[:, 1], 'g')
    for i in corner_points:
        plt.axvline(x=i, color='k')

    plt.xlabel('index')
    plt.ylabel('distance')
    plt.grid(True)
    plt.show()

    return chull[corner_points]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-f", type=str, required=True, metavar="FILE",
        help="path to a .pcd file"
    )
    parser.add_argument(
        "-a", type=float, default=0.07, metavar="ALPHA",
        help="alpha value used for concave hull extraction, default: 0.07"
    )
    parser.add_argument(
        "-t", type=float, default=0.0005, metavar="THRESHOLD",
        help="minimal distance, default: 0.0005"
    )
    filename = parser.parse_args().f
    alpha = parser.parse_args().a
    threshold = parser.parse_args().t

    print("-------LOADING PCD-------")
    cloud = pcl.load(filename)

    print("-------DOWNSAMPLING-------")
    vg = cloud.make_voxel_grid_filter()
    vg.set_leaf_size(0.02, 0.02, 0.02)
    cloud = vg.filter()

    corners(cloud, alpha, threshold)


if __name__ == '__main__':
    main()
