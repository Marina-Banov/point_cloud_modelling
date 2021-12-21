import argparse
import numpy as np
import pcl
import pcl.pcl_visualization
import utils
import random
import json


def remove_points(cloud, indices):
    cloud_points = np.full((cloud.size, 3), cloud, dtype=np.float32)
    finalpoints = np.delete(cloud_points, indices, axis=0)
    return pcl.PointCloud(finalpoints)


def random_color():
    return np.array([random.random(), random.random(), random.random()],
                    dtype=np.float32)


def segmentation(cloud, threshold=5000):
    planes = []
    segmented_cloud = []
    visualize_points = np.empty((0, 6), dtype=np.float32)

    x, y, z = (0, 0, 1)
    seg = utils.setup_segmenter(cloud, x, y, z)
    indices, coefficients = seg.segment()
    planes.append(coefficients)
    inliers = utils.get(cloud, indices, utils.VisualizeType.ONLY_INLIERS)
    cloud = remove_points(cloud, indices)
    segmented_cloud.append(inliers)
    visualize_points = np.append(
        visualize_points,
        utils.add_color_to_points(inliers, random_color()),
        axis=0
    )

    x, y, z = (1, 0, 0)
    while True:
        seg = utils.setup_segmenter(cloud, x, y, z)
        indices, coefficients = seg.segment()

        if len(indices) > threshold:
            planes.append(coefficients)
            inliers = utils.get(cloud, indices,
                                utils.VisualizeType.ONLY_INLIERS)
            cloud = remove_points(cloud, indices)
            segmented_cloud.append(inliers)
            visualize_points = np.append(
                visualize_points,
                utils.add_color_to_points(inliers, random_color()),
                axis=0
            )
            x, y = y, x
        else:
            break

    utils.visualize(visualize_points)
    return planes, segmented_cloud


def intersection(planes):
    corners = []

    for i in range(1, len(planes), 2):
        for j in range(2, len(planes), 2):
            A = [planes[0][:3], planes[i][:3], planes[j][:3]]
            B = [-planes[0][3], -planes[i][3], -planes[j][3]]
            corner = np.linalg.solve(A, B)
            corners.append(list(corner))

    corners = np.asarray(corners)
    utils.visualize(corners)

    return corners


def get_net(corners, planes):
    passed = list(range(len(corners)))
    keep = {}
    for i in passed:
        keep[str(i)] = [[], [], [], []]

    i = 0
    while len(passed) > 0:
        try:
            passed.remove(i)
        except Exception as e:
            break
        f = i
        for j in passed:
            diff_x = corners[i, 0] - corners[j, 0]
            diff_y = corners[i, 1] - corners[j, 1]
            if 0 < diff_x < 0.1:
                keep[str(i)][0].append((abs(diff_x), j))
                keep[str(j)][1].append((abs(diff_x), i))
                f = j
            if 0 >= diff_x > -0.1:
                keep[str(i)][1].append((abs(diff_x), j))
                keep[str(j)][0].append((abs(diff_x), i))
                f = j
            if 0 < diff_y < 0.1:
                keep[str(i)][2].append((abs(diff_y), j))
                keep[str(j)][3].append((abs(diff_y), i))
                f = j
            if 0 >= diff_y > -0.1:
                keep[str(i)][3].append((abs(diff_y), j))
                keep[str(j)][2].append((abs(diff_y), i))
                f = j
        i = f

    result = []
    for key, directions in keep.items():
        for d in directions:
            if len(d) == 0:
                continue
            min_diff = min(d)[1]
            if (int(key), min_diff) not in result and (
                min_diff, int(key)) not in result and edge_exists(
                    corners[int(key)], corners[min_diff], planes):
                result.append((int(key), min_diff))
    return result


def edge_exists(point_a, point_b, segmented_cloud):
    if abs(point_a[0] - point_b[0]) <= 0.1:
        middle = (point_a[1] + point_b[1]) / 2
        for i in range(1, len(segmented_cloud), 2):
            p = np.asarray(segmented_cloud[i])
            together = np.where((abs(p[:, 1] - middle) <= 0.1) &
                                (abs(p[:, 0] - point_b[0]) <= 0.1) &
                                (abs(p[:, 0] - point_a[0]) <= 0.1))[0]
            if len(together) > 0:
                return True  # get min and max z height for edge
    elif abs(point_a[1] - point_b[1]) <= 0.1:
        middle = (point_a[0] + point_b[0]) / 2
        for i in range(2, len(segmented_cloud), 2):
            p = np.asarray(segmented_cloud[i])
            together = np.where((abs(p[:, 0] - middle) <= 0.1) &
                                (abs(p[:, 1] - point_b[1]) <= 0.1) &
                                (abs(p[:, 1] - point_a[1]) <= 0.1))[0]
            if len(together) > 0:
                return True  # get min and max z height for edge

    return False


def get_polygon_indices(net):
    result = []

    while len(net) > 0:
        start_corner, next_corner = net[0]
        polygon = [start_corner]
        del net[0]

        while next_corner != start_corner:
            found = [(i, item) for i, item in enumerate(net) if next_corner in
                     item]
            if len(found) == 0:
                break
            index, item = found[0]
            polygon.append(next_corner)
            next_corner = net[index][1 if item[0] == next_corner else 0]
            del net[index]

        polygon.append(start_corner)
        result.append(polygon)

    return result


def save_json(filename, polygons, corners):
    data = {}
    data["name"] = (filename.split('\\')[-1]).split('.')[0]
    data["definition"] = {
        "positivemeshes": [],
        "negativemeshes": []
    }
    for p in polygons:
        data["definition"]["positivemeshes"].append({
            "polygon": corners[p][:,:2].tolist(),
            "bottom": 0.0,
            "top": 0.0,
        })
    with open('data.json', 'w') as outfile:
        json.dump(data, outfile, indent=4)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-f", type=str, required=True, metavar="FILE",
        help="path to a .pcd file"
    )
    parser.add_argument(
        "-t", type=int, default=100, metavar="THRESHOLD",
        help="minimum number of points a segmented plane should contain, "
             "default: 100"
    )
    filename = parser.parse_args().f
    threshold = parser.parse_args().t

    print("-------LOADING PCD-------")
    cloud = pcl.load(filename)

    print("-------DOWNSAMPLING-------")
    vg = cloud.make_voxel_grid_filter()
    vg.set_leaf_size(0.05, 0.05, 0.05)
    cloud = vg.filter()

    planes, segmented_cloud = segmentation(cloud, threshold)
    corners = intersection(planes)
    net = get_net(corners, segmented_cloud)
    polygon_indices = get_polygon_indices(net)
    save_json(filename, polygon_indices, corners)


if __name__ == '__main__':
    main()
