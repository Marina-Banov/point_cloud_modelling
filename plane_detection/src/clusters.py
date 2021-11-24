import argparse
import pcl
import numpy as np
from utils import visualize, get, VisualizeType, setup_segmenter


def clusters(cloud):
    seg = setup_segmenter(cloud, 0, 0, 1)
    indices, coefficients = seg.segment()
    print(coefficients)
    cloud = get(cloud, indices, VisualizeType.CONCAVE_HULL)

    print("-------KDTREE-------")
    tree = cloud.make_kdtree()
    ec = cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.2)
    ec.set_MinClusterSize(20)
    ec.set_MaxClusterSize(2200)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    cloud_cluster = pcl.PointCloud()
    clusters_array = []

    for j, indices in enumerate(cluster_indices):
        print("-------EXTRACTING CLUSTER-------")
        points = np.zeros((len(indices), 3), dtype=np.float32)

        for i, index in enumerate(indices):
            points[i][0] = cloud[index][0]
            points[i][1] = cloud[index][1]
            points[i][2] = cloud[index][2]

        visualize(points)
        cloud_cluster.from_array(points)
        clusters_array.append(cloud_cluster)
        pcl.save(cloud_cluster, f'cloud_cluster_{j}.pcd')

    return clusters_array


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", type=str, required=True, metavar="FILE", help="path to a .pcd file")
    filename = parser.parse_args().f

    print("-------LOADING PCD-------")
    cloud = pcl.load(filename)

    print("-------DOWNSAMPLING-------")
    vg = cloud.make_voxel_grid_filter()
    vg.set_leaf_size(0.02, 0.02, 0.02)
    cloud = vg.filter()

    clusters(cloud)


if __name__ == '__main__':
    main()
