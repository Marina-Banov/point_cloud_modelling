import numpy as np
import pcl
import pcl.pcl_visualization
import utils


def main():
    filenames = [
        './20211130124103.pcd',
        './20211201113849.pcd',
        './20211201113930.pcd',
        './20211201114059.pcd',
        './20211201114416.pcd',
        './20211201114528.pcd',
        './20211201114543.pcd',
        './20211201114646.pcd',
        './20211201115803.pcd',
    ]

    result = np.empty((0, 3), dtype=np.float32)

    for f in filenames:
        cloud = pcl.load(f)
        result = np.append(result, np.asarray(cloud), axis=0)
        print(f'joined {f}: {len(result)} points')

    result = pcl.PointCloud(result)
    vg = result.make_voxel_grid_filter()
    vg.set_leaf_size(0.01, 0.01, 0.01)
    result = vg.filter()
    result.to_file("house.pcd".encode('utf-8'), ascii=False)
    utils.visualize(np.asarray(result))


if __name__ == '__main__':
    main()
