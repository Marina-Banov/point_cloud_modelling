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

    print("-------DOWNSAMPLING-------")
    vg = cloud.make_voxel_grid_filter()
    vg.set_leaf_size(0.03, 0.03, 0.03)
    cloud = vg.filter()

    utils.visualize(np.asarray(cloud))


if __name__ == '__main__':
    main()
