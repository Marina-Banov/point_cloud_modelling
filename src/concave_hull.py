import argparse
import pcl
import numpy as np
import utils


def concave_hull(cloud):
    seg = utils.setup_segmenter(cloud, 0, 0, 1)
    indices, coefficients = seg.segment()
    print(coefficients)
    result = utils.get(cloud, indices, utils.VisualizeType.CONCAVE_HULL)
    utils.visualize(np.asarray(result))
    return result


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
    vg.set_leaf_size(0.02, 0.02, 0.02)
    cloud = vg.filter()

    concave_hull(cloud)


if __name__ == '__main__':
    main()
