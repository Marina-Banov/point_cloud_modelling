import argparse
import pcl
import numpy as np
from utils import VisualizeType, get, visualize, setup_segmenter


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", "-f", type=str, required=True)
    filename = parser.parse_args().file

    print("-------LOADING PCD-------")
    cloud = pcl.load(filename)

    seg = setup_segmenter(cloud, 0, 0, 1)
    indices, coefficients = seg.segment()
    print(coefficients)
    result = get(cloud, indices, VisualizeType.CONCAVE_HULL)
    visualize(np.asarray(result))


if __name__ == '__main__':
    main()
