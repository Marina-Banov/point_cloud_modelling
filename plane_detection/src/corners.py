import argparse
import pcl
from scipy.spatial import distance
import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt
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
    chull = np.asarray(get(cloud, indices, VisualizeType.CONCAVE_HULL, alpha=0.5))

    print("-------CALCULATING MEAN AND DIST-------")
    centroid = chull.mean(axis=0)
    print(centroid)

    dist = distance.cdist([centroid], chull, 'euclidean')[0]
    dist = np.insert(dist, 0, 0, axis=0)
    corners, _ = find_peaks(dist, prominence=0.5)
    for i in corners:
        print(chull[i])

    visualize(np.take(chull, corners, axis=0))

    plt.plot(list(range(dist.shape[0])), dist, 'b.')
    plt.plot(corners, dist[corners], 'ro', markersize=8)
    plt.xlabel('index')
    plt.ylabel('distance')
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    main()
