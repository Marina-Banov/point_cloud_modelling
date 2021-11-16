import argparse
import numpy as np
import pcl
import pcl.pcl_visualization


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", "-f", type=str, required=True)
    filename = parser.parse_args().file

    print("-------LOADING PCD-------")
    cloud = pcl.load(filename)

    print("-------COMPUTING MODEL-------")
    """
    model_p = pcl.SampleConsensusModelPlane(cloud)
    ransac = pcl.RandomSampleConsensus(model_p)
    ransac.set_DistanceThreshold(.017)
    ransac.computeModel()
    inliers = ransac.get_Inliers()
    """
    seg = cloud.make_segmenter_normals(ksearch=10)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.05)
    seg.set_normal_distance_weight(0.005)
    seg.set_max_iterations(100)
    inliers, coefficients = seg.segment()
    print(coefficients)

    if len(inliers) != 0:
        cloud_points = np.full((cloud.size, 3), cloud, dtype=np.float32)
        """
        print("-------GETTING PLANE POINTS-------")
        finalpoints = np.copy(cloud_points[inliers])
        final = pcl.PointCloud()
        final.from_array(finalpoints)

        print("-------GETTING CONCAVE HULL-------")
        chull = final.make_ConcaveHull()
        # chull.set_Alpha(0.5)
        chull.set_Alpha(0.07)
        final = chull.reconstruct()
        """
        print("-------REMOVING PLANE POINTS-------")
        finalpoints = np.delete(cloud_points, inliers, axis=0)
        final = pcl.PointCloud()
        final.from_array(finalpoints)

        print("-------VISUALIZING-------")
        viewer = pcl.pcl_visualization.PCLVisualizering('3D Viewer')
        viewer.SetBackgroundColor(0, 0, 0)
        viewer.AddPointCloud(final, b'inliers cloud')
        viewer.SetPointCloudRenderingProperties(pcl.pcl_visualization.PCLVISUALIZER_POINT_SIZE, 3, b'inliers cloud')
        viewer.InitCameraParameters()

        while not viewer.WasStopped():
            viewer.SpinOnce(100)


if __name__ == '__main__':
    main()
