import argparse
import numpy as np
import pcl
import pcl.pcl_visualization


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", "-f", type=str, required=True)
    
    print("-------LOADING PCD-------")
    filename = parser.parse_args().file
    cloud = pcl.load(filename)

    print("-------COMPUTING MODEL-------")
    model_p = pcl.SampleConsensusModelPlane(cloud)
    ransac = pcl.RandomSampleConsensus(model_p)
    ransac.set_DistanceThreshold(.017)
    ransac.computeModel()

    print("-------GETTING INLIERS-------")
    inliers = ransac.get_Inliers()

    if len(inliers) != 0:
        print("-------GETTING FINAL POINTS-------")
        finalpoints = np.zeros((len(inliers), 3), dtype=np.float32)
        
        for i in range(0, len(inliers)):
            finalpoints[i][0] = cloud[inliers[i]][0]
            finalpoints[i][1] = cloud[inliers[i]][1]
            finalpoints[i][2] = cloud[inliers[i]][2]

        final = pcl.PointCloud()
        final.from_array(finalpoints)

        print("-------GETTING CONCAVE HULL-------")
        chull = final.make_ConcaveHull()
        # chull.set_Alpha(0.5)
        chull.set_Alpha(0.07)
        cloud_hull = chull.reconstruct()

        # creates the visualization object and adds all of the inliers
        print("-------VISUALIZING-------")
        viewer = pcl.pcl_visualization.PCLVisualizering('3D Viewer')
        viewer.SetBackgroundColor(0, 0, 0)
        viewer.AddPointCloud(cloud_hull, b'inliers cloud')
        viewer.SetPointCloudRenderingProperties(pcl.pcl_visualization.PCLVISUALIZER_POINT_SIZE, 3, b'inliers cloud')
        viewer.InitCameraParameters()

        while not viewer.WasStopped():
            viewer.SpinOnce(100)


if __name__ == '__main__':
    main()
