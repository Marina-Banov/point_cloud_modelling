from enum import Enum
import numpy as np
import pcl
import pcl.pcl_visualization


class VisualizeType(Enum):
    ONLY_INLIERS = 1
    NO_INLIERS = 2
    CONCAVE_HULL = 3


def get(cloud, inliers, v_type):
    cloud_points = np.full((cloud.size, 3), cloud, dtype=np.float32)

    if v_type == VisualizeType.NO_INLIERS:
        print("-------REMOVING PLANE POINTS-------")
        finalpoints = np.delete(cloud_points, inliers, axis=0)
        final = pcl.PointCloud()
        final.from_array(finalpoints)
    else:
        print("-------GETTING PLANE POINTS-------")
        finalpoints = np.copy(cloud_points[inliers])
        final = pcl.PointCloud()
        final.from_array(finalpoints)
        if v_type == VisualizeType.CONCAVE_HULL:
            print("-------GETTING CONCAVE HULL-------")
            chull = final.make_ConcaveHull()
            # chull.set_Alpha(0.5)
            chull.set_Alpha(0.07)
            final = chull.reconstruct()

    return final


def visualize(points):
    print("-------VISUALIZING-------")
    viewer = pcl.pcl_visualization.PCLVisualizering('3D Viewer')
    viewer.SetBackgroundColor(0, 0, 0)
    viewer.AddPointCloud(points, b'points')
    viewer.SetPointCloudRenderingProperties(pcl.pcl_visualization.PCLVISUALIZER_POINT_SIZE, 3, b'points')
    viewer.InitCameraParameters()

    while not viewer.WasStopped():
        viewer.SpinOnce(100)


def setup_segmenter(cloud, x, y, z):
    print("-------COMPUTING MODEL-------")
    seg = cloud.make_segmenter_normals(ksearch=10)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_PERPENDICULAR_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.1)
    seg.set_normal_distance_weight(0.005)
    seg.set_max_iterations(500)
    seg.set_axis(x, y, z)
    seg.set_eps_angle(np.pi/20);
    return seg
