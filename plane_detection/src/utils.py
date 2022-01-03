from enum import Enum
import numpy as np
import pcl
import pcl.pcl_visualization
import open3d as o3d


class VisualizeType(Enum):
    ONLY_INLIERS = 1
    NO_INLIERS = 2
    CONCAVE_HULL = 3


def get(cloud, inliers, v_type, **kwargs):
    default_kwargs = {'alpha': 0.07}
    kwargs = {**default_kwargs, **kwargs}

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
            chull.set_Alpha(kwargs['alpha'])
            final = chull.reconstruct()

    return final


def visualize(points):
    print("-------VISUALIZING-------")
    vis = o3d.geometry.PointCloud()
    vis.points = o3d.utility.Vector3dVector(list(points[:, :3]))
    if points.shape[1] == 6:
        vis.colors = o3d.utility.Vector3dVector(list(points[:, 3:]))
    o3d.visualization.draw_geometries([vis])


def add_color_to_points(points, color):
    points = np.asarray(points)
    res = np.zeros((points.shape[0], 6))
    res[:, :3] = points
    res[:, 3:] = color
    return res


def setup_segmenter(cloud, x, y, z, **kwargs):
    default_kwargs = {
        'ksearch': 20,
        'distance_threshold': 0.1,
        'normal_distance_weight': 0.01,
        'max_iterations': 1000,
        'eps_angle': np.pi / 90,
    }
    kwargs = {**default_kwargs, **kwargs}

    seg = cloud.make_segmenter_normals(ksearch=kwargs['ksearch'])
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_PERPENDICULAR_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(kwargs['distance_threshold'])
    seg.set_normal_distance_weight(kwargs['normal_distance_weight'])
    seg.set_max_iterations(kwargs['max_iterations'])
    seg.set_axis(x, y, z)
    seg.set_eps_angle(kwargs['eps_angle'])
    return seg
