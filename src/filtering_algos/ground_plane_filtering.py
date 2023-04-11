import logging
import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2

from typing import Tuple, Dict
from sensor_msgs.msg import PointCloud2

from utils.pre_processing import convert_pc2_to_o3d_xyz

logger = logging.getLogger(__name__)


def ground_plane_filter(
    pcd: PointCloud2, params: Dict
) -> Tuple[PointCloud2, PointCloud2]:
    """Removes the ground plane from the point cloud data

    Employed ground plane filtering algorithms:
        RANSAC to get initial ground plane estimate (Look into adaptive RANSAC)
            Initial seed for RANSAC
                Do a quantile based analysis of the point cloud data, use the points
                    towards the bottom of the point cloud as the initial seed (z-axis)
                Make sure the points in initial seed don't have unusually negative z-values,
                    i.e. don't want to start with z-min points as they could be noise
        Estimate additional features for initial ground plane points
            Surface Normals
                Points on ground plane should have normals close to (0, 0, -1)
                    assuming a relatively flat ground plane
            Analyze intensity and reflectivity (Quantile based)
                Points with low intensity could be noise, eliminate them
                Points with high reflectivity could be noise, eliminate them
                Intensity and Reflectivity Gradient (look into it, might help in removing noise)
            Range Gradient and Range Curvature
                Know this is a feature used in a lot of other algorithms, need to study it
            KNN to get distribution of points around the ground plane
                Points on ground plane should have a high density of points around them
                Neighbors of points on ground plane should be close to each other in z-direction
                    again, assuming a relatively flat ground plane
        Use the features to remove outliers
        DBSCAN clustering based ground plane refinement (Look into HDBSCAN)
            Try a feature pyramid approach here, that can help in handling ground plane that is not flat
            Divide the initial ground plane into multiple bins based on concentric circles
            Apply DBSCAN clustering to each bin
            Merge the bins based on z-gradients
            This would allow us to handle ground plane that is not flat
        PostProcessing to improve the ground plane estimate
            Remove isolated clusters of small points
            Merge clusters of points that are close to each other in z-direction
            Fill holes in the ground plane (Morphological operations)

    Args:
        pcd (PointCloud2): Point cloud data in a ROS PointCloud2 format
            The shape is (H, W) where H is the height, W is the width
            Each point is represented by 9 values (x, y, z, intensity, time, reflectivity, ring, ambient, range)
        params (Dict): Parameters for the noise filter

    Returns:
        Tuple[PointCloud2, PointCloud2]: Ground plane point cloud data, non-ground plane point cloud data
    """
    pcd_all_fields = list(pc2.read_points(pcd, field_names=None, skip_nans=False))
    pcd_o3d = convert_pc2_to_o3d_xyz(pcd)

    plane_model, inlier_indices = pcd_o3d.segment_plane(
        distance_threshold=params["RANSAC"]["distance_threshold"],
        ransac_n=params["RANSAC"]["min_points_to_fit"],
        num_iterations=params["RANSAC"]["num_iterations"],
    )

    [a, b, c, d] = plane_model
    logger.debug(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    # O3D Visualization
    inlier_cloud = pcd_o3d.select_by_index(inlier_indices)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd_o3d.select_by_index(inlier_indices, invert=True)
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

    inlier_indices = set(inlier_indices)
    ground_plane_pts = [
        pt for pt_idx, pt in enumerate(pcd_all_fields) if pt_idx in inlier_indices
    ]
    non_ground_plane_pts = [
        pt for pt_idx, pt in enumerate(pcd_all_fields) if pt_idx not in inlier_indices
    ]

    ground_plane_pcd = pc2.create_cloud(pcd.header, pcd.fields, ground_plane_pts)
    non_ground_plane_pcd = pc2.create_cloud(
        pcd.header, pcd.fields, non_ground_plane_pts
    )
    return ground_plane_pcd, non_ground_plane_pcd
