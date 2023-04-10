import logging
import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2

from typing import Dict
from sensor_msgs.msg import PointCloud2


logger = logging.getLogger(__name__)


def pre_processor(pcd: PointCloud2, filtering_params: Dict) -> PointCloud2:
    """Pre-processes the point cloud data.

    Employed pre-processing algorithms:
        Statistical outlier filter
            (Tentative: Radius outlier filter)
            (Tentative: Bilateral filter)

    Args:
        pcd (PointCloud2): Point cloud data in a ROS PointCloud2 format
            The shape is (H, W) where H is the height, W is the width
            Each point is represented by 9 values (x, y, z, intensity, time, reflectivity, ring, ambient, range)

    Returns:
        PointCloud2: Pre-processed point cloud data
    """
    filtered_pcd = statistical_outlier_removal(
        pcd, filtering_params["statistical_outlier_removal"]
    )
    filtered_pcd = radius_outlier_removal(
        filtered_pcd, filtering_params["radius_outlier_removal"]
    )
    return filtered_pcd


def statistical_outlier_removal(pcd: PointCloud2, params: Dict) -> PointCloud2:
    """Filters out outliers in the point cloud data using statistical outlier filter.

    Args:
        pcd (PointCloud2): Point cloud data in a ROS PointCloud2 format
            The shape is (H, W) where H is the height, W is the width
            Each point is represented by 9 values (x, y, z, intensity, time, reflectivity, ring, ambient, range)
        params (Dict): Parameters for the statistical outlier filter
    Returns:
        PointCloud2: Filtered point cloud data
    """

    pcd_all_fields = list(pc2.read_points(pcd, field_names=None, skip_nans=False))
    pcd_xyz = pc2.read_points(pcd, field_names=("x", "y", "z"), skip_nans=True)
    pcd_o3d = o3d.geometry.PointCloud()
    pcd_o3d.points = o3d.utility.Vector3dVector(pcd_xyz)

    # Define statistical outlier filter parameters
    nb_neighbors = params["nb_neighbors"] if "nb_neighbors" in params else 20
    std_ratio = params["std_ratio"] if "std_ratio" in params else 2.0

    # Apply statistical outlier filter
    pcd_o3d, inlier_indices = pcd_o3d.remove_statistical_outlier(
        nb_neighbors=nb_neighbors, std_ratio=std_ratio
    )

    # Preserve the other fields for the filtered points
    inlier_indices = set(inlier_indices)
    filtered_points_all_fields = [
        pt for pt_idx, pt in enumerate(pcd_all_fields) if pt_idx in inlier_indices
    ]
    pcd_filtered = pc2.create_cloud(pcd.header, pcd.fields, filtered_points_all_fields)
    return pcd_filtered


def radius_outlier_removal(pcd: PointCloud2, params: Dict) -> PointCloud2:
    """Filters out outliers in the point cloud data using radius outlier filter.

    Args:
        pcd (PointCloud2): Point cloud data in a ROS PointCloud2 format
            The shape is (H, W) where H is the height, W is the width
            Each point is represented by 9 values (x, y, z, intensity, time, reflectivity, ring, ambient, range)
        params (Dict): Parameters for the radius outlier filter
    Returns:
        PointCloud2: Filtered point cloud data
    """
    return pcd
