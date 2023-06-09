import time
import logging
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2

from typing import Dict
from sensor_msgs.msg import PointCloud2

logger = logging.getLogger(__name__)


def pre_processor(pcd: PointCloud2, filtering_params: Dict) -> PointCloud2:
    """Pre-processes the point cloud data.

    Employed pre-processing algorithms:
        Statistical outlier filter
        Radius outlier filter

    Args:
        pcd (PointCloud2): Point cloud data in a ROS PointCloud2 format
            The shape is (H, W) where H is the height, W is the width
            Each point is represented by 9 values (x, y, z, intensity, time, reflectivity, ring, ambient, range)

    Returns:
        PointCloud2: Pre-processed point cloud data
    """
    filtered_pcd = apply_outlier_filters(pcd, filtering_params)
    return filtered_pcd


def apply_outlier_filters(pcd: PointCloud2, filtering_params: Dict) -> PointCloud2:
    """Applies outlier filters to the point cloud data

    Args:
        pcd (PointCloud2): Point cloud data in a ROS PointCloud2 format
            The shape is (H, W) where H is the height, W is the width
            Each point is represented by 9 values (x, y, z, intensity, time, reflectivity, ring, ambient, range)
        filtering_params (Dict): Parameters for the outlier filters

    Returns:
        PointCloud2: Filtered point cloud data
    """
    pcd_all_fields = list(pc2.read_points(pcd, field_names=None, skip_nans=False))
    pcd_o3d = convert_pc2_to_o3d_xyz(pcd)

    # Apply statistical outlier filter, takes 0.5s for 1 frame on average
    tick = time.time()
    statistical_params = filtering_params["statistical_outlier_removal"]
    _, inlier_indices_sor = pcd_o3d.remove_statistical_outlier(
        nb_neighbors=statistical_params["nb_neighbors"],
        std_ratio=statistical_params["std_ratio"],
    )
    tock = time.time()
    # print("SOR time: {}".format(tock - tick))

    # Apply radius outlier filter, takes 0.8s for 1 frame on average
    tick = time.time()
    radius_params = filtering_params["radius_outlier_removal"]
    _, inlier_indices_ror = pcd_o3d.remove_radius_outlier(
        nb_points=radius_params["min_neighbors"], radius=radius_params["radius"]
    )
    tock = time.time()
    # print("ROR time: {}".format(tock - tick))

    # Preserve the other fields for the filtered points
    inlier_indices = set(inlier_indices_sor + inlier_indices_ror)
    filtered_points_all_fields = [
        pt for pt_idx, pt in enumerate(pcd_all_fields) if pt_idx in inlier_indices
    ]

    pcd_filtered = pc2.create_cloud(pcd.header, pcd.fields, filtered_points_all_fields)

    if logger.isEnabledFor(logging.DEBUG):
        # O3D Visualization
        pcd_o3d.paint_uniform_color([1.0, 0, 0])
        pcd_o3d_filtered = convert_pc2_to_o3d_xyz(pcd_filtered)
        o3d.visualization.draw_geometries([pcd_o3d, pcd_o3d_filtered])

    return pcd_filtered


def convert_pc2_to_o3d_xyz(pcd: PointCloud2) -> o3d.geometry.PointCloud:
    """Converts a ROS PointCloud2 message to an Open3D point cloud object.

    Args:
        pcd (PointCloud2): Point cloud data in a ROS PointCloud2 format
            The shape is (H, W) where H is the height, W is the width
            Each point is represented by 9 values (x, y, z, intensity, time, reflectivity, ring, ambient, range)

    Returns:
        o3d.geometry.PointCloud: Open3D point cloud object holding the point cloud data (x, y, z)
    """
    pcd_xyz = pc2.read_points(pcd, field_names=("x", "y", "z"), skip_nans=True)
    pcd_o3d = o3d.geometry.PointCloud()
    pcd_o3d.points = o3d.utility.Vector3dVector(pcd_xyz)
    return pcd_o3d
