import logging
import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2

from typing import Tuple, Dict
from sensor_msgs.msg import PointCloud2

from utils.pre_processing import convert_pc2_to_o3d_xyz

logger = logging.getLogger(__name__)


def noise_filter(pcd: PointCloud2, params: Dict) -> Tuple[PointCloud2, PointCloud2]:
    """Removes noise from the point cloud data

    Employed noise filtering algorithms:
        Remove all points beyond a certain LiDAR measurement range

    Args:
        pcd (PointCloud2): Point cloud data in a ROS PointCloud2 format
            The shape is (H, W) where H is the height, W is the width
            Each point is represented by 9 values (x, y, z, intensity, time, reflectivity, ring, ambient, range)
        params (Dict): Parameters for the noise filter

    Returns:
        Tuple[PointCloud2, PointCloud2]: Noise PointCloud2, Filtered PointCloud2
    """
    noise_points_all_fields = []
    filtered_points_all_fields = []
    min_range = params["min_range"]
    max_range = params["max_range"]
    pcd_all_fields = list(pc2.read_points(pcd, field_names=None, skip_nans=False))

    for pt in pcd_all_fields:
        dist_from_origin = calc_dist_from_origin(pt)
        if min_range <= dist_from_origin <= max_range:
            filtered_points_all_fields.append(pt)
        else:
            noise_points_all_fields.append(pt)

    pcd_filtered = pc2.create_cloud(pcd.header, pcd.fields, filtered_points_all_fields)
    pcd_noise = pc2.create_cloud(pcd.header, pcd.fields, noise_points_all_fields)
    return pcd_noise, pcd_filtered


def calc_dist_from_origin(pt_xyz: Tuple[float, float, float]) -> float:
    """Calculates the distance of a point from the origin

    Args:
        pt_xyz (Tuple[float, float, float]): Point in the form of (x, y, z)

    Returns:
        float: Distance of the point from the origin
    """
    return np.sqrt(pt_xyz[0] ** 2 + pt_xyz[1] ** 2 + pt_xyz[2] ** 2)
