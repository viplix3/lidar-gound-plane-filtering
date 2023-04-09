import logging
import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2


logger = logging.getLogger(__name__)


def pre_processor(pcd: PointCloud2) -> PointCloud2:
    """Pre-processes the point cloud data.

    Employed pre-processing algorithms:
        - None

    Args:
        pcd (PointCloud2): Point cloud data in a ROS PointCloud2 format
            The shape is (H, W) where H is the height, W is the width
            Each point is represented by 9 values (x, y, z, intensity, time, reflectivity, ring, ambient, range)

    Returns:
        PointCloud2: Pre-processed point cloud data
    """
    filtered_pcd = statistical_outlier_filter(pcd)
    return filtered_pcd


def statistical_outlier_filter(pcd: PointCloud2):
    """Filters out outliers in the point cloud data using statistical outlier filter.

    Args:
        pcd (PointCloud2): Point cloud data in a ROS PointCloud2 format
            The shape is (H, W) where H is the height, W is the width
            Each point is represented by 9 values (x, y, z, intensity, time, reflectivity, ring, ambient, range)
    """

    pcd_xyz = pc2.read_points(pcd, field_names=("x", "y", "z"), skip_nans=True)
    pcd_o3d = o3d.geometry.PointCloud()
    pcd_o3d.points = o3d.utility.Vector3dVector(pcd_xyz)

    # Define statistical outlier filter parameters
    nb_neighbors = 20
    std_ratio = 2.0

    # Apply statistical outlier filter
    pcd_o3d, ind = pcd_o3d.remove_statistical_outlier(
        nb_neighbors=nb_neighbors, std_ratio=std_ratio
    )

    # Extract the filtered points from pcd_o3d
    filtered_xyz = np.asarray(pcd_o3d.points)

    # Create a new PointCloud2 message with the filtered points
    pcd_filtered = pc2.create_cloud_xyz32(pcd.header, filtered_xyz)

    return pcd_filtered
