import numpy as np
import sensor_msgs.point_cloud2 as pc2

from typing import Tuple, Dict
from sensor_msgs.msg import PointCloud2


def noise_filter(pcd: PointCloud2, params: Dict) -> Tuple[PointCloud2, PointCloud2]:
    """Removes noise from the point cloud data

    Employed noise filtering algorithms:
        (Tentative: Remove all points before the ring of the first scan)

    Args:
        pcd (PointCloud2): Point cloud data in a ROS PointCloud2 format
            The shape is (H, W) where H is the height, W is the width
            Each point is represented by 9 values (x, y, z, intensity, time, reflectivity, ring, ambient, range)
        params (Dict): Parameters for the noise filter

    Returns:
        Tuple[PointCloud2, PointCloud2]: Noise PointCloud2, Filtered PointCloud2
    """
    return PointCloud2(), pcd
