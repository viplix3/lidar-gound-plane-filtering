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
    # From the analysis of the point cloud data and ROS bag files, it is evident that
    # the LiDAR used is Ouster OS2 and we can use the datasheet to filter out the noise
    # However, even if the datasheet wasn't available, we would have been able to filter out the noise
    # By looking at the point cloud data and the ROS bag files, it was clear that the there were
    # points that were below the first rang of points, hence, these points were noise and they could
    # be removed without losing any important information

    # OUSTER OS2 LiDAR Data Sheet: https://data.ouster.io/downloads/datasheets/datasheet-revd-v2p0-os2.pdf

    # According to the datasheet, the minimum range is 1m while the maximum range is 240m
    # Precision of the range as given in the datasheet is:
    # 1 - 30 m: ± 2.5 cm
    # 30 - 60 m: ± 4 cm
    # >60 m: ± 8 cm

    # Considering this, all the points that are there below 1m (+/- 2.5cm) are noise and they can be removed
    # without losing any important information
    pcd_all_fields = list(pc2.read_points(pcd, field_names=None, skip_nans=False))
    points_xy = list(pc2.read_points(pcd, field_names=("x", "y"), skip_nans=False))

    min_range = params.get("min_range", 1.0 - 0.025)  # 1m - 2.5cm
    max_range = params.get("max_range", 240.0)  # 240m

    range_data = [np.linalg.norm(point) for point in points_xy]
    noise_points = [
        idx for idx, rng in enumerate(range_data) if rng < min_range or rng > max_range
    ]

    # Preserve the other fields for the filtered points
    filtered_points_all_fields = [
        pt for pt_idx, pt in enumerate(pcd_all_fields) if pt_idx not in noise_points
    ]
    noise_points_all_fields = [
        pt for pt_idx, pt in enumerate(pcd_all_fields) if pt_idx in noise_points
    ]

    logger.debug(f"Filtered points: {len(filtered_points_all_fields)}")
    logger.debug(f"Noise points: {len(noise_points_all_fields)}")

    # O3d visualization
    # filtered_pcd_o3d = pcd_o3d.select_by_index(noise_points, invert=True)
    # noise_pcd_o3d = pcd_o3d.select_by_index(noise_points)
    filtered_points_xyz = np.array(
        [
            pt[:3]
            for pt_idx, pt in enumerate(pcd_all_fields)
            if pt_idx not in noise_points
        ]
    )
    filtered_pcd_o3d = o3d.geometry.PointCloud()
    filtered_pcd_o3d.points = o3d.utility.Vector3dVector(filtered_points_xyz)

    noise_points_xyz = np.array(
        [pt[:3] for pt_idx, pt in enumerate(pcd_all_fields) if pt_idx in noise_points]
    )
    noise_pcd_o3d = o3d.geometry.PointCloud()
    noise_pcd_o3d.points = o3d.utility.Vector3dVector(noise_points_xyz)

    filtered_pcd_o3d.paint_uniform_color([1.0, 0, 0])
    o3d.visualization.draw_geometries([filtered_pcd_o3d, noise_pcd_o3d])

    pcd_filtered = pc2.create_cloud(pcd.header, pcd.fields, filtered_points_all_fields)
    pcd_noise = pc2.create_cloud(pcd.header, pcd.fields, noise_points_all_fields)
    return pcd_noise, pcd_filtered
