import logging
import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2

from typing import Tuple, Dict
from sensor_msgs.msg import PointCloud2
from enum import Enum

from utils.pre_processing import convert_pc2_to_o3d_xyz

logger = logging.getLogger(__name__)


class SamplingMethod(Enum):
    """Enum for the sampling method to be used for RANSAC seed points"""

    SURFACE_NORMALS = "surface_normals"
    HORIZONTAL_GRADIENT = "horizontal_gradient"


def ground_plane_filter(
    pcd: PointCloud2, params: Dict
) -> Tuple[PointCloud2, PointCloud2]:
    """Removes the ground plane from the point cloud data

    Employed ground plane filtering algorithms:
        RANSAC to get initial ground plane estimate (Look into adaptive RANSAC)
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

    # RANSAC ground plane estimation with initial seed
    seed_indices = sample_ransac_seed(pcd_o3d, params["RANSAC"]["seed_sampling"])
    ransac_input = pcd_o3d.select_by_index(seed_indices)
    plane_model, inlier_indices_ransac = ransac_input.segment_plane(
        distance_threshold=params["RANSAC"]["distance_threshold"],
        ransac_n=params["RANSAC"]["min_points_to_fit"],
        num_iterations=params["RANSAC"]["num_iterations"],
    )

    # Segregate ground plane points on the basis of RANSAC
    inlier_indices = set(inlier_indices_ransac)
    ground_plane_pts = [
        pt for pt_idx, pt in enumerate(pcd_all_fields) if pt_idx in inlier_indices
    ]
    non_ground_plane_pts = [
        pt for pt_idx, pt in enumerate(pcd_all_fields) if pt_idx not in inlier_indices
    ]

    # DBSCAN clustering for ground plane refinement
    # cluster_labels = o3d.geometry.PointCloud.cluster_dbscan(
    #     ground_plane_pcd, eps=0.2, min_points=3, print_progress=True
    # )
    # filtered_indices = np.where(cluster_labels != -1)[0]
    # ground_plane_pcd = ground_plane_pcd.select_by_index(filtered_indices)
    # ground_plane_pcd.paint_uniform_color([0, 1.0, 0])
    # o3d.visualization.draw_geometries([ground_plane_pcd, non_ground_plane_pcd])

    # # Convert back to ROS PointCloud2
    # ground_plane_pcd_ros = pc2.create_cloud_xyz32(
    #     pcd.header, np.asarray(ground_plane_pcd.points)
    # )
    # non_ground_plane_pcd_ros = pc2.create_cloud_xyz32(
    #     pcd.header, np.asarray(non_ground_plane_pcd.points)
    # )

    ground_plane_pcd = pc2.create_cloud(pcd.header, pcd.fields, ground_plane_pts)
    non_ground_plane_pcd = pc2.create_cloud(
        pcd.header, pcd.fields, non_ground_plane_pts
    )

    ground_plane_pcd_o3d = convert_pc2_to_o3d_xyz(ground_plane_pcd)
    non_ground_plane_pcd_o3d = convert_pc2_to_o3d_xyz(non_ground_plane_pcd)
    ground_plane_pcd_o3d.paint_uniform_color([1.0, 0, 0])
    o3d.visualization.draw_geometries([ground_plane_pcd_o3d, non_ground_plane_pcd_o3d])

    return ground_plane_pcd, non_ground_plane_pcd


def sample_ransac_seed(
    pcd_o3d: o3d.geometry.PointCloud, sampling_params: Dict
) -> o3d.geometry.PointCloud:
    """Samples points from the point cloud to be used as the initial seed for RANSAC ground plane estimation

    Args:
        pcd_o3d (o3d.geometry.PointCloud): Point cloud data in an Open3D PointCloud format
        params (Dict): Parameters for the noise filter

    Returns:
        o3d.geometry.PointCloud: Point cloud data in an Open3D PointCloud format
    """
    sampled_point_indices = []
    sampling_method = SamplingMethod(sampling_params["method"])

    if sampling_method == SamplingMethod.SURFACE_NORMALS:
        sampling_params = sampling_params["surface_normals"]
        pcd_o3d.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=sampling_params["radius"],
                max_nn=sampling_params["max_nn"],
            )
        )
        normals = np.asarray(pcd_o3d.normals)

        # Points on ground plane should have normals close to (0, 0, -1)
        for idx, normal in enumerate(normals):
            if normal[2] > sampling_params["selection_threshold"]:
                sampled_point_indices.append(idx)

    elif sampling_method == SamplingMethod.HORIZONTAL_GRADIENT:
        sampling_params = sampling_params["horizontal"]
        pass
    else:
        logger.warning("Invalid sampling method for RANSAC seed")
        logger.warning("Using random sampling method instead")
        sampled_point_indices = np.random.choice(
            len(pcd_o3d.points), sampling_params["num_samples"], replace=False
        )

    return sampled_point_indices
