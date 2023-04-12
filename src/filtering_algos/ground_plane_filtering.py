import logging
import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2

from typing import Tuple, Dict
from dataclasses import dataclass, field
from sensor_msgs.msg import PointCloud2

from utils.pre_processing import convert_pc2_to_o3d_xyz

logger = logging.getLogger(__name__)


@dataclass
class GroundPlaneFilter:
    """Ground plane filtering class

    Args:
        ransac_frequency (int): No. of frames to skip before running RANSAC again
        min_points_to_fit (int): Minimum no. of points required to fit a plane
        distance_threshold (float): Distance threshold for RANSAC
        num_iterations (int): No. of iterations for RANSAC
        seed_sampling (SamplingMethod): Sampling method for RANSAC seed points
        num_samples (int): No. of samples to be used for RANSAC seed points
        surface_normal_sampling_params (Dict): Parameters for surface normal sampling
        horizontal_sampling_params (Dict): Parameters for horizontal sampling
    """

    ransac_frequency: int
    min_points_to_fit: int
    distance_threshold: float
    num_iterations: int
    seed_sampling_method: str
    surface_normal_sampling_params: field(default_factory=Dict)
    horizontal_sampling_params: field(default_factory=Dict)
    random_sampling_params: field(default_factory=Dict)

    def __post_init__(self):
        self.frame_counter = 0
        self.plane_model = None

        if self.seed_sampling_method == "surface_normals":
            self.sample_ransac_seed = self.surface_normal_sampling
        elif self.seed_sampling_method == "horizontal_angle":
            self.sample_ransac_seed = self.horizontal_sampling
        else:
            logger.warning("Using random sampling for RANSAC seed points")
            self.sample_ransac_seed = self.random_sampling

    def filter(self, pcd: PointCloud2) -> Tuple[PointCloud2, PointCloud2]:
        """Removes the ground plane from the point cloud data

        Employed ground plane filtering algorithms:
            RANSAC to get initial ground plane estimate

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

        Returns:
            Tuple[PointCloud2, PointCloud2]: Ground plane point cloud data, non-ground plane point cloud data
        """
        pcd_all_fields = list(pc2.read_points(pcd, field_names=None, skip_nans=False))
        pcd_o3d = convert_pc2_to_o3d_xyz(pcd)

        if self.frame_counter % self.ransac_frequency == 0:
            seed_indices = self.sample_ransac_seed(pcd_o3d)
            ransac_input = pcd_o3d.select_by_index(seed_indices)
            self.plane_model, inlier_indices_ransac = ransac_input.segment_plane(
                distance_threshold=self.distance_threshold,
                ransac_n=self.min_points_to_fit,
                num_iterations=self.num_iterations,
            )

        points = np.asarray(pcd_o3d.points)
        distances = np.abs(
            np.dot(
                points,
                np.array(
                    [self.plane_model[0], self.plane_model[1], self.plane_model[2]]
                ),
            )
            + self.plane_model[3]
        )
        distances /= np.linalg.norm(self.plane_model[:3])
        ground_indices = np.where(distances < self.distance_threshold)[0]
        inlier_indices = set(ground_indices)

        ground_plane_pts = [
            pt for pt_idx, pt in enumerate(pcd_all_fields) if pt_idx in inlier_indices
        ]
        non_ground_plane_pts = [
            pt
            for pt_idx, pt in enumerate(pcd_all_fields)
            if pt_idx not in inlier_indices
        ]

        ground_plane_pcd = pc2.create_cloud(pcd.header, pcd.fields, ground_plane_pts)
        non_ground_plane_pcd = pc2.create_cloud(
            pcd.header, pcd.fields, non_ground_plane_pts
        )

        self.frame_counter += 1
        if logger.isEnabledFor(logging.DEBUG):
            self.visualize(ground_plane_pcd, non_ground_plane_pcd)
        self.visualize(ground_plane_pcd, non_ground_plane_pcd)

        return ground_plane_pcd, non_ground_plane_pcd

    def surface_normal_sampling(self, pcd_o3d: o3d.geometry.PointCloud):
        """Samples points from the point cloud based on surface normals

        Args:
            pcd_o3d (o3d.geometry.PointCloud): Point cloud data in Open3D format

        Returns:
            List[int]: Indices of the sampled points
        """
        sampled_point_indices = []

        pcd_o3d.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.surface_normal_sampling_params["radius"],
                max_nn=self.surface_normal_sampling_params["max_nn"],
            )
        )
        normals = np.asarray(pcd_o3d.normals)

        for idx, normal in enumerate(normals):
            if normal[2] > self.surface_normal_sampling_params["selection_threshold"]:
                sampled_point_indices.append(idx)

        return sampled_point_indices

    def horizontal_sampling(self, pcd_o3d: o3d.geometry.PointCloud):
        """Samples points from the point cloud based on horizontal angles

        Args:
            pcd_o3d (o3d.geometry.PointCloud): Point cloud data in Open3D format

        Returns:
            List[int]: Indices of the sampled points
        """
        sampled_point_indices = []

        points = np.asarray(pcd_o3d.points)
        for idx, point in enumerate(points):
            if (
                np.arctan2(point[1], point[0])
                > self.horizontal_sampling_params["min_angle"]
                and np.arctan2(point[1], point[0])
                < self.horizontal_sampling_params["max_angle"]
            ):
                sampled_point_indices.append(idx)

        return sampled_point_indices

    def random_sampling(self, pcd_o3d: o3d.geometry.PointCloud):
        """Samples points from the point cloud randomly

        Args:
            pcd_o3d (o3d.geometry.PointCloud): Point cloud data in Open3D format

        Returns:
            List[int]: Indices of the sampled points
        """
        sampled_point_indices = np.random.choice(
            len(pcd_o3d.points),
            self.random_sampling_params["num_samples"],
            replace=False,
        )

        return sampled_point_indices

    def visualize(
        self, ground_plane_pcd: PointCloud2, non_ground_plane_pcd: PointCloud2
    ):
        """Visualizes the ground plane and non-ground plane point cloud data
            Ground plane point cloud data is colored red

        Args:
            ground_plane_pcd (PointCloud2): Ground plane point cloud data
            non_ground_plane_pcd (PointCloud2): Non-ground plane point cloud data
        """
        ground_plane_pcd_o3d = convert_pc2_to_o3d_xyz(ground_plane_pcd)
        non_ground_plane_pcd_o3d = convert_pc2_to_o3d_xyz(non_ground_plane_pcd)
        ground_plane_pcd_o3d.paint_uniform_color([1, 0, 0])
        o3d.visualization.draw_geometries(
            [ground_plane_pcd_o3d, non_ground_plane_pcd_o3d]
        )
