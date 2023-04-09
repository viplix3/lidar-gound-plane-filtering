import numpy as np
from typing import Tuple


def ground_plane_filter(pcd_numpy: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Removes the ground plane from the point cloud data

    Employed ground plane filtering algorithms:
        - None

    Args:
        pcd_numpy (np.ndarray): Point cloud data in numpy array

    Returns:
        np.ndarray: Point cloud data containing only the ground plane
        np.ndarray: Filtered point cloud data with the ground plane removed
    """
    return np.empty_like(pcd_numpy), pcd_numpy
