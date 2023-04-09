import numpy as np
from typing import Tuple


def noise_filter(pcd_numpy: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Removes noise from the point cloud data

    Employed noise filtering algorithms:
        - None

    Args:
        pcd_numpy (np.ndarray): Point cloud data in numpy array

    Returns:
        np.ndarray: Point cloud data containing only the noise
        np.ndarray: Filtered point cloud data with the noise removed
    """
    return np.empty_like(pcd_numpy), pcd_numpy
