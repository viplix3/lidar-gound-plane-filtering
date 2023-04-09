import logging
import numpy as np

logger = logging.getLogger(__name__)


def pre_processor(pcd_numpy: np.ndarray) -> np.ndarray:
    """Pre-processes the point cloud data.

    Employed pre-processing algorithms:
        - None

    Args:
        pcd_numpy (np.ndarray): Point cloud data in numpy array

    Returns:
        np.ndarray: Pre-processed point cloud data in numpy array"""
    return pcd_numpy
