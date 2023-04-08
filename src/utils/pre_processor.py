import logging
import numpy as np

from typing import List
from sensor_msgs import point_cloud2 as pc2

logger = logging.getLogger(__name__)


def pointcloud2_to_numpy(msg, field_names: List[str] = None, skip_nans: bool = True):
    """Converts a PointCloud2 message to a structured numpy array.

    Args:
        msg (PointCloud2): The PointCloud2 message to convert.
        field_names (List[str]): The list of field names to include in the structured numpy array.
        skip_nans (bool): Whether to skip NaN values when converting the PointCloud2 message to a numpy array.
    """
    pcd_list = pc2.read_points_list(msg, field_names=field_names, skip_nans=skip_nans)
    pcd = np.array(pcd_list)
    return pcd.reshape(msg.height, msg.width, -1)
