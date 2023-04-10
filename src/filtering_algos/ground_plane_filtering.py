import numpy as np
from typing import Tuple


def ground_plane_filter(pcd_numpy: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Removes the ground plane from the point cloud data

    Employed ground plane filtering algorithms:
        RANSAC to get initial ground plane estimate (Look into adaptive RANSAC)
            Initial seed for RANSAC
                Do a quantile based analysis of the point cloud data, use the points
                    towards the bottom of the point cloud as the initial seed (z-axis)
                Make sure the points in initial seed don't have unusually negative z-values,
                    i.e. don't want to start with z-min points as they could be noise
        Estimate additional features for initial ground plane points
            Surface Normals
                Points on ground plane should have normals close to (0, 0, -1)
                    assuming a relatively flat ground plane
            Analyze intensity and reflectivity (Quantile based)
                Points with low intensity could be noise, eliminate them
                Points with high reflectivity could be noise, eliminate them
                Intensity and Reflectivity Gradient (look into it, might help in removing noise)
            Range Gradient and Range Curvature
                Know this is a feature used in a lot of other algorithms, need to study it
            KNN to get distribution of points around the ground plane
                Points on ground plane should have a high density of points around them
                Neighbors of points on ground plane should be close to each other in z-direction
                    again, assuming a relatively flat ground plane
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
        pcd_numpy (np.ndarray): Point cloud data in numpy array

    Returns:
        np.ndarray: Point cloud data containing only the ground plane
        np.ndarray: Filtered point cloud data with the ground plane removed
    """
    return np.empty_like(pcd_numpy), pcd_numpy
