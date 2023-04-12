# LiDAR Ground Plane & Noise Filtering

This project aims to remove ground plane and noise points from a LiDAR point cloud.
The primary objective is to implement a point cloud filtering algorithm using [Python](https://www.python.org/) that can be integrated into the [ROS](https://www.ros.org/) framework.

## Installation

A [Dockerfile](Dockerfile) is provided for streamlined installation of the required dependencies.
To build the Docker image, execute the following command from the root directory of the repository:

```bash
docker build -t lidar-filtering .
```

## Usage

Bash scripts are provided for initializing the Docker container with GUI support and running the filtering algorithm.

To initialize the Docker container, execute the following command from the root directory of the repository:

```bash
./docker_init.sh
```

To run the plane filtering algorithm, execute the following command after the Docker container has been initialized:

```bash
./run.sh
```

## Methodology

### Choice of Language

[Python](https://www.python.org/) was chosen for this project due to its ease of use.

While [C++](https://isocpp.org/) offers more efficiency and extensive support for point cloud processing using [PCL](http://pointclouds.org/), Python is more accessible and better suited for rapid prototyping. Therefore, Python was selected for this project.

### Selected Library of Point Cloud Processing

Initially, [PCL](http://pointclouds.org/) was considered the prime candidate for the point cloud processing library because of its extensive support for LiDAR sensors and ease of integration with ROS. However, it was not used in the development of this project due to its lack of support for Python.

[PCL-Python](https://github.com/strawlab/python-pcl) provides Python bindings for PCL, but it is not actively maintained. Consequently, [Open3D](http://www.open3d.org/) was chosen as the library for point cloud processing in this project.

## Implementation Details

### Point-Cloud Analysis

An initial analysis of the point cloud was conducted to determine an effective approach for filtering the ground plane and noise points.
The details of the analysis can be found in the [analysis report][def].

[def]: DATA_NOTES.md

An initial visual analysis was done using [RViz](http://wiki.ros.org/rviz). It was observed that the ground plane is primarily flat, which suggests that a plane fitting algorithm like [RANSAC](http://pointclouds.org/documentation/tutorials/planar_segmentation.php#ransac) can be used to remove the ground plane points.

Furthermore, the point cloud contains a significant amount of noise points (as can be seen in the image below) that are situated far outside the standard point cloud data.

[![Point Cloud][1]][1]

[1]: assets/raw_pcd_rviz.png

As it is evident from the image above, a lot of points exist in isolation. These points can be considered outliers and can be removed using outlier filtering algorithms like [Statistical Outlier Removal](http://pointclouds.org/documentation/tutorials/statistical_outlier.php) or [Radius Outlier Removal](http://pointclouds.org/documentation/tutorials/radius_outlier.php).

In addition to the noise mentioned above, there are some points that are very close to the origin of the point cloud.

[![Point Cloud][2]][2]

[2]: assets/raw_pcd_rviz_mount_noise.png

These points likely exist because of how the LiDAR sensor has been mounted on the host and do not provide any useful information. Therefore, these points can be removed using a distance threshold without any major loss of information.

***The filtering algorithm was developed on the basis of the above analysis.***

### Outlier Filtering

To remove the noise, outlier filtering was performed using the [Statistical Outlier Removal](http://pointclouds.org/documentation/tutorials/statistical_outlier.php) algorithm.

Statical outlier removal works by calculating the mean distance of each point to its nearest neighbors. If the distance of a point to its nearest neighbors is greater than a user-defined threshold, the point is considered an outlier and is removed.

Following parameters were used for the outlier filtering:

```yaml
statistical_outlier_removal:
  nb_neighbors: 78  # small: too sensitive to noise, large: too conservative
  std_ratio: 3.4  # small: would remove too much, large: would fail to capture outliers
```

Using statistical outlier filtering alone didn't remove all the noise points. Therefore, radius outlier filtering was also used to remove the remaining noise points with following parameters:

```yaml
  radius_outlier_removal:
    radius: 2  # small: too sensitive to noise, large: too conservative
    min_neighbors: 4  # small: would fail to capture outliers, large: would remove too much
```

After applying these filters, the point cloud looks like this ***(the removed noise points are shown in red)***:

[![Point Cloud][3]][3]

[3]: assets/0000_pcd_filtering.png

### Noise Removal

As discussed, apart from the outliers, there are some points that are very close to the origin of the point cloud. These points were removed by using a distance threshold from the origin, i.e. points lying within a certain distance from the origin were removed.

The thresholds used for the distance filtering were:

```yaml
  noise_filtering_params:
    min_range: 3  # smallest lidar pcd range (in m)
    max_range: 240  # largest lidar pcd range (in m)
```

The point cloud after applying the distance threshold looks like this ***(the removed noise points are shown in red)***:

[![Point Cloud][4]][4]

[4]: assets/0001_pcd_noise_removed.png

### Ground Plane Removal

Finally, the ground plane was removed using the [RANSAC](http://pointclouds.org/documentation/tutorials/planar_segmentation.php#ransac) algorithm.

RANSAC works by randomly selecting a subset of points and fitting a model to the subset. The model is then evaluated to determine how well it fits the data. The model with the best fit is selected as the best model. This process is repeated a number of times to find the best model.

We can apply RANSAC algorithm on every time step of the point cloud to find the best model for the ground plane. However, this would be computationally expensive.

To reduce the computational cost, we can use the best model from the previous time step to filter out the points that are close to the ground plane. We can keep using the same model for a number of time steps and then re-estimate the model using RANSAC.
This approach has been implemented in the filtering algorithm.

The RANSAC algorithm would yield the best results if the ground plan is relatively flat.
In the provided point cloud, RANSAC was able to remove the ground plane points with a high degree of accuracy, hence, further filtering was not required.

The parameters used for the RANSAC algorithm:

```yaml
  min_points_to_fit: 3
  distance_threshold: 0.1
  num_iterations: 1000
  ransac_frequency: 3
```

The point cloud after applying the RANSAC algorithm looks like this ***(the removed ground plane points are shown in red)***:

[![Point Cloud][5]][5]

[5]: assets/0002_pcd_ground_filtering_ransac_on_full_pcd.png

The results shown in the image above fed the entire point cloud to RANSAC for estimating the ground plane. However, this approach is computationally expensive.

Therefore, a few experiments were conducted to determine the best initial seed that can be fed to the RANSAC algorithm to reduce the computational cost.

### Random Seed

In this experiment, a random subset of *100 points* was used as the initial seed for the RANSAC algorithm.

### Surface Normals Seed

In this experiment, the surface normals of the point cloud were used to generate the initial seed for the RANSAC algorithm.

Surface normals are the vectors that are perpendicular to the surface of an object. They can be used to determine the orientation of the object.
As we figured out in the initial analysis, the ground plane is relatively flat. Therefore, the surface normals of the ground plane points should be close to the z-axis.
Hence, the points having the surface normals within a certain threshold from the z-axis can be used as the initial seed for the RANSAC algorithm.

Parameters used for the surface normals seed:

```yaml
surface_normal_sampling_params:
  radius: 0.1
  max_nn: 50
  selection_threshold: 0.1
```

### Horizontal Angle Seed

In this experiment, horizontal angle of the points was calculated using the following formula:

```python
horizontal_angle = np.arctan2(point_y, point_x)
```

The points having the horizontal angle within a certain threshold from the z-axis can be used as the initial seed for the RANSAC algorithm.

Parameters used for the horizontal angle seed:

```yaml
horizontal_sampling_params:
  min_angle: -0.523599
  max_angle: 0.523599
```

### Experiment Results

To find a balance between the computational cost and the accuracy of the ground plane removal, the results of the above experiments were compared.
Visual inspection of the results showed random seed and surface normals perform similarly. However, the horizontal angle seed performed poorly.

The results of the experiments are shown in the following images ***(the removed ground plane points are shown in red)***:

- ***The (t) means the time step the RANSAC algorithm was applied on***
- ***(t-1) means the RANSAC algorithm was applied on the previous time step to get the ground plane and the same plane model has been used for the current time step***

[![Point Cloud][6]][6]

[6]: assets/stitched/plane_fitted_(t).png

[![Point Cloud][7]][7]

[7]: assets/stitched/used_estimated_plane_(t-1).png

[![Point Cloud][8]][8]

[8]: assets/stitched/used_estimated_plane_(t-2).png

[![Point Cloud][9]][9]

[9]: assets/stitched/used_estimated_plane_(t-3).png

[![Point Cloud][10]][10]

[10]: assets/stitched/used_estimated_plane_(t-4).png

- As the results show both the random seed and surface normals seed perform similarly, the ***random seed was used for the final implementation*** as it is computationally less expensive.
- Moreover, ***a predicted ground plane model was used for the next 3 time steps to reduce the computational cost*** (empirically determined).
