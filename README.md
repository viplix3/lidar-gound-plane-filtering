# LiDAR Ground Plane & Noise Filtering

This project aims to remove ground plane and noise points from a LiDAR point cloud.
The primary objective is to implement a point cloud filtering algorithm using [Python](https://www.python.org/) that can be integrated into the [ROS](https://www.ros.org/) framework.

[![Results][0]][0]

[0]: assets/results.gif

## Table of Contents

- [LiDAR Ground Plane & Noise Filtering](#lidar-ground-plane--noise-filtering)
  - [Installation](#installation)
  - [Usage](#usage)
  - [Methodology](#methodology)
    - [Choice of Language](#choice-of-language)
    - [Selected Library of Point Cloud Processing](#selected-library-of-point-cloud-processing)
  - [Implementation Details](#implementation-details)
    - [Point-Cloud Analysis](#point-cloud-analysis)
    - [Outlier Filtering](#outlier-filtering)
    - [Noise Removal](#noise-removal)
    - [Ground Plane Removal](#ground-plane-removal)
  - [Experiment & Results](#experiments--results)
  - [Issues & Limitations](#issues--limitations)
  - [Conclusions](#conclusion)

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

Initially, [PCL](http://pointclouds.org/) was considered as the prime candidate for the point cloud processing library because of its extensive support for LiDAR sensors and ease of integration with ROS. However, it was not used in the development of this project due to its lack of support for Python.

[PCL-Python](https://github.com/strawlab/python-pcl) provides Python bindings for PCL, but it is not actively maintained. Consequently, [Open3D](http://www.open3d.org/) was chosen as the library for point cloud processing in this project.

## Implementation Details

### Point-Cloud Analysis

An initial analysis of the point cloud was conducted to determine an effective approach for filtering the ground plane and noise points.
The details of the analysis can be found in the [analysis report][def].

[def]: DATA_NOTES.md

Visual analysis was done using [RViz](http://wiki.ros.org/rviz).

- It was observed that the ground plane is primarily flat, which suggested that a plane fitting algorithm like [RANSAC](http://pointclouds.org/documentation/tutorials/planar_segmentation.php#ransac) can be used to remove the ground plane points.
- Furthermore, the point cloud contained a significant amount of noise points (as can be seen in the image below) that are situated far outside the standard point cloud data.

[![Point Cloud][1]][1]

[1]: assets/raw_pcd_rviz.png

As it is evident from the image above, a lot of points exist in isolation. These points can be considered outliers and can be removed using outlier filtering algorithms like [Statistical Outlier Removal](http://pointclouds.org/documentation/tutorials/statistical_outlier.php) or [Radius Outlier Removal](http://pointclouds.org/documentation/tutorials/radius_outlier.php).

- In addition to the noise mentioned above, there are some points that are very close to the origin of the point cloud.

[![Point Cloud][2]][2]

[2]: assets/raw_pcd_rviz_mount_noise.png

These points most likely exist because of how the LiDAR sensor has been mounted on the host and do not provide any useful information. Therefore, these points can be removed using a distance threshold without any major loss of information.

***The noise filtering algorithm was developed on the basis of the above analysis.***

### Outlier Filtering

To remove the noise, outlier filtering was performed using the [Statistical Outlier Removal](http://pointclouds.org/documentation/tutorials/statistical_outlier.php) algorithm.

Statical outlier removal works by calculating the mean distance of each point to its nearest neighbors. If the distance of a point to its nearest neighbors is greater than a user-defined threshold, the point is considered an outlier and is removed.

Following parameters were used for the outlier filtering:

```yaml
statistical_outlier_removal:
  nb_neighbors: 78  # small: too sensitive to noise, large: too conservative
  std_ratio: 3.4  # small: would remove too much, large: would fail to capture outliers
```

Tuning the statistical outlier removal parameters was a bit tricky as choosing a smaller value for `nb_neighbors` resulted in too many points being removed, while choosing a larger value resulted in too many noise points being retained.
Therefore, radius outlier filtering was used on top of statistical outlier filter to remove the noise points with following parameters:

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

- To reduce the computational cost, we can use the best model from the previous time step to filter out the points that are close to the ground plane.
- We can keep using the predicted plane model for N-number of time steps, updating the model periodically to account for any changes in the ground plane.

This approach has been implemented in the filtering algorithm.
In the provided point cloud, RANSAC was able to remove the ground plane points with a high degree of accuracy, hence, further ground plane filtering was not done.

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
Therefore, a few experiments were conducted to determine the best initial seed that can be fed to the RANSAC algorithm to reduce the computational cost. Details of these experiments are discussed below.

### Random Seed

In this experiment, a random subset of *200 points* was used as the initial seed for the RANSAC algorithm.

### Surface Normals Seed

In this experiment, the surface normals of the points were used to generate the initial seed for the RANSAC algorithm.

Surface normals are the vectors that are perpendicular to the surface of an object. They can be used to determine the orientation of the object.
As it was determined in the initial analysis that the ground plane is relatively flat, the surface normals of the plane points close to the z-axis can be used as the initial seed for the RANSAC algorithm.

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

## Experiments & Results

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

### Testing on KITTI Dataset

The proposed pipeline was also tested on the KITTI dataset out of curiosity to see how it performs on a different point cloud distribution.

```yaml
Note: The frames shown are not the exact same for random seed, sufface normals seed and horizontal angle seed as it was difficult to sync and find the exact same frames for all the seeds. However, the results shown are representative of the overall performance of the seeds.
```

The images below show the results on raw KITTI dataset sequence `kitti_2011_09_26_drive_0002_synced` ***(the estimated ground plane points are shown in green)***:

[![Point Cloud][11]][11]

[11]: assets/results_kitti/ground_plane_estimation_random_sampling_ransac.png

[![Point Cloud][12]][12]

[12]: assets/results_kitti/ground_plane_estimation_surface_normals_sampling_ransac.png

[![Point Cloud][13]][13]

[13]: assets/results_kitti/ground_plane_estimation_horizontal_angle_sampling_ransac.png

***Surprisingly, the surface normals seed performed the worst on KITTI dataset. More exploration is needed to identify any issues that may have caused this unexpected result. Possible reasons for the poor performance of the surface normals seed could be related to differences in point cloud characteristics between the datasets, such as point density or the ground plane characteristics.***

## Issues & Limitations

**Processing Time**: The proposed pipeline is computationally expensive.The processing time for Statistical Outlier Removal and Radius Outlier Removal algorithms is significant (~0.5 and ~0.8 seconds per time step respectively). This would hinder real-time applications of the ground plane removal pipeline. A possible solution is implementing voxel grid downsampling to reduce the point cloud size and processing time without a significant loss of accuracy. However, this has not been implemented in the project so far.

**Dataset Dependency**: The pipeline's performance seems to be sensitive to the specific dataset used. While the pipeline performed reasonably well on the initial dataset, the results were less satisfactory when tested on the KITTI dataset. This highlights the need for further exploration and fine-tuning to ensure robustness across various datasets and point cloud characteristics.

**Empirical Parameter Selection**: A lot of parameters, such as the choice of using a predicted ground plane model for the next three time steps, were selected empirically. This may not be the optimal solution for all scenarios or datasets, which could lead to suboptimal results. A more systematic approach to parameter tuning and selection may be necessary for broader applicability.

**Initial Seed Selection**: The experiments indicated that different seed selection methods resulted in varying performance. While the random seed was chosen for the final implementation due to its computational efficiency, other seeding strategies, like surface normals or horizontal angles, may yield better results in certain cases. Further research could be conducted to identify more robust and adaptive seed selection methods.

**Point Cloud Quality**: The pipeline's performance could be influenced by the quality of the input point cloud data, such as the presence of noise or varying point densities. The pipeline may need additional pre-processing steps or improvements to handle such variations in input data effectively.

## Conclusion

In conclusion, this report presents a comprehensive ground plane removal pipeline that has been successfully tested on a point cloud dataset obtained from a mobile robot. The pipeline employs statistical outlier removal (SOR) and radius outlier removal (ROR) to clean the point cloud data, and a RANSAC-based method for detecting and removing the ground plane.

The pipeline's performance was evaluated using different seed selection methods for the RANSAC algorithm, including random seed, surface normals seed, and horizontal angle seed. The random seed was selected for the final implementation due to its computational efficiency, and a predicted ground plane model was used for the next three time steps to further reduce processing time.

However, there are several limitations and issues identified with the pipeline. These include the processing time, dataset dependency, empirical parameter selection, initial seed selection, the trade-off between accuracy and computational cost, and point cloud quality. Further research and optimization efforts are needed to address these limitations and enhance the pipeline's performance across various datasets and scenarios.
