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

As it is evident from the image above, a lot of points exist in isolation. These points can be consiidered outliers and can be removed using outlier filtering algorithms like [Statistical Outlier Removal](http://pointclouds.org/documentation/tutorials/statistical_outlier.php) or [Radius Outlier Removal](http://pointclouds.org/documentation/tutorials/radius_outlier.php).

In addition to the noise mentioned above, there are some points that are very close to the origin of the point cloud.

[![Point Cloud][2]][2]

[2]: assets/raw_pcd_rviz_mount_noise.png

These points likely exist because of how the LiDAR sensor has been mounted on the host and do not provide any useful information. These points can be removed using a distance threshold.

***The filtering algorithm was developed on the basis of the above analysis.***
