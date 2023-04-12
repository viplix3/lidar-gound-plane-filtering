# LiDAR Ground Plane & Noise Filtering

This project aims to remove ground plane and noise points from a LiDAR point cloud.
The main objective is to implement a filtering algorithm using [Python](https://www.python.org/) that can be integrated into [ROS](https://www.ros.org/) framework.

The project uses [Open3D](http://www.open3d.org/) for point cloud processing.

## Installation

A [Dockerfile](Dockerfile) is provided for easy installation of the required dependencies.
To build the Docker image, run the following command from the root directory of the repository:

```bash
docker build -t lidar-filtering .
```

## Usage

Bash scripts are provided for initializing the Docker container with GUI support and running the ground plane filtering algorithm.

To initialize the Docker container, run the following command from the root directory of the repository:

```bash
./docker_init.sh
```

To run the ground plane filtering algorithm, run the following command once the Docker container has been initialized:

```bash
./run.sh
```

## Methodology

### Choice of Language

[Python](https://www.python.org/) was chosen as the language for this project because of its ease of use.

Although [C++](https://isocpp.org/) is more efficient and has extensive support for point-cloud processing ([PCL](http://pointclouds.org/)), Python is more accessible and is better suited for rapid prototyping, hence it was chosen for this project.

### Selected Library of Point Cloud Processing

[PCL](http://pointclouds.org/) was the prime candidate for point cloud processing library because of its extensive support for LiDAR sensors and ease of integration with ROS, but it was not used in development of this project because of its lack of support for Python.

[PCL-Python](https://github.com/strawlab/python-pcl) provides Python bindings for PCL, but it is not actively maintained, hence [Open3D](http://www.open3d.org/) was chosen as the library for point cloud processing in this project.

## Implementation Details

### Point-Cloud Analysis

An initial analysis of the point cloud was performed to determine the best approach for filtering the ground plane and noise points.
The details of the analysis can be found in the [analysis report][def].

[def]: DATA_NOTES.md
