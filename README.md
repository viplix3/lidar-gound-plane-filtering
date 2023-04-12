# LiDAR Ground Plane & Noise Filtering

This project aims to remove ground plane and noise points from a LiDAR point cloud.
The main objective is to implement a filtering algorithm using [Python](https://www.python.org/) that can be integrated into a [ROS](https://www.ros.org/) framework.

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

Although [C++](https://isocpp.org/) is a more efficient language, and has more extensive support ([PCL](http://pointclouds.org/)), Python is more accessible better suited for rapid prototyping, hence it was chosen for this project.

### Choice of Framework

[ROS](https://www.ros.org/) was chosen as the framework for this project because of its extensive support for LiDAR sensors.

### Selected Library of Point Cloud Processing

[PCL](http://pointclouds.org/) was the prime candidate for point cloud processing because of its extensive support for LiDAR sensors and ease of integration with ROS, but it was not used in development of this project because of its lack of support for Python 3.

[PCL-Python](https://github.com/strawlab/python-pcl) provides Python bindings for PCL, but it is not actively maintained, hence [Open3D](http://www.open3d.org/) was chosen as the library for point cloud processing in this project.

## Implementation Details and Results
