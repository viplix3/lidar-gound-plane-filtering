# LiDAR Point-Cloud Ground Plane & Noise Filtering

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

## Experiments and Results
