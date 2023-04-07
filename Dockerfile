FROM ros:melodic

# Set working directory
WORKDIR /workspace

# Install necessary ROS and Python packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-melodic-rviz \
    ros-melodic-rosbag \
    ros-melodic-rosbridge-suite && \
    rm -rf /var/lib/apt/lists/*

# Create a directory for rosbag files
RUN mkdir /data

# Set ROS environment variable
ENV ROS_MASTER_URI=http://localhost:11311

# Copy the source code into the container
COPY src /workspace/src

# Set the default command for the container
CMD ["bash", "-c", "source /opt/ros/melodic/setup.bash && exec \"$@\"", "bash"]
