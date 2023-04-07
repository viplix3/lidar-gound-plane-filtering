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

# Copy the source files into the container
COPY src /workspace/src
COPY configs /workspace/configs

# Source /opt/ros/melodic/setup.bash whenever docker run or docker exec is executed
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# Set the default command for the container
CMD ["bash", "-c", "source /opt/ros/melodic/setup.bash && exec \"$@\"", "bash"]
