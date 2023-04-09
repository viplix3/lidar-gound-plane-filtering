FROM ros:noetic-robot-focal

# Set working directory
WORKDIR /workspace

# Install dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3 \
    python3-dev \
    python3-pip \
    ros-noetic-rviz \
    ros-noetic-rosbag \
    ros-noetic-ros-numpy \
    ros-noetic-rosbridge-suite && \
    rm -rf /var/lib/apt/lists/*

# Create a directory for rosbag files
RUN mkdir /data /src

# Set ROS environment variable
ENV ROS_MASTER_URI=http://localhost:11311

# Install python dependencies
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# Source ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash exec \"$@\"", "bash"]

