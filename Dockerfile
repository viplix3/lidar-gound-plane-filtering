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
    && rm -rf /var/lib/apt/lists/*

# Install python dependencies
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash exec \"$@\"", "bash"]

