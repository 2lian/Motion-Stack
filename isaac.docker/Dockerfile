# Base image
FROM nvcr.io/nvidia/isaac-sim:4.5.0

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Update and install required tools
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Add the ROS 2 GPG key manually
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the ROS 2 repository to the sources list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list

ENV ROS_DISTRO=humble
ENV ROS2_WS=/moonbot

# ROS 2 requires an older version. This might cause issues with things like straming or caching in Isaac Sim
RUN apt-get update && apt-get install -y --allow-downgrades libbrotli1=1.0.9-2build6

# Install pip packages 
# Update package lists and install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    libfontconfig1-dev \
    libfreetype6-dev \
    g++ \
    wget \
    curl \
    lsb-release \
    gnupg \
    libsdl1.2-dev \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-pip \   
    python3-rosgraph \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-rosgraph-msgs \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-transmission-interface \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-topic-tools \
    ros-${ROS_DISTRO}-topic-tools-interfaces \
    ros-${ROS_DISTRO}-realsense2-camera-msgs \
    ros-${ROS_DISTRO}-realsense2-description \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    git \
    libsdl1.2-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip install numpy numpy-quaternion pip-tools scipy numba xacro nptyping pydantic

# Install easy_robot_control package dependencies
WORKDIR ${ROS2_WS}/src/easy_robot_control
COPY ./src/easy_robot_control/setup.py .
RUN pip install pip-tools && \
    python3 -m piptools compile --extra dev -o requirements.txt setup.py && \
    pip install -r requirements.txt --force-reinstall --upgrade && \
    rm -rf *.egg-info/ requirements.txt

# setup entrypoint
COPY ./isaac.docker/entrypoint.sh /

WORKDIR ${ROS2_WS}

COPY . .

ENTRYPOINT ["/entrypoint.sh"]

# Set the entrypoint to bash for convenience
CMD ["bash"]
