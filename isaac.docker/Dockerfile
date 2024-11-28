# Base image
FROM nvcr.io/nvidia/isaac-sim:4.2.0

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

USER $USERNAME

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

# Update package lists and install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=humble
ENV ROS2_WS=/moonbot

# Install pip packages 
# Update package lists and install ROS 2 Humble
RUN apt-get update && apt-get install -y \
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
    git \
    && rm -rf /var/lib/apt/lists/*
RUN pip install numpy numpy-quaternion pip-tools scipy numba xacro

# Install the forked robotics-toolbox-python repository
RUN git clone https://github.com/hubble14567/robotics-toolbox-python /robotics-toolbox-python
WORKDIR /robotics-toolbox-python
RUN python3 -m piptools compile -o requirements.txt setup.py
RUN pip install -r requirements.txt
RUN pip install -e .

# Install necessary packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libsdl1.2-dev \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./isaac.docker/entrypoint.sh /



WORKDIR ${ROS2_WS}

COPY . .

ENTRYPOINT ["/entrypoint.sh"]

# Set the entrypoint to bash for convenience
CMD ["bash"]
