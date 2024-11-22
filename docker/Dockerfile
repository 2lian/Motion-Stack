# Use the official ROS 2 Humble base image
FROM nvcr.io/nvidia/isaac-sim:4.2.0

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata apt-utils && \
    rm -rf /var/lib/apt/lists/*

# Set environment variables
ENV ROS_DISTRO humble
ENV ROS2_WS /moonbot
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8



# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    curl \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null


# Install necessary packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
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

# Install pip packages 
RUN pip install numpy numpy-quaternion pip-tools scipy numba xacro

# Install the forked robotics-toolbox-python repository
RUN git clone https://github.com/hubble14567/robotics-toolbox-python /robotics-toolbox-python

# Install GZ Sim
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" |  tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    gz-harmonic \
    ros-humble-ros-ign-gazebo \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /robotics-toolbox-python
RUN python3 -m piptools compile -o requirements.txt setup.py
RUN pip install -r requirements.txt
RUN pip install -e .

# isaac ros2_bridge config
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/isaac-sim/exts/omni.isaac.ros2_bridge/humble/lib
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

WORKDIR $ROS2_WS

# setup entrypoint
COPY ./docker/entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]