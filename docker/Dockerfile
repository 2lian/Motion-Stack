# Use the official ROS 2 Humble base image
FROM osrf/ros:humble-desktop

# Set environment variables
ENV ROS_DISTRO humble
ENV ROS2_WS /moonbot

# Install necessary packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-pip \   
    python3-rosgraph \
    ros-${ROS_DISTRO}-rosgraph-msgs \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-transmission-interface \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install pip packages 
RUN pip install numpy numpy-quaternion pip-tools scipy numba xacro

# Install the forked robotics-toolbox-python repository
RUN git clone https://github.com/hubble14567/robotics-toolbox-python /robotics-toolbox-python

WORKDIR /robotics-toolbox-python
RUN python3 -m piptools compile -o requirements.txt setup.py
RUN pip install -r requirements.txt
RUN pip install -e .

WORKDIR $ROS2_WS

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]