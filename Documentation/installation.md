# Installation

This will run you through the installation and setup. Please update it when you find new requierments 
(notabily the python packages)

## Overview

* [Ubuntu 22.04](installation.md#os-ubuntu-2204)
* [ros2-humble](installation.md#ROS2-humble)
  * All default ROS2 messages
  * RViz
* [Python 3.10](installation.md#Python-libraries)
  * numpy
  * quaternion-numpy
  * robotic-toolbox (custom fork)
  * scipy
  * numba 
* [Shortcuts](installation.md#Shortcuts)
  * `${ROS2_INSTALL_PATH}` points to `/opt/ros/humble`
  * `${ROS2_MOONBOT_WS}` points to `path_to_this_repo`

## OS: Ubuntu 22.04

It does not work on Windows, do not try.
You can manage through WSL2 (Windows Subsystem for Linux), but if you don't know what you are doing and are new to linux, a virtual machine (= emulation of Linux) will be harder than a linux dual boot. (and it requires a better PC, and more ram, and lots of things are broken)

## ROS2 humble

Follow the installation guide of humble: [https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)

## Python libraries

Run this to install the requiered Python libraries and compatible version:
(please report any missing libraries)

````bash
sudo apt install pip
python3 -m pip install --upgrade --force-reinstall numpy
python3 -m pip install --upgrade --force-reinstall numba
python3 -m pip install --upgrade --force-reinstall quaternion-numpy
python3 -m pip install --upgrade --force-reinstall scipy

git clone https://github.com/hubble14567/robotics-toolbox-python
cd robotics-toolbox-python
pip3 install -e .
````

Robotic toolbox has a bug that I fixed, until my PR https://github.com/petercorke/robotics-toolbox-python/pull/441 is approved, you will have to install my fork.

## Shortcuts

This will modify your `.bash.rc`. The bashrc contains a series of command that are executed when you open a new terminal.
We will define some variables that points to folders on your system. Like a shortcut. If you have changed the location or installed somewhere else, change those shortcuts.  If you don't want to use the shortcuts replace the variable by the corresponding path when they are used by this repo.

- `${ROS2_INSTALL_PATH}` should point to your ros2 installation path (`/opt/ros/humble` by default for humble). use this command to write this shortcut in you bashrc:
````bash
echo 'export ROS2_INSTALL_PATH=/opt/ros/humble' >> ~/.bashrc
````
- `${ROS2_MOONBOT_WS}` should point to where you cloned/downloaded this repo, (replace `path_to_this_repo` with your path, replace with `~/moonbot_software` if you cloned in your home directory)
````bash
echo 'export ROS2_MOONBOT_WS=path_to_this_repo' >> ~/.bashrc
````

