# Installation

This will run you through the installation and setup. Please update it when you find new requirements (notably the python packages)

## Overview

* [Ubuntu 22.04](installation.md#os-ubuntu-2204)
* [ros2-humble](installation.md#ROS2-humble)
  * All default ROS2 messages
  * RViz
  * Xacro
* [Python 3.10](installation.md#Python-libraries)
  * numpy
  * numpy-quaternion
  * robotic-toolbox (custom fork)
  * scipy
  * numba 
  * xacro
* [This repo](installation.md#this-repo)
* [Shortcuts](installation.md#Shortcuts)
  * `${ROS2_INSTALL_PATH}` points to `/opt/ros/humble`
  * `${ROS2_MOONBOT_WS}` points to `path_to_this_repo`

Tested to work on ros2-foxy (Ubuntu 20.04).

## OS: Ubuntu 22.04

It does not work on Windows, do not try.
You can manage through WSL2 (Windows Subsystem for Linux), but if you don't know what you are doing and are new to linux, a virtual machine (= emulation of Linux) will be harder than a linux dual boot. (and it requires a better PC, and more ram, and lots of things are broken)

## ROS2 humble

Follow the installation guide of humble: [https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)

You just need to install xacro globally using:
```bash
sudo apt install ros-humble-xacro
```

## Python 3 libraries

Ensure pip is installed:

````bash
sudo apt update
sudo apt install python3-venv python3-pip
````

First.
Robotic toolbox has a bug that I fixed, until [my PR](https://github.com/petercorke/robotics-toolbox-python/pull/441) is approved, you will have to install my fork (this is done in the code below).
````bash
git clone https://github.com/hubble14567/robotics-toolbox-python
cd robotics-toolbox-python
sudo pip install pip-tools # for dependencies
sudo python3 -m piptools compile -o requirements.txt setup.py # takes a while (2-3 min on embedded pc)
sudo pip install -r requirements.txt # for dependencies
sudo pip3 install -e .
````

KNOWN ISSUE: pip install of my custom rtb fork does not install the dependencies of rtb. You can install then uninstall the official rtb, or fix the dependencies yourself as a workaround. The code above tries to fix that, but it has not been tested a lot.

Run this to install the required Python libraries and compatible version:
(please report any missing libraries)

````bash
sudo python3 -m pip install --upgrade --force-reinstall numpy numba numpy-quaternion scipy xacro
````

## This repo

````bash
git clone https://github.com/2lian/Moonbot-Motion-Stack.git
cd Moonbot-Motion-Stack
````

## Shortcuts (now optional)

This will modify your `.bash.rc`. The bashrc contains a series of command that are executed when you open a new terminal. You only need to execute those command once, or if you want to change the paths.
We will define some variables that points to folders on your system. Like a shortcut. If you have changed the location or installed somewhere else, change those shortcuts.

- `${ROS2_INSTALL_PATH}` should point to your ros2 installation path (`/opt/ros/humble` by default for humble). Use this command to write this shortcut in you bashrc:
````bash
echo 'export ROS2_INSTALL_PATH=/opt/ros/humble' >> ~/.bashrc
````
- `${ROS2_MOONBOT_WS}` should point to where you cloned/downloaded this repo, (replace `path_to_this_repo` with your path, replace with `~/moonbot_software` if you cloned in your home directory)
````bash
echo 'export ROS2_MOONBOT_WS=path_to_this_repo' >> ~/.bashrc
````

