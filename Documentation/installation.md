# Installation

This will run you through the installation and setup. Please update it when you find new requierments 
(notabily the python packages)

## Overview

* Ubuntu 20.04
* ros2-foxy
  * All default ROS2 messages
  * RViz
* Python
  * numpy

## OS: Ubuntu 20.04

It does not work on Windows, do not try.
You can manage through WSL (Windows Subsystem for Linux), but if you don't know what you are doing and are new to linux
a virtual machine (= emulation of Linux) will be harder than a linux dual boot. 
(and it requires a better PC, and more ram, and lots of things are broken)

## ROS2 foxy

Follow the installation guide of foxy: [https://docs.ros.org/en/foxy/Installation.html](https://docs.ros.org/en/foxy/Installation.html)

Those commands in this order should work right away on a new Ubuntu install:
````bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
sudo apt install ros-foxy-desktop

sudo apt install python3-colcon-common-extensions

source /opt/ros/foxy/setup.bash
````

## Python libraries

Run this to install the requiered libraries and compatible version:

````bash
sudo apt install pip
python3 -m pip install --upgrade --force-reinstall numpy
````

## Shortcuts

This will modify your `.bash.rc`. The bashrc contains a series of command that are executed when you open a new terminal.
We will define some variables that points to folders on your system. Like a shortcut. If you have changed the location 
or installed somewhere else, change those shortcuts.  If you don't want to use the shortcuts 
replace the variable by the corresponding path when they are used by this repo.

- `${ROS2_INSTALL_PATH}` should point to your ros2 installation path (`/opt/ros/foxy` by default). use this command to write this shortcut in you bashrc:
````bash
echo 'export ROS2_INSTALL_PATH=/opt/ros/foxy' >> ~/.bashrc
````
- `${ROS2_MOONBOT_WS}` should point to where you cloned/downloaded this repo, (replace `path_to_this_repo` with the path,
for example `~/moonbot_software` if you cloned in your home directory)
````bash
echo 'export ROS2_MOONBOT_WS=path_to_this_repo' >> ~/.bashrc
````

