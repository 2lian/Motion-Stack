# Installation

This will run you through the installation and setup. Please update it when you find new requirements (notably the python packages)

## ROS2 humble (Ubuntu 22.04)

Follow the installation guide of humble: [https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)

(also tested to be compatible with foxy)

## Download the workspace

Clone this repo in the Moonbot-Motion-Stack workspace (or anywhere else you like):

```bash
cd
git clone https://github.com/2lian/Moonbot-Motion-Stack.git
cd Moonbot-Motion-Stack
```

## Use rosdep to install everything ros

```bash
# source ros here
cd ~/Moonbot-Motion-Stack
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r
```

If using foxy you will need to run manually: `sudo apt install ros-foxy-xacro ros-foxy-joint-state-publisher`

## Use pip to install everything Python

```bash
cd ~/Moonbot-Motion-Stack/src/easy_robot_control
sudo apt install python3-pip
pip install pip-tools # for dependencies
python3 -m piptools compile -o requirements.txt setup.py
pip install -r requirements.txt --force-reinstall --upgrade
rm -rf *.egg-info/
```

## (Testing)

Those installation steps are tested regularly, from a fresh Ubuntu install, using GitHub workflow. [See the installation test routine, for more details]()

[![Install foxy | humble](https://github.com/2lian/Moonbot-Motion-Stack/actions/workflows/stepbystep.yaml/badge.svg)](https://github.com/2lian/Moonbot-Motion-Stack/actions/workflows/stepbystep.yaml)

## OLD

You just need to install xacro globally using (change `humble` to `foxy` or else depending on your version), and a lib for the keyboard node.

```bash
sudo apt install ros-humble-xacro
sudo apt install libsdl1.2-dev
```

You might need colcon if it is not installed by default with ros2 (not installed with foxy):

```bash
sudo apt install python3-colcon-common-extensions
```

### Python 3 libraries

Ensure pip and git are installed:

```bash
sudo apt update
sudo apt install python3-venv python3-pip
sudo apt install git
```

First.
Robotic toolbox has a bug that I fixed, until [my PR](https://github.com/petercorke/robotics-toolbox-python/pull/441) is approved, you will have to install my fork (this is done in the code below).

```bash
git clone https://github.com/hubble14567/robotics-toolbox-python
cd robotics-toolbox-python
sudo pip install pip-tools # for dependencies
sudo python3 -m piptools compile -o requirements.txt setup.py # takes a while (2-3 min on embedded pc)
sudo pip install -r requirements.txt # for dependencies
# errors usually happen during the 3 steps above, but it works in the end
sudo pip3 install -e .
```

KNOWN ISSUE: `pip install -e` of my custom rtb fork does not install the dependencies of rtb. You can install then uninstall the official rtb then `pip install -e`, or `pip install -e` then fix the dependencies yourself as a workaround. The code above tries to fix the dependencies automatically, but it has not been tested a lot and has a lot of errors.

Run this to install the required Python libraries and compatible version:
(please report any missing libraries)

```bash
sudo python3 -m pip install --upgrade --force-reinstall numpy numpy-quaternion scipy xacro pytest
```

I had problems with pytest on foxy, I had to uninstall then `pip install pytest==5.4.3`

### This repo

Use this read-only token as username and password to download the repo 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

```bash
cd
git clone https://github.com/2lian/Moonbot-Motion-Stack.git
cd Moonbot-Motion-Stack
```

### Shortcuts (now optional)

This will modify your `.bash.rc`. The bashrc contains a series of command that are executed when you open a new terminal. You only need to execute those command once, or if you want to change the paths.
We will define some variables that points to folders on your system. Like a shortcut. If you have changed the location or installed somewhere else, change those shortcuts.

- `${ROS2_INSTALL_PATH}` should point to your ros2 installation path (`/opt/ros/humble` by default for humble). Use this command to write this shortcut in you bashrc:

```bash
echo 'export ROS2_INSTALL_PATH=/opt/ros/humble' >> ~/.bashrc
```

- `${ROS2_MOONBOT_WS}` should point to where you cloned/downloaded this repo, (replace `path_to_this_repo` with your path, replace with `~/moonbot_software` if you cloned in your home directory)

```bash
echo 'export ROS2_MOONBOT_WS=path_to_this_repo' >> ~/.bashrc
```
