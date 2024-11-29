# Motion stack

[![Ubuntu- 20.04 | 22.04](https://img.shields.io/badge/Ubuntu-%2020.04%20%7C%2022.04%20-%20blue)](https://ubuntu.com/)
[![Ros2- foxy | humble](https://img.shields.io/badge/Ros2-Foxy%20%7C%20Humble-%20blue)](https://github.com/ros2)
[![Python 3.8+](https://img.shields.io/badge/Python-3.8+-%20blue)](https://www.python.org/)
[![Architecture x86 and arm64](https://img.shields.io/badge/Arch-x86_|_arm64-purple)]()
[![Robotics Toolbox](https://img.shields.io/badge/Inverse_Kinematics-Robotics%20Toolbox-%20green)](https://github.com/petercorke/robotics-toolbox-python)

[![GitHub last commit (branch)](https://img.shields.io/github/last-commit/2lian/Moonbot-Motion-Stack/main)]()
[![MIT](https://img.shields.io/badge/License-MIT-gold)](LICENSE)


Modular walking robots or a single robotic arm, seamlessly bring your robots to life with just a URDF! Built for maximum flexibility, ease of use and source-code customization.

### Features:
- Modular, any limb anywhere
- Multi limb synchronization
- Custom trajectories (in developpment)
- Inverse Kinematics (3Dof and above)
- URDF parser
- Customizable actuators interface (overload the source-code with what you need)
- Flexible launch system
- Documented example of moonbot zero

## Guides

* [Installation](/Documentation/installation.md)
* [How to setup your URDF](/Documentation/URDF_use.md)
* [How to use](/Documentation/use.md)

## Ros2 Structure overview

The current basic structure can be interpreted as this tree:
```
                      levels
  00    |     01      |     02   |   03   |    04   |    05   |


Motor X -- Joint 0 -- |
Motor X -- Joint 1 -- +- IK 0 -- Leg 0 -- |
Motor X -- Joint 2 -- |                   |
                                          |
Motor X -- Joint 0 -- |                   |
Motor X -- Joint 1 -- +- IK 1 -- Leg 1 -- +-  Mover  -- Gait
Motor X -- Joint 2 -- |                   |
                                          |
                                  ...  -- |
```

The power of this structure can be seen below. Packages responsible for a level can be swapped in/out for other packages responsible for the same levels.
When using the real robot [dynamixel_hotplug_ros2_python](https://github.com/hubble14567/dynamixel_hotplug_ros2_python) is used. When trying things without the robot [rviz_basic](src/rviz_basic) is used.

```
                      levels
     00       |    01   |   02  |   03  |   04   |  05   |
---------------------packages----------------------------
              |             easy robot control
---------------------------------------------------------
  rviz basic  |
---------------------------------------------------------
dynamixel...  |
---------------------------------------------------------
Maxon motr... |
```

Levels 01, 02, 03, 04 and 05 are available in:
- [easy_robot_control](src/easy_robot_control): Simple and easy inverse kinematics and movements

Levels 00 are available in:
- [rviz_basic](src/rviz_basic): Displays the robot fixed in Rviz.
- [dynamixel_hotplug_ros2_python](https://github.com/hubble14567/dynamixel_hotplug_ros2_python): Controls dynamixels motors.
- Make it yourself for your system ;)

# Files and Folders

- `src/easy_robot_control` is the main Ros2 Package of this repo, providing motion control.
- `src/urdf_packer` Holds the URDF of several robots.
- `src/custom_messages` contains Ros2 messages used by this repo.
- `src/rviz_basic` is the interface to Rviz.
- `robot_launcher.launch.py` is a launchfile launching other launchfiles with specified namespaces and settings.
It typically launches all levels of the stack, while providing an easy way to change what interface and levels are being launched.
Modify this to launch only the Rviz interface, or lvl 03, or lvl 02, or only leg#2 ... depending on environment variables and more
- `launch_stack.bash` Sources, builds, everything then launches `launch_stack_rviz.launch.py`.
- `launch_only_rviz.bash` Sources and launches the Rviz gui.
- `launch_rqt.bash` Sources and launches the RQT gui.


Note perso:
https://easings.net/
# My stuff that you should not look at:

- `src/pcl_reader` Displays a pointcloud from a numpy array, usefull to display a map in Rviz.
- `src/ros2_numpy-foxy-devel`
- `multi_auto_place.bash` Places several robots at the desired position, and the IK automatically choses the best foothold from the map.
- `robot_place.bash` Manully places multiple robots.
