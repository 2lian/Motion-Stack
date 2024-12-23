<!-- Motion-Stack documentation master file, created by
sphinx-quickstart on Fri Dec 20 23:08:51 2024.
You can adapt this file completely to your liking, but it should at least
contain the root `toctree` directive. -->

 Contents:

* [Installation](installation-link.md)
* [How to use](use-link.md)
* [How to use your URDF with this repo](URDF_use-link.md)
* [Reprogram the Stack: Make It Yours.](API-link.md)
* [easy_robot_control package](easy_robot_control.md)
* [easy_robot_control.injection package](easy_robot_control.injection.md)

# Motion stack

[![Ubuntu- 20.04 | 22.04](https://img.shields.io/badge/Ubuntu-%2020.04%20%7C%2022.04%20-%20blue)](https://ubuntu.com/)
[![Ros2- foxy | humble](https://img.shields.io/badge/Ros2-Foxy%20%7C%20Humble-%20blue)](https://github.com/ros2)
[![Python 3.8 | 3.10](https://img.shields.io/badge/Python-3.8_%7C_3.10-%20blue)](https://www.python.org/)
[![Powered by the Robotics Toolbox](https://raw.githubusercontent.com/petercorke/robotics-toolbox-python/master/.github/svg/rtb_powered.min.svg)](https://github.com/petercorke/robotics-toolbox-python)

[![Install foxy | humble](https://github.com/2lian/Moonbot-Motion-Stack/actions/workflows/stepbystep.yaml/badge.svg)](https://github.com/2lian/Moonbot-Motion-Stack/actions/workflows/stepbystep.yaml)
[![MIT](https://img.shields.io/badge/License-MIT-gold)]()

Modular walking robots or a single robotic arm, seamlessly bring your robots to life with just a URDF! Built for maximum flexibility, ease of use and source-code customization.

## Features:

- Modular, any limb anywhere
- Multi limb synchronization
- Custom trajectories (in developpment)
- Inverse Kinematics (3Dof and above)
- URDF parser
- Customizable actuators interface (overload the source-code with what you need)
- Flexible launch system
- Documented example of moonbot zero

## Guides

* [Installation]()
* [How to setup your URDF]()
* [How to use]()
* [Reprogram the Stack: Make It Yours.]()

## Run Docs on Local Server

```bash
python -m http.server 8000 -d docs/build/html
```

## Ros2 Structure overview

The current basic structure can be interpreted as this tree:

```default
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

The power of this structure is illustrated below. Packages responsible for a level can be swapped in/out for other packages responsible for the same levels.
For example when using the real robot [dynamixel_hotplug_ros2_python](https://github.com/hubble14567/dynamixel_hotplug_ros2_python) is used. When trying things without the robot [rviz_basic]() is used.

```default
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

All robots are different, you can easily overload relevant part of the code and use it like an API in which you inject your code, examples and tools are given to do so. This way you do not need to create a new, complicated, Ros2 node to adapt to quirks of your robot, just change what you need directly.

```default
                      levels
     00       |    01   |   02  |   03  |   04   |  05   |
---------------------packages----------------------------
              |             easy robot control
---------------------------------------------------------
Overload for my robot   |                        |  Overload for my robot
---------------------------------------------------------
```
