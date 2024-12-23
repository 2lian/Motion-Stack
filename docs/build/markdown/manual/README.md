# Motion Stack

[![image](https://img.shields.io/badge/Ubuntu-%2020.04%20%7C%2022.04%20-%20blue)](https://ubuntu.com/)[![image](https://img.shields.io/badge/Ros2-Foxy%20%7C%20Humble-%20blue)](https://github.com/ros2)[![image](https://img.shields.io/badge/Python-3.8_|_3.10-%20blue)](https://www.python.org/)[![image](https://raw.githubusercontent.com/petercorke/robotics-toolbox-python/master/.github/svg/rtb_powered.min.svg)](https://github.com/petercorke/robotics-toolbox-python)[![image](https://github.com/2lian/Moonbot-Motion-Stack/actions/workflows/stepbystep.yaml/badge.svg)](https://github.com/2lian/Moonbot-Motion-Stack/actions/workflows/stepbystep.yaml)[![image](https://img.shields.io/badge/License-MIT-gold)](LICENSE)

Modular walking robots or a single robotic arm, seamlessly bring your robots to life with just a URDF! Built for maximum flexibility, ease of use, and source-code customization.

# Guides:

* [Installation](install.md)
* [Your URDF with This Repo](URDF.md)
* [How to use](use.md)
* [API](api.md)

## Features:

- **Modular**, any limb anywhere
- **Multi-limb synchronization**
- **Custom trajectories**  *(in development)*
- **Inverse Kinematics** (3Dof and above)
- **URDF parser**
- **Customizable actuators interface** (overload the source-code with what you need)
- **Flexible launch system**
- **Documented example of Moonbot Zero**

## Ros2 Structure Overview

The current basic structure can be interpreted as the following tree:

```text
|                       levels
|   00    |     01      |     02   |   03   |    04   |    05   |
| Motor X -- Joint 0 -- |
| Motor X -- Joint 1 -- +- IK 0 -- Leg 0 -- |
| Motor X -- Joint 2 -- |                   |
|                                           |
| Motor X -- Joint 0 -- |                   |
| Motor X -- Joint 1 -- +- IK 1 -- Leg 1 -- +-  Mover  -- Gait
| Motor X -- Joint 2 -- |                   |
|                                           |
|                                   ...  -- |
```

The power of this structure lies in its modularity. Packages responsible for a level can be swapped in/out for other packages responsible for the same level.

For example:
- When using the real robot, [dynamixel_hotplug_ros2_python](https://github.com/hubble14567/dynamixel_hotplug_ros2_python) is used.
- When testing without the robot, [rviz_basic](src/rviz_basic) is used.

```text
|                       levels
|      00       |    01   |   02  |   03  |   04   |  05   |
| ---------------------packages----------------------------
|               |             easy robot control
| ---------------------------------------------------------
|   rviz basic  |
| ---------------------------------------------------------
| dynamixel...  |
| ---------------------------------------------------------
| Maxon motr... |
```

All robots are different. You can easily overload relevant parts of the code and use it like an API in which you inject your custom code. Examples and tools are provided for this purpose. This way, you do not need to create a new, complex ROS2 node to adapt to the quirks of your robot—just change what you need directly.

```text
|                       levels
|      00       |    01   |   02  |   03  |   04   |  05   |
| ---------------------packages----------------------------
|               |             easy robot control
| ---------------------------------------------------------
| Overload for my robot   |                        |  Overload for my robot
| ---------------------------------------------------------
```
