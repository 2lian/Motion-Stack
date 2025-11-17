# Motion Stack

![Synchronization of 6 robots](media/landing.gif)

[![ubuntu](https://img.shields.io/badge/Ubuntu-%2020.04%20%7C%2022.04%20%7C%2024.04-%20blue)](https://ubuntu.com/)
[![ros](https://img.shields.io/badge/Ros2-Foxy%20%7C%20Humble%20%7C%20Jazzy-%20blue)](https://github.com/ros2)
[![python](https://img.shields.io/badge/Python-3.8_|_3.10_|_3.12-%20blue)](https://www.python.org/)
[![mit](https://img.shields.io/badge/License-MIT-gold)](https://opensource.org/license/mit)

[![rtb](https://img.shields.io/badge/Powered_by-Robotics_toolbox-006400)](https://github.com/petercorke/robotics-toolbox-python)
[![pixi](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/prefix-dev/pixi/main/assets/badge/v0.json)](https://pixi.sh)
[![test](https://github.com/2lian/Motion-Stack/actions/workflows/doit_install.yaml/badge.svg)](https://github.com/2lian/Motion-Stack/actions/workflows/doit_install.yaml)

From modular robots with distributed computation, to a simple robotic arm, the motion stack provides control for (multi-)limbed systems. The goal of the project is maximum flexibility reflecting the flexibility of modular robotics, while abstracting away the complexity of such systems.

# Table of Contents:

* [Installation using Pixi](manual/install.md)
* [Installation from source](manual/install.md#installation-from-source)
* [Quick start](manual/start.md)
* [Operator TUI](manual/operator_tui.md)
* [API](manual/api.md)
* [ROS2 nodes and interfaces](manual/use.md)

# Credits:

* [Cite Us!](manual/papers.md)
* [Contributors](manual/credits.md)

<h2>Features</h2>

---
- **Modular** – any limb anywhere on the robot.
- **Distributed** – any process anywhere on the network (ROS2 interface).
- **Runtime hardware agnosticism** – adapts in real-time to robot characteristics.
- **Separation of concerns** – team-member implementations and robot specificities, minimally impacts the other systems.
- **Inverse Kinematics** – 3Dof and above.
- **Multi-limb synchronization**
- **TUI included** – control your robot through a Terminal User Interface.
- **Customizable interfaces** – Use the API and override the source-code for your robot and team.
- **URDF parser**
- **Flexible launch system**
- **Documented example of Moonbot Zero**

![image](media/landingx3.gif)<h2>Upcomming Features</h2>

---
- Advanced launcher with new robot and advanced modularity.
- Zenoh interface to replace ROS2.

<h2>Ros2 Structure Overview</h2>

---

The current basic structure can be interpreted as the following tree:

```text
|                       levels
|   00    |     01      |    02    |  High Lvl API   |
| Motor X -- Joint 0 -- |
| Motor X -- Joint 1 -- +- IK 0 -- |   +- Python API
| Motor X -- Joint 2 -- |          |   |
|                                  |   +- Python API
| Motor X -- Joint 0 -- |          |   |
| Motor X -- Joint 1 -- +- IK 1 -- + - +- Python API -- TUI
| Motor X -- Joint 2 -- |          |   |
|                                  |   +- Python API
```

The power of this structure lies in its modularity. Levels can be modified,
swapped and assembled while remaining compatible with other levels. This can
happen anywhere on the network, lvl02 doesn’t need to run on every robot, so
one can assemble several lvl1 and control them with one lvl2 IK.

Several higher levels, can also – more or less – control the same lower
level. This is very useful when several APIs control the same robot, so several
people can take control and do their experiment seamlessly.

# Code:

* [motion_stack.api package](api/motion_stack/motion_stack.api.md)
  * [Subpackages](api/motion_stack/motion_stack.api.md#subpackages)
  * [Submodules](api/motion_stack/motion_stack.api.md#submodules)
  * [motion_stack.api.ik_syncer module](api/motion_stack/motion_stack.api.md#module-motion_stack.api.ik_syncer)
  * [motion_stack.api.joint_syncer module](api/motion_stack/motion_stack.api.md#module-motion_stack.api.joint_syncer)
* [motion_stack.core package](api/motion_stack/motion_stack.core.md)
  * [Subpackages](api/motion_stack/motion_stack.core.md#subpackages)
  * [Submodules](api/motion_stack/motion_stack.core.md#submodules)
  * [motion_stack.core.lvl1_joint module](api/motion_stack/motion_stack.core.md#module-motion_stack.core.lvl1_joint)
  * [motion_stack.core.lvl2_ik module](api/motion_stack/motion_stack.core.md#module-motion_stack.core.lvl2_ik)
  * [motion_stack.core.lvl4_mover module](api/motion_stack/motion_stack.core.md#module-motion_stack.core.lvl4_mover)
* [motion_stack.ros2 package](api/motion_stack/motion_stack.ros2.md)
  * [Subpackages](api/motion_stack/motion_stack.ros2.md#subpackages)
  * [Submodules](api/motion_stack/motion_stack.ros2.md#submodules)
  * [motion_stack.ros2.communication module](api/motion_stack/motion_stack.ros2.md#module-motion_stack.ros2.communication)

# Operator TUI:

* [ms_operator package](api/ms_operator/ms_operator.md)
  * [Submodules](api/ms_operator/ms_operator.md#submodules)
  * [ms_operator.operator_node module](api/ms_operator/ms_operator.md#module-ms_operator.operator_node)
  * [ms_operator.operator_tui module](api/ms_operator/ms_operator.md#module-ms_operator.operator_tui)
  * [ms_operator.operator_utils module](api/ms_operator/ms_operator.md#module-ms_operator.operator_utils)

# Deprecated Code:

* [easy_robot_control package](api/easy_robot_control/easy_robot_control.md)
  * [Subpackages](api/easy_robot_control/easy_robot_control.md#subpackages)
  * [Submodules](api/easy_robot_control/easy_robot_control.md#submodules)
  * [easy_robot_control.EliaNode module](api/easy_robot_control/easy_robot_control.md#module-easy_robot_control.EliaNode)
  * [easy_robot_control.gait_key_dev module](api/easy_robot_control/easy_robot_control.md#module-easy_robot_control.gait_key_dev)
  * [easy_robot_control.gait_node module](api/easy_robot_control/easy_robot_control.md#module-easy_robot_control.gait_node)
  * [easy_robot_control.ik_heavy_node module](api/easy_robot_control/easy_robot_control.md#module-easy_robot_control.ik_heavy_node)
  * [easy_robot_control.joint_state_interface module](api/easy_robot_control/easy_robot_control.md#module-easy_robot_control.joint_state_interface)
  * [easy_robot_control.lazy_joint_state_publisher module](api/easy_robot_control/easy_robot_control.md#module-easy_robot_control.lazy_joint_state_publisher)
  * [easy_robot_control.leg_api module](api/easy_robot_control/easy_robot_control.md#module-easy_robot_control.leg_api)
  * [easy_robot_control.leg_node module](api/easy_robot_control/easy_robot_control.md#module-easy_robot_control.leg_node)
  * [easy_robot_control.mover_node module](api/easy_robot_control/easy_robot_control.md#module-easy_robot_control.mover_node)
