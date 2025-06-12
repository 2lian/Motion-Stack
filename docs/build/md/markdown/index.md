# Motion Stack

![lvl4 whole body movement](media/lvl4.gif)

[![ubuntu](https://img.shields.io/badge/Ubuntu-%2020.04%20%7C%2022.04%20%7C%2024.04-%20blue)](https://ubuntu.com/)
[![ros](https://img.shields.io/badge/Ros2-Foxy%20%7C%20Humble%20%7C%20Jazzy-%20blue)](https://github.com/ros2)
[![python](https://img.shields.io/badge/Python-3.8_|_3.10_|_3.12-%20blue)](https://www.python.org/)
[![mit](https://img.shields.io/badge/License-MIT-gold)](https://opensource.org/license/mit)

[![rtb](https://img.shields.io/badge/Powered_by-Robotics_toolbox-006400)](https://github.com/petercorke/robotics-toolbox-python)
[![doit](https://img.shields.io/badge/Automated_by-DOIT-006400)](https://pydoit.org/)
[![test](https://github.com/2lian/Motion-Stack/actions/workflows/doit_install.yaml/badge.svg)](https://github.com/2lian/Motion-Stack/actions/workflows/doit_install.yaml)

From modular robots with distributed computation, to a simple robotic arm, this package provides control for (multi-)limbed systems. The goal of the project is maximum flexibility reflecting the flexibility of modular robotics, while abstracting away the complexity of such systems.

# Guides:

* [Installation](manual/install.md)
* [Quick start](manual/start.md)
* [ROS2 nodes and interfaces](manual/use.md)
* [API](manual/api.md)
* [Operator TUI](manual/operator_tui.md)

<h2>Features</h2>

---
- **Modular** – any limb anywhere on the robot.
- **Distributed** – any process anywhere on the network.
- **Runtime hardware agnosticism** – adapts in real-time to robot characteristics.
- **Separation of concerns** – team-member implementations and robot specificities, minimally impacts the other systems.
- **Inverse Kinematics** – 3Dof and above.
- **Multi-limb synchronization**
- **URDF parser**
- **Customizable interfaces** – Use the API and override the source-code for your robot and team.
- **Flexible launch system**
- **Documented example of Moonbot Zero**

<h2>Upcomming Features</h2>

---
- **Custom trajectories**  *(in development, lvl3 will be deleted and fused with lvl4)*

<h2>Ros2 Structure Overview</h2>

---

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
|               |             motion stack
| ---------------------------------------------------------
|   rviz basic  |
| ---------------------------------------------------------
| dynamixel...  |
| ---------------------------------------------------------
| Maxon motr... |
```

All robots are different. You can easily override relevant parts of the code and use it like an API in which you inject your custom code. Examples and tools are provided for this purpose. This way, you do not need to create a new, complex ROS2 node to adapt to the quirks of your robot—just change what you need directly.

```text
|                       levels
|      00       |    01   |   02  |   03  |   04   |  05   |
| ---------------------packages----------------------------
|               |             motion stack
| ---------------------------------------------------------
| Overrides for my robot  |                        |  Robot-Agnostic API
| ---------------------------------------------------------
```

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
