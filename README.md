<!-- This file is auto-generated from the docs. refere to ./docs/source/manual/README.rst -->
# Motion Stack

![Synchronization of 6 robots](media/landing.gif)

[![ubuntu](https://img.shields.io/badge/OS-Linux_amd64%20%7C%20Linux_aarch64-%20blue)](https://ubuntu.com/)
[![ros](https://img.shields.io/badge/Ros2-Humble%20%7C%20Jazzy%20%7C%20Kilted-%20blue)](https://github.com/ros2)
[![python](https://img.shields.io/badge/Python-3.10_|_3.12-%20blue)](https://www.python.org/)
[![mit](https://img.shields.io/badge/License-MIT-gold)](https://opensource.org/license/mit)

[![pixi](https://img.shields.io/endpoint?url=https%3A%2F%2Fraw.githubusercontent.com%2Fprefix-dev%2Fpixi%2Fmain%2Fassets%2Fbadge%2Fv0.json&color=006400)](https://pixi.sh)
[![rtb](https://img.shields.io/badge/Powered_by-Robotics_toolbox-006400)](https://github.com/petercorke/robotics-toolbox-python)
[![test](https://github.com/2lian/Motion-Stack/actions/workflows/pixi_tests.yaml/badge.svg)](https://github.com/2lian/Motion-Stack/actions/workflows/pixi_tests.yaml)

From modular robots with distributed computation, to a simple robotic arm, the motion stack provides control for (multi-)limbed systems. The goal of the project is maximum flexibility reflecting the flexibility of modular robotics, while abstracting away the complexity of such systems.

# Table of Contents:
 
Access the complete documentation at: [https://motion-stack.deditoolbox.fr/](https://motion-stack.deditoolbox.fr/).

* [Installation using Pixi](docs/build/md/markdown/manual/install.md)
* [Installation from source](docs/build/md/markdown/manual/install.md#installation-from-source)
* [Quick start](docs/build/md/markdown/manual/start.md)
* [Operator TUI](docs/build/md/markdown/manual/operator_tui.md)
* [API](docs/build/md/markdown/manual/api.md)
* [ROS2 nodes and interfaces](docs/build/md/markdown/manual/use.md)

# Credits:

* [Cite Us!](docs/build/md/markdown/manual/papers.md)
* [Contributors](docs/build/md/markdown/manual/credits.md)

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

* [motion_stack.api package](docs/build/md/markdown/api/motion_stack/motion_stack.api.md)
  * [Subpackages](docs/build/md/markdown/api/motion_stack/motion_stack.api.md#subpackages)
  * [Submodules](docs/build/md/markdown/api/motion_stack/motion_stack.api.md#submodules)
  * [motion_stack.api.ik_syncer module](docs/build/md/markdown/api/motion_stack/motion_stack.api.md#module-motion_stack.api.ik_syncer)
  * [motion_stack.api.joint_syncer module](docs/build/md/markdown/api/motion_stack/motion_stack.api.md#module-motion_stack.api.joint_syncer)
* [motion_stack.core package](docs/build/md/markdown/api/motion_stack/motion_stack.core.md)
  * [Subpackages](docs/build/md/markdown/api/motion_stack/motion_stack.core.md#subpackages)
  * [Submodules](docs/build/md/markdown/api/motion_stack/motion_stack.core.md#submodules)
  * [motion_stack.core.lvl1_joint module](docs/build/md/markdown/api/motion_stack/motion_stack.core.md#module-motion_stack.core.lvl1_joint)
  * [motion_stack.core.lvl2_ik module](docs/build/md/markdown/api/motion_stack/motion_stack.core.md#module-motion_stack.core.lvl2_ik)
  * [motion_stack.core.lvl4_mover module](docs/build/md/markdown/api/motion_stack/motion_stack.core.md#module-motion_stack.core.lvl4_mover)
* [motion_stack.ros2 package](docs/build/md/markdown/api/motion_stack/motion_stack.ros2.md)
  * [Subpackages](docs/build/md/markdown/api/motion_stack/motion_stack.ros2.md#subpackages)
  * [Submodules](docs/build/md/markdown/api/motion_stack/motion_stack.ros2.md#submodules)
  * [motion_stack.ros2.communication module](docs/build/md/markdown/api/motion_stack/motion_stack.ros2.md#module-motion_stack.ros2.communication)

# Operator TUI:

* [ms_operator package](docs/build/md/markdown/api/ms_operator/ms_operator.md)
  * [Submodules](docs/build/md/markdown/api/ms_operator/ms_operator.md#submodules)
  * [ms_operator.operator_node module](docs/build/md/markdown/api/ms_operator/ms_operator.md#ms-operator-operator-node-module)
  * [ms_operator.operator_tui module](docs/build/md/markdown/api/ms_operator/ms_operator.md#ms-operator-operator-tui-module)
  * [ms_operator.operator_utils module](docs/build/md/markdown/api/ms_operator/ms_operator.md#ms-operator-operator-utils-module)
