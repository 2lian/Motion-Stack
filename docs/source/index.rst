Motion Stack
============

.. |br| raw:: html

   <br />

.. |ubuntu| image:: https://img.shields.io/badge/Ubuntu-%2020.04%20%7C%2022.04%20%7C%2024.04-%20blue
           :target: https://ubuntu.com/

.. |ros| image:: https://img.shields.io/badge/Ros2-Foxy%20%7C%20Humble%20%7C%20Jazzy-%20blue
           :target: https://github.com/ros2

.. |python| image:: https://img.shields.io/badge/Python-3.8_|_3.10_|_3.12-%20blue
           :target: https://www.python.org/

.. |mit| image:: https://img.shields.io/badge/License-MIT-gold
           :target: https://opensource.org/license/mit

.. |rtb| image:: https://img.shields.io/badge/Powered_by-Robotics_toolbox-006400
           :target: https://github.com/petercorke/robotics-toolbox-python

.. |pixi| image:: https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/prefix-dev/pixi/main/assets/badge/v0.json
           :target: https://pixi.sh

.. |test| image:: https://github.com/2lian/Motion-Stack/actions/workflows/doit_install.yaml/badge.svg
           :target: https://github.com/2lian/Motion-Stack/actions/workflows/doit_install.yaml

.. grid:: 2
    :gutter: 0

    .. grid-item::
     :columns: 12 7 6 6

     .. card::
       :img-background: ./media/landing.gif
       :img-alt: Synchronization of 6 robots

    .. grid-item::
     :columns: 12 7 6 6

     .. card::

       |ubuntu|
       |ros|
       |python|
       |mit|

       |rtb|
       |pixi|
       |test|

From modular robots with distributed computation, to a simple robotic arm, the motion stack provides control for (multi-)limbed systems. The goal of the project is maximum flexibility reflecting the flexibility of modular robotics, while abstracting away the complexity of such systems.

.. toctree::
    :maxdepth: 1
    :caption: Table of Contents:

    manual/install
    manual/start
    manual/use
    manual/api
    manual/operator_tui
    manual/credits

.. raw:: html

   <h2>Features</h2>

---------------------

- **Modular** -- any limb anywhere on the robot.
- **Distributed** -- any process anywhere on the network (ROS2 interface).
- **Runtime hardware agnosticism** -- adapts in real-time to robot characteristics.
- **Separation of concerns** -- team-member implementations and robot specificities, minimally impacts the other systems.
- **Inverse Kinematics** -- 3Dof and above.
- **Multi-limb synchronization**
- **TUI included** -- control your robot through a Terminal User Interface.
- **Customizable interfaces** -- Use the API and override the source-code for your robot and team.
- **URDF parser**
- **Flexible launch system**
- **Documented example of Moonbot Zero**

.. figure:: media/landingx3.gif
   :width: 100%
   :align: center

   Motion-Stack API synchronizing 3 different robots over the network, totaling 6 end-effectors.


.. raw:: html

   <h2>Upcomming Features</h2>

---------------------

- **Deprecation of lvl 3, 4** Level 3 and 4 have been replaced by the much safer and versatile high level API.
- Advanced launcher with new robot and advanced modularity.
- Zenoh interface to replace ROS2.


.. raw:: html

    <h2>Ros2 Structure Overview</h2>

---------------------

The current basic structure can be interpreted as the following tree:

.. code-block:: text

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

The power of this structure lies in its modularity. Levels can be modified,
swapped and assembled while remaining compatible with other levels. This can
happen anywhere on the network, lvl02 doesn't need to run on every robot, so
one can assemble several lvl1 and control them with one lvl2 IK.

Several higher levels, can also -- more or less -- control the same lower
level. This is very useful when several APIs control the same robot, so several
people can take control and do their experiment seamlessly.


.. toctree::
   :maxdepth: 2
   :caption: Code:

   api/motion_stack/motion_stack.api
   api/motion_stack/motion_stack.core
   api/motion_stack/motion_stack.ros2

.. toctree::
   :maxdepth: 2
   :caption: Operator TUI:

   api/ms_operator/ms_operator

.. toctree::
   :maxdepth: 2
   :caption: Deprecated Code:

   api/easy_robot_control/easy_robot_control


.. toctree::
   :hidden:

   media/test_report

