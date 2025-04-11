Motion Stack
============

.. |br| raw:: html

   <br />

.. |lvl4| image:: media/lvl4.gif
          :alt: lvl4 whole body movement

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

.. |doit| image:: https://img.shields.io/badge/Automated_by-DOIT-006400
           :target: https://pydoit.org/

.. |test| image:: https://github.com/2lian/Motion-Stack/actions/workflows/doit_install.yaml/badge.svg
           :target: https://github.com/2lian/Motion-Stack/actions/workflows/doit_install.yaml

.. grid:: 2
    :gutter: 0

    .. grid-item::
     :columns: 12 7 6 6

     .. card::
       :img-background: ./media/lvl4.gif
       :img-alt: lvl4 whole body movement

    .. grid-item::
     :columns: 12 7 6 6

     .. card::

       |ubuntu|
       |ros|
       |python|
       |mit|

       |rtb|
       |doit|
       |test|

Modular walking robots with distributed computation, or a simple robotic arm, seamlessly bring robots to life with just a URDF! Built for maximum flexibility, ease of use, and source-code customization.

.. toctree::
    :maxdepth: 1
    :caption: Guides:

    manual/install
    manual/start
    manual/use
    manual/api

.. raw:: html

   <h2>Features</h2>

---------------------

- **Modular**, any limb anywhere on the robot
- **Distributed**, any process anywhere on the network
- **Hardware agnostic**, processes adapt to real-time robot characteristic
- **Inverse Kinematics** (3Dof and above)
- **Multi-limb synchronization**
- **URDF parser**
- **Customizable actuators interface** (overload the source-code with what you need)
- **Flexible launch system**
- **Documented example of Moonbot Zero**

.. raw:: html

   <h2>Upcomming Features</h2>

---------------------

- **Multi-limb-motor synchronization** *(in rework for agnostic system, lvl3 will be deleted)*
- **Custom trajectories** *(in development)*
- **High level Pyhton API**, API sends ros messages for you to any system


.. raw:: html

    <h2>Ros2 Structure Overview</h2>

---------------------

The current basic structure can be interpreted as the following tree:

.. code-block:: text
    
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

The power of this structure lies in its modularity. Packages responsible for a level can be swapped in/out for other packages responsible for the same level. 

For example:
- When using the real robot, `dynamixel_hotplug_ros2_python <https://github.com/hubble14567/dynamixel_hotplug_ros2_python>`_ is used.
- When testing without the robot, `rviz_basic <src/rviz_basic>`_ is used.

.. code-block:: text
    
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

All robots are different. You can easily overload relevant parts of the code and use it like an API in which you inject your custom code. Examples and tools are provided for this purpose. This way, you do not need to create a new, complex ROS2 node to adapt to the quirks of your robot—just change what you need directly.

.. code-block:: text
    
    |                       levels
    |      00       |    01   |   02  |   03  |   04   |  05   |
    | ---------------------packages----------------------------
    |               |             motion stack
    | ---------------------------------------------------------
    | Overload for my robot   |                        |  Overload for my robot
    | ---------------------------------------------------------


.. toctree::
   :maxdepth: 2
   :caption: Code:

   api/motion_stack/motion_stack.api
   api/motion_stack/motion_stack.core
   api/motion_stack/motion_stack.ros2

.. toctree::
   :maxdepth: 2
   :caption: Deprecated Code:

   api/easy_robot_control/easy_robot_control


.. toctree::
   :hidden:

   media/test_report

