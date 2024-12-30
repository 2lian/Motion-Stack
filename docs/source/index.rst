Motion Stack
============

.. image:: https://img.shields.io/badge/Ubuntu-%2020.04%20%7C%2022.04%20-%20blue
   :target: https://ubuntu.com/
.. image:: https://img.shields.io/badge/Ros2-Foxy%20%7C%20Humble-%20blue
   :target: https://github.com/ros2
.. image:: https://img.shields.io/badge/Python-3.8_|_3.10-%20blue
   :target: https://www.python.org/
.. image:: https://raw.githubusercontent.com/petercorke/robotics-toolbox-python/master/.github/svg/rtb_powered.min.svg
   :target: https://github.com/petercorke/robotics-toolbox-python


.. image:: https://github.com/2lian/Moonbot-Motion-Stack/actions/workflows/stepbystep.yaml/badge.svg
   :target: https://github.com/2lian/Moonbot-Motion-Stack/actions/workflows/stepbystep.yaml
.. image:: https://img.shields.io/badge/License-MIT-gold
   :target: LICENSE

Modular walking robots or a single robotic arm, seamlessly bring your robots to life with just a URDF! Built for maximum flexibility, ease of use, and source-code customization.

.. toctree::
    :maxdepth: 1
    :caption: Guides:

    manual/install
    manual/start
    manual/use
    manual/URDF
    manual/api

.. raw:: html

   <h2>Features</h2>

---------------------

- **Modular**, any limb anywhere
- **Multi-limb synchronization**
- **Custom trajectories** *(in development)*
- **Inverse Kinematics** (3Dof and above)
- **URDF parser**
- **Customizable actuators interface** (overload the source-code with what you need)
- **Flexible launch system**
- **Documented example of Moonbot Zero**

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
    |               |             easy robot control
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
    |               |             easy robot control
    | ---------------------------------------------------------
    | Overload for my robot   |                        |  Overload for my robot
    | ---------------------------------------------------------

.. toctree::
   :maxdepth: 2
   :caption: Code:

   api/easy_robot_control/easy_robot_control

.. toctree::
   :maxdepth: 2
   :caption: Future code:

   api/motion_stack/motion_stack.api
   api/motion_stack/motion_stack.core
   api/motion_stack/motion_stack.ros2
