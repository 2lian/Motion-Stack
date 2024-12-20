URDF parsing
============

The robot’s dimension and structure is loaded from a URDF, you should
not need to assign leg numbers and joint names manually. You only need
to launch the right number of leg node and specify the end effecctor you
want to use (if none is given, it will be chosen automatically based on
the leg number, if a number N is given, the Nth longest kinemaic chain
starting from the root of the URDF will be chosen).

Design and Code Principles
==========================

Important points
----------------

-  Minimize dependencies.
-  Make it work on any brand new linux machine.
-  Make it so a new student can launch it easily. Not only you.
-  Make your nodes independent and safe (one node crashing should make
   another node crash).
-  Do not delete, break or deeply change behavior of existing nodes of
   ``main``. Create a new function in an existing node, or a new node or
   new package instead.
-  Important values should be ros2 parameters set inside
   launch_settings.py and imported in the launch file. A node’s CODE.py
   should not change when parameters – such as leg dimensions, motor
   speed – are changed.

Overall ROS2 structure and design to respect
--------------------------------------------

The structure is meant to be like a tree with several levels: - Higher
level means the abstraction and responsibilities are higher. - Levels
can be composed of several nodes. - Levels should ONLY be dependent on
nodes below: - If levels below are missing: current level waits or
changes behavior (but does not crash). - If levels above are missing:
current level works as espected. - If nodes on same levels are missing:
current level works as espected - Commands flow down - Information flows
up

The current basic structure can be interpreted as this tree:

::

                         levels
     01    |     02   |     03   |   04   |     05    |


   Motor X -- Joint 0 -- |
   Motor X -- Joint 1 -- +- IK 0 -- Leg 0 -- |
   Motor X -- Joint 2 -- |                   |
                                             |
   Motor X -- Joint 0 -- |                   |       
   Motor X -- Joint 1 -- +- IK 1 -- Leg 1 -- +- Mover -- ...
   Motor X -- Joint 2 -- |                   |
                                             |
                                     ...  -- |

You can also see it as a pyramid, because the tree is simple:

::

   level XX       /    \      
                 /  ..  \     
   level 05     /  Mover \     x1
   level 04    /   Leg    \    x4
   level 03   /     IK     \   x4
   level 02  /     Joint    \  x12
   level 01 /      Motor     \ x12

This makes the structure robust and flexible but mainly easy to test,
debbug and understand. If a bug happens, levels can be isolated (for
example by launching only levels up to 02) and the ros2 components -
topics, services, parameters - can be checked with minimal dependencies.
New objects, nodes and functionalities can also be added easily by
adding or modifying branches of the tree.

The downside is that inter-node communication can look complex and
heavy. But there is not much data being sent so it’s not that bad. The
whole structure is also deeply asynchronous, this is a very good side
effect, but it is hard to understand at start.

Role of each node: - Mover: Synchronizes the movement of several legs -
Leg: Moves the tip of a leg from one target to another in a controlled
smooth motion - IK: Computes the joints angles for the leg tip to be on
target - Joint: Interprets and correct the joint angle before sending to
the corresponding motor (applies angle limits, flips angle, zeros) -
Motor: Handles hardware/software connection, receives and converts
angle, and rotates.

In English with example
~~~~~~~~~~~~~~~~~~~~~~~

The joint node handle everything related to the joint. It receives
target angles from the level above: IK (Inverse Kinematics), and sends
commands to the motor node. It also sends the current angle of the joint
to the levels above (IK and more).

The joint nodes shouldn’t rely on data of levels above, so the joint
does not have access to the position of the leg tip or the step
direction. The joint nodes only cares about level below, so only about
the motor. Similarly, the joint node cannot send commands to any node
above, so a joint node cannot ask directly for a step to be done.

Levels should ONLY be dependent on nodes below. So if the node leg #1 is
not available: - The IK, Joint and motors are still working fine
(because level below) - The other legs also work (because same level) -
The step node waits or needs to change its behavior by changing the gait
(because level above)

For your research
-----------------

Do not work on the ``main`` git branch
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This branch is meant to be used by everyone and should be a very solid
base for every new student to do research. Do not add broken code in
``main``. - If you plan on contributing or improving the code in
``main``: Create your own branch from ``main``. - Once you are done with
a feature or an improvement in your personal branch/fork. You should
create a pull request to merge your personal branch with ``main``, an
admin of the repo will approve your pull request. - Please do pull
request more or less frequently. You should not do one giant pull
request with 5000 lines of code after 2 years of work. It should be
incremental. First this is good for you not to be lost, second it is
easier for admins to approve your pull request. - If you do not plan on
contributing to, nor updating, the code in ``main``. You only want to
use it: create your own repo and clone this one inside. - If you plan on
contributing to ``main`` but your research is unrelated: Create your
branch from ``main`` AND keep your research SOMEWHERE ELSE in another
repo. - Use pull requests as explained above when you improve main. -
Manage your research repo yourself, we do not care about that here.

Clearly define which level and tree branch your work belongs to
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Your tree branch can replace several levels. Or make the tree deeper by
adding levels. Or widen the tree by adding new branches.

Examples of projects: - Motor update: should replace level 01. - New
simulation software: should replace level 01 (maybe 02). - New inverse
kinematics: Should replace only level 03. - Reinforcement learning to
move the robot: take place of level 03, 04 and 05 (depending on your
implementation). - Adding a wheel: New branch in level 04, update or
replace level 05 - Adding an IMU: new branch, unrelated to the legs, on
the side - Adding keyboard control: new level extending the tree -
Adding a simple csv reader: new branch to replace level 03 and above

::

                               levels
     01    |     02      |   03   |        04       |     05    |
     
     
   Motor ? -- Joint 0_0 -- |
   Motor ? -- Joint 0_1 -- +- IK 0 -- Leg 0 ---------- |
   Motor ? -- Joint 0_2 -- |                           |
                                                       |
   Motor ? -- Joint 1_0 -- |                           |       
   Motor ? -- Joint 1_1 -- +- IK 1 -- Leg 1 ---------- +- Mover -- |
   Motor ? -- Joint 1_2 -- |                           |           |
                                                       |           |
                                       ...  ---------- |           |

   Examples: .   .   .   .   .   .   .   .   .   .   .   .   .   .   .      
                                                       |           |
                                                       |           +- Keyboard
   Motor ? -- Joint 7_0 -- +- IK Wheel -- Leg Wheel -- |           |  
   Motor ? -- Joint 7_1 -- |                                       |
                                                                   |
                                                                   |
   IMU -- Orientation -------------------------------------------- |


   Motor ? -- Joint 0_0 -- |
   Motor ? -- Joint 0_1 -- |
   Motor ? -- Joint 0_2 -- |
                           + Easy CSV reader controller
   Motor ? -- Joint 1_0 -- |
   Motor ? -- Joint 1_1 -- |
   Motor ? -- Joint 1_2 -- |

Contributing and general ros2 advice
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-  Adding a new tool/feature should be done by creating a new package.

   -  Example: You create a package to move based on keys pressed on the
      keyboard. With two nodes: One node handles the keyboard, another
      handles the movement (sending messages to other nodes in already
      existing packages).

-  Adding a new object (a thing containing functions and states) should
   be done by creating a new node.

   -  Example: You add a node to support a joystick.

-  Adding a new functionality should be done by adding a new ros2
   component (topic, service, action …) to an existing node.

   -  Example: You add a subscription in the movement node to react to
      joystick messages.

-  Improving a functionality should be done by updating an existing
   function’s code. Preferabily without modifying any ros2 component.

   -  Example: You disable keyboard control if the joystick is used.

Deprecated
==========

Leg and joint numbering convention
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The body center is considered the origin. The vectors: - x points
forward (East) - y points left (North) - z points up

Folowing rules describe the leg numbering: - Template:
``leg_{leg_number}`` - first leg is ``leg_0`` - leg_number increases as
the z coordinate increases. - leg_number increases following the
trigonometric (counterclockwise) angle in the xy plane (East is first,
North second …).

Folowing rules describe the joint numbering and rotation direction: -
Template: ``joint_{leg_number}_{joint_number}`` - first joint is
``joint_0_0`` - joint_number increases as they get closer to the tip. -
angles are in radiants - angles at 0 means the leg is straight and all
frames of reference of this leg share the same orientation. - positive
angle direction should be consistent with the trigonometric definition
of an angle relative to the last joint frame of reference (movement
going from the axis x to y, y to z, z to x, are positive rotations).

Example of moonbot 0 joint subrscriber (4 legged robot)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In the URDF, joints are following the template exactly, so
``joint_0_0``, ``joint_0_1``\ …

In the Ros2 there is one subscriber per joint, listening to the topic
``set_joint_{leg_number}_{joint number}_real``, messages are of type
``float32``. The motor should go at the angle that is received by this
subscriber. Yes it is full and simple angle control on the
easy_robot_control package. Here are the details: - ``{leg_number}`` is
the number of the leg going from 0 to 3. - ``{joint_number}`` is the
number of the joint of the given the leg going from 0 to 2. - So there
are 12 subscribers.

-  For the legs assignment, when seen from above,

   -  ``leg 0`` is on the right (East),
   -  ``leg 1`` is at the top (North),
   -  ``leg 2`` is left (West),
   -  ``leg 3`` is at the bottom (South).

-  The angles should be in radiant.

-  The angles at 0 mean the leg is straight/flat (like a starfish).

-  Positive angle on the coxa joints ``number 0`` (of any legs), means
   the motor turns in the trigonometric direction (counterclockwise, so
   from East to North) when seen from the top.

-  For femur an dtibia joints ``number 1`` and ``number 2`` (of any
   legs), positive angle must be so the leg tip goes DOWN towards the
   ground (when starting from every angle at 0). This is counter
   intuitive, yes, but mathematically correct, be careful.

-  Please make sure it is working by sending messages on the topic
   manually and checking if the motor is moving at the right place.
