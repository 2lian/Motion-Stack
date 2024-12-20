How to use
==========

What’s this? A package? A workspace?
------------------------------------

This repo is a whole workspace, this is not a package. You can easily
take out and use the package
```src/easy_robot_control`` </src/easy_robot_control/>`__ and
```src/urdf_packer/`` </src/urdf_packer/>`__ for your own workspace. I
think providing a fully working workspace instead of a lonely package is
easier to understand.

Parameter and Launchers
-----------------------

A customizable launching system is provided. It can be used (and
overloaded) by other packages. - Importable and customizable launch
tools are in
```/src/easy_robot_control/easy_robot_control/launch/`` </src/easy_robot_control/easy_robot_control/launch/>`__.
Those are meant to be reused over packages outside the motion stack. -
```/src/easy_robot_control/launch/default_params.py`` </src/easy_robot_control/launch/default_params.py>`__
defines the defaults parameters. Each parameters’ documentation is in
this file. -
```/src/easy_robot_control/launch/builder.py`` </src/easy_robot_control/launch/builder.py>`__
defines the LevelBuilder class. This will generate your nodes (and
launch description) depending on: - The name of the robot. - The
multiple end effectors. - Parameters to overwrite. - Those tools should
be imported in your own package and launcher
``from easy_robot_control.launch.builder import LevelBuilder`` to
generate the nodes and launch description. If you wish to change the
behaviors of those tools, you should `overload the class with your
changes <Documentation/API_for_DIY.md>`__. - Launchers for specific
robots and configurations are in
```/src/easy_robot_control/launch/`` </src/easy_robot_control/launch/>`__.
Those python scripts are not available to other packages. -
```/src/easy_robot_control/launch/moonbot_zero.launch.py`` </src/easy_robot_control/launch/moonbot_zero.launch.py>`__
is the launcher for moonbot_zero. - You can make you own launcher, in a
separate package, in your
``./src/YOUR_PKG/launch/YOUR_LAUNCHER.launch.py``. Take inspiration from
the moonbot_zero, you can import everything that
``moonbot_zero.launch.py`` imports.

`This section explains how to make your own
package <Documentation/API_for_DIY.md>`__, and overload those launch
tools to customise them for your robot structure.

Launching
---------

``.bash`` files are provided to build, source and launch the
moonbot_zero example. You can open the files and see what commands are
running. Nothing complicated. Change (or run yourself) the last line
``ros2 launch easy_robot_control moonbot_zero.launch.py MS_up_to_level:=4``
to launch your own launcher and change what levels are launched.

``launch_stack.bash`` will build everything then execute the launcher
for moonbot_zero.

.. code:: bash

   bash launch_stack.bash

You will notice that nothing is running, only waiting. This is because
the nodes are waiting for other nodes before continuing their execution
(in reality they wait for a service to be available). If it’s your first
time launching, the lvl1 is waiting for lvl0 which is the rviz
simulation node:

.. code:: bash

   bash launch_simu_rviz.bash  # (separate terminal)

You should see a robot!

``launch_simu_rviz.bash`` launches rviz and a simulation node that
imitates a motor’s response. When using the real robot, you must not use
this additional node (it will interfere with messages from the motors).
You should only launch rviz using ``launch_only_rviz.bash``

Topics and example
------------------

To run those example ensure the robot is not automatically performing
some movement from lvl5. Depending on what you do, you can select what
levels to launch using the arguments
``ros2 launch easy_robot_control moonbot_zero.launch.py MS_up_to_level:=2``,
this will launch levels 1 and 2.

Level 01: Joint node
~~~~~~~~~~~~~~~~~~~~

Is the glue between the motion stack and lower levels like Rviz,
simulation or real robot. Its goal is to process joint states (sensor
reading and motor commands). Handled joints are decided based on the
URDF and/or launch parameters. It can be responsible for only one joint,
one leg, one robot or all joints it receives.

`You can easily overload this node object in your own package and add
functionalities to it. <Documentation/API_for_DIY.md>`__ A few tools are
provided in ``/src/easy_robot_control/easy_robot_control/injection/``.

Interface ``JointState``: All angles, speeds and efforts describing
several joints. Fused into one (or several) ``JointState`` messages
according to `Ros2
doc <http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html>`__.

Topics: - ``joint_set`` (Input from lvl2) ``JointState``: Goal state for
the joints - ``joint_read`` (Output to lvl2) ``JointState``: Current
state of the joints - ``joint_commands`` (Output to lvl0)
``JointState``: Command sent to the motors - ``joint_states`` (Input
from lvl0) ``JointState``: Sensors reading of the joints

Services: - ``advertise_joints`` (Output) ``JointState``: Returns the
names (in the URDF) of all joints being handled by that node.
(Additionally returns the latest state data, with nan if no data is
available. But, this should not be used as a replacement to joint_read.)

List all joints handled by leg1 using:

.. code:: bash

   cd ${ROS2_MOONBOT_WS}
   . install/setup.bash
   ros2 service call /leg1/advertise_joints motion_stack_msgs/srv/ReturnJointState

.. raw:: html

   <!-- ``` -->

.. raw:: html

   <!-- >>> -->

.. raw:: html

   <!-- waiting for service to become available... -->

.. raw:: html

   <!-- requester: making request: motion_stack_msgs.srv.ReturnJointState_Request() -->

.. raw:: html

   <!---->

.. raw:: html

   <!-- response: -->

.. raw:: html

   <!-- motion_stack_msgs.srv.ReturnJointState_Response(\ -->

.. raw:: html

   <!-- js=sensor_msgs.msg.JointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(\ -->

.. raw:: html

   <!-- sec=1732604524, nanosec=228119773), frame_id=''), \ -->

.. raw:: html

   <!-- name=['joint1-1', 'joint1-2', 'joint1-3'], \ -->

.. raw:: html

   <!-- position=[0.0, 0.0, 0.0], \ -->

.. raw:: html

   <!-- velocity=[nan, nan, nan], \ -->

.. raw:: html

   <!-- effort=[nan, nan, nan])) -->

.. raw:: html

   <!-- ``` -->

Read the angles:

.. code:: bash

   cd ${ROS2_MOONBOT_WS}
   . install/setup.bash
   ros2 topic echo /leg1/joint_read

.. raw:: html

   <!-- ``` -->

.. raw:: html

   <!-- >>> -->

.. raw:: html

   <!-- --- -->

.. raw:: html

   <!-- header: -->

.. raw:: html

   <!--   stamp: -->

.. raw:: html

   <!--     sec: 1732604776 -->

.. raw:: html

   <!--     nanosec: 75253027 -->

.. raw:: html

   <!--   frame_id: '' -->

.. raw:: html

   <!-- name: -->

.. raw:: html

   <!-- - joint1-1 -->

.. raw:: html

   <!-- - joint1-2 -->

.. raw:: html

   <!-- - joint1-3 -->

.. raw:: html

   <!-- position: -->

.. raw:: html

   <!-- - 0.0 -->

.. raw:: html

   <!-- - 0.0 -->

.. raw:: html

   <!-- - 0.0 -->

.. raw:: html

   <!-- velocity: [] -->

.. raw:: html

   <!-- effort: [] -->

.. raw:: html

   <!-- --- -->

.. raw:: html

   <!-- ``` -->

Send an angle of 1 rad:

.. code:: bash

   cd ${ROS2_MOONBOT_WS}
   . install/setup.bash
   ros2 topic pub /leg1/joint_set sensor_msgs/msg/JointState "{name: [joint1-2], position: [1.0], velocity: [], effort: []}"

Set angle command:

Level 02: IK node
~~~~~~~~~~~~~~~~~

This node loads the urdf to get all the kinematic information about its
assigned leg. It computes the IK of the given target and outputs the
joint states toward lvl1.

Topics: - ``set_ik_target`` (Input from lvl3) ``Transform``: Target
command for the end effector of the leg. Relative to the body center
(``base_link``). - If less than 6 DoF leg, quaternion data is ignored. -
``tip_pos`` (Output to lvl3) ``Transform``: Publishes the Transform of
the leg’s end effector according to the joint angles reading. -
``joint_set`` (Output to lvl1) ``JointState``: see lvl1 - ``joint_read``
(Input from lvl1) ``JointState``: see lvl1

.. code:: bash

   cd ${ROS2_MOONBOT_WS}
   . install/setup.bash
   ros2 topic pub /leg1/set_ik_target geometry_msgs/msg/Transform "{translation: {x: 400, y: 0, z: -100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}" -1

.. code:: bash

   cd ${ROS2_MOONBOT_WS}
   . install/setup.bash
   ros2 topic echo /leg1/tip_pos

IK target:

Level 03: Leg node
~~~~~~~~~~~~~~~~~~

Topics: - ``tip_pos`` (Input from lvl2) ``Transform``: See lvl 02. -
``set_ik_target`` (Output to lvl2) ``Transform``: See lvl 02.

Services: - ``rel_transl`` (Input from lvl4) ``TFService``: Translates
the tip of the leg linearly to the target. (Relative to the base_link) -
``shift`` (Input from lvl4) ``TFService``: Translates the tip of the leg
linearly to the target. (Relative to the current tip position) -
``rel_hop`` (Input from lvl4) ``TFService``: jumps the tip of the leg to
the traget. Trajectory goes up, then moves above the target before going
down onto the target. (Relative to the base_link) - ``rot`` (Input from
lvl4) ``TFService``: Rotates the leg tip linearly, BUT !!! around the
center specified by the TF. (Relative to the base_link) - Use ``shift``
to rotate the leg tip with the center of rotation being the leg tip. -
``tip_pos`` (Output to lvl4) ``ReturnVect3``: Returns the current
position of the tip of the leg.

.. code:: bash

   cd ${ROS2_MOONBOT_WS}
   . install/setup.bash
   ros2 service call /leg1/shift motion_stack_msgs/srv/TFService "{tf: {translation: {x: 0, y: 0, z: 100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

.. code:: bash

   cd ${ROS2_MOONBOT_WS}
   . install/setup.bash
   ros2 service call /leg1/tip_pos motion_stack_msgs/srv/ReturnVect3

Linear translations:

Leg hopping:

Level 04
~~~~~~~~

Service: - ``body_tfshift`` (Input) ``TFService``: Translates the body
by the given TF. - ``get_targetset`` (Input) ``ReturnTargetSet``:
Returns the current target set of the robot (list of ee coordinates)

.. code:: bash

   cd ${ROS2_MOONBOT_WS}
   . install/setup.bash
   ros2 service call body_tfshift motion_stack_msgs/srv/TFService "{tf: {translation: {x: 0, y: 0, z: 100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
   ros2 service call body_tfshift motion_stack_msgs/srv/TFService "{tf: {translation: {x: 0, y: 0, z: -100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
   ros2 service call body_tfshift motion_stack_msgs/srv/TFService "{tf: {translation: {x: 0, y: 0, z: 100}, rotation: {x: 0.1, y: 0.0, z: 0.0, w: 1.0}}}"
   ros2 service call body_tfshift motion_stack_msgs/srv/TFService "{tf: {translation: {x: 0, y: 0, z: 100}, rotation: {x: -0.1, y: 0.0, z: 0.0, w: 1.0}}}"

.. code:: bash

   cd ${ROS2_MOONBOT_WS}
   . install/setup.bash
   ros2 service call /get_targetset motion_stack_msgs/srv/ReturnTargetSet

Body translation:
