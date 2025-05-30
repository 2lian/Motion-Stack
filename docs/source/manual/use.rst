ROS2 nodes and interfaces
=========================

This section explains the ROS2 interfaces exposed by the Motion-Stack with examples.

.. Important::

   Ensure you sourced the workspace before running any of those commands: ``source ~/Motion-Stack/install/setup.bash``
        
        

Level 01: Joint
--------------------

.. important::

   This node's Python code is meant to be specialized for your robot (through wrapping, overwriting, injecting ...). Refer to :ref:`lvl1-api-label` to make the ROS2 interface/messages that suits your needs.

Lvl1 the glue between the motion stack and lower levels like Rviz, simulation or real robot.
Its goal is to process joint states (sensor reading and motor commands).
Handled joints are decided based on the URDF and/or launch parameters. It can be responsible for only one joint, one leg, one robot or all joints it receives.

**Source code:**
  * Python: :py:mod:`motion_stack.core.lvl1_joint`
  * Ros2 interface: :py:mod:`motion_stack.ros2.base_node.lvl1`
  * Ros2 default node: :py:mod:`motion_stack.ros2.default_node.lvl1`

**Topics:**
  * ``joint_set`` (**Input** from lvl2) ``JointState``: Goal state for the joints
  * ``joint_read`` (**Output** to lvl2) ``JointState``: Current state of the joints
  * ``joint_commands`` (**Output** to lvl0) ``JointState``: Command sent to the motors
  * ``joint_states`` (**Input** from lvl0) ``JointState``: Sensors reading of the joints

**Services:**
  * ``advertise_joints`` (**Output**) ``ReturnJointState``: Returns the names (in the URDF) of all joints being handled by that node.
    (Additionally returns the latest state data, with nan if no data is available. However, this should not be used as a replacement to joint_read.)

.. caution::

    ROS2 Message ``JointState`` does not guarantee the order, nor the existence of joints, nor the presence of each data array. \ `Refer to the the doc <http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html>`_. 
    
    Leveraging this, the motion stack uses a *lazy* communication strategy, hence a joint at rest will not be published at high frequency.

.. code-block:: bash

    # List all joints handled by leg1 using:
    ros2 service call /leg1/advertise_joints motion_stack_msgs/srv/ReturnJointState

.. code-block:: bash
    
    # Read the angles:
    ros2 topic echo /leg1/joint_read

.. code-block:: bash
    
    # Send an angle of 1 rad:
    ros2 topic pub /leg1/joint_set sensor_msgs/msg/JointState "{name: [joint1_2], position: [1.0], velocity: [], effort: []}"

.. image:: ../media/lvl1.gif
        :width: 400

Level 02: IK
-----------------

This node loads the urdf to get all the kinematic information about its assigned leg.
It computes the IK of the given target and outputs the joint states toward lvl1.

**Source code:**
  * Python: :py:class:`motion_stack.core.lvl2_ik`
  * Ros2 interface: :py:mod:`motion_stack.ros2.base_node.lvl2`
  * Ros2 default node: :py:mod:`motion_stack.ros2.default_node.lvl2`

**Topics:**
    - ``set_ik_target`` (**Input** from lvl3) ``Transform``: Target command for the end effector of the leg. Relative to the body center (``base_link``). (If less than 6 DoF leg, quaternion data is ignored.)
    - ``tip_pos`` (**Output** to lvl3) ``Transform``: Publishes the Transform of the leg's end effector according to the joint angles reading.
    - ``joint_set`` (**Output** to lvl1) ``JointState``: see lvl1
    - ``joint_read`` (**Input** from lvl1) ``JointState``: see lvl1



.. code-block:: bash
    
    # Send an IK target
    ros2 topic pub /leg1/set_ik_target geometry_msgs/msg/Transform "{translation: {x: 400, y: 0, z: -100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}" -1

.. code-block:: bash
    
    # Read the end effector position
    ros2 topic echo /leg1/tip_pos

.. image:: ../media/lvl2.gif
        :width: 400

Level 03: Leg
------------------

.. Caution::

   This level is to be deprecated in favor of the python API (:ref:`high-api-label`) and improved lvl4.

This node handles long running trajectories, outputing IK targets. It does not hold any dimension information.

Source code:
  * :py:class:`easy_robot_control.leg_node.LegNode`

Topics:
    - ``tip_pos`` (**Input** from lvl2) ``Transform``: See lvl 02.
    - ``set_ik_target`` (**Output** to lvl2) ``Transform``: See lvl 02.

Services:
    - ``rel_transl`` (**Input** from lvl4) ``TFService``: Translates the tip of the leg linearly to the target. (Origin is base_link)
    - ``shift`` (**Input** from lvl4) ``TFService``: Translates the tip of the leg linearly to the target. (Origin is current tip position, origin orientation is similar to *base_link*)
    - ``rel_hop`` (**Input** from lvl4) ``TFService``: jumps the tip of the leg to the traget. Trajectory goes up, then moves above the target before going down onto the target. (Origin is base_link)
    - ``rot`` (**Input** from lvl4) ``TFService``: Rotates the leg tip linearly, BUT !!! around the center specified by the TF. (Origin is base_link)
    - ``tip_pos`` (**Output** to lvl4) ``ReturnVect3``: Returns the current position of the tip of the leg or the target if the tip is close to it. (Origin is *base_link*)

.. Note::
    Use ``shift`` to rotate the leg tip with the center of rotation being the leg tip.

.. code-block:: bash
    
    # send a straight shift motion 100 mm upward
    ros2 service call /leg1/shift motion_stack_msgs/srv/TFService "{tf: {translation: {x: 0, y: 0, z: 100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

.. code-block:: bash

    # requests the tip position
    ros2 service call /leg1/tip_pos motion_stack_msgs/srv/ReturnVect3

Leg translation:
    .. image:: ../media/lvl3top.gif
        :width: 400

    .. image:: ../media/lvl3right.gif
        :width: 400

Leg hopping:
    .. image:: ../media/lvl3front.gif
        :width: 400


Level 04: Mover
---------------

Synchronizes several legs.

Source code:
  * :py:class:`easy_robot_control.mover_node.MoverNode`

Service:
    - ``body_tfshift`` (**Input** from lvl5) ``TFService``: Translates the body by the given TF.
    - ``get_targetset`` (**Input** form lvl4s) ``ReturnTargetSet``: Returns the current target set of the robot (list of ee coordinates)
    - ``legX/rel_transl`` ``legX/shift`` ``legX/rel_hop``  ``legX/rot`` (**Output** to lvl4): Refer to lvl4
    - ``legX/tip_pos`` (**Input** from lvl4) ``ReturnVect3``: Refer to lvl4.


.. code-block:: bash
    
    cd ${ROS2_MOONBOT_WS}
    . install/setup.bash
    ros2 service call body_tfshift motion_stack_msgs/srv/TFService "{tf: {translation: {x: 0, y: 0, z: 100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
    ros2 service call body_tfshift motion_stack_msgs/srv/TFService "{tf: {translation: {x: 0, y: 0, z: -100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
    ros2 service call body_tfshift motion_stack_msgs/srv/TFService "{tf: {translation: {x: 0, y: 0, z: 100}, rotation: {x: 0.1, y: 0.0, z: 0.0, w: 1.0}}}"
    ros2 service call body_tfshift motion_stack_msgs/srv/TFService "{tf: {translation: {x: 0, y: 0, z: 100}, rotation: {x: -0.1, y: 0.0, z: 0.0, w: 1.0}}}"

.. code-block:: bash
    
    cd ${ROS2_MOONBOT_WS}
    . install/setup.bash
    ros2 service call /get_targetset motion_stack_msgs/srv/ReturnTargetSet

Body translation:
    .. image:: ../media/lvl4.gif
        :width: 400

