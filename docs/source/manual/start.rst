Quick start
============

.. important::
    **Changing the code for your robot might be the most important feature of the motion stack**, do not underestimate the power of yourself doing a better job than me who never saw your robot. The :ref:`api-label` section explains how to make your own package, launcher and node using the motion stack API.

What's this? A package? A workspace?
-------------------------------------

This repo is a whole workspace, this is not a package.
You can easily take out and use the package `src/motion_stack <https://github.com/2lian/Motion-Stack/blob/main/src/motion_stack>`_ and `src/motion_stack_msg <https://github.com/2lian/Motion-Stack/blob/main/src/motion_stack_msg>`_ for your own workspace.
I think providing a fully working workspace instead of a lonely package is easier to understand.

Executing
---------

.. Tip::

   Run the following to have prettier terminal outputs from ROS 2

    .. code-block:: bash

        export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
        export RCUTILS_COLORIZED_OUTPUT=1

.. Tip::

    If you use pixi, don't forget to use ``pixi run <your command here>`` or enter the pixi shell with ``pixi shell``.

After building the workspace, you can launch the Moonbot Zero:

.. code-block:: bash

    ros2 launch motion_stack moonbot_zero.launch.py MS_down_from_level:=1 MS_up_to_level:=2 MS_simu_mode:=true


**You will notice that nothing is running, warning you about missing data.**
The Motion-Stack uses a data first approach, if data is missing it will warn you. This method avoids relying on the concept of nodes, participants or pings that can be brittle.

.. dropdown:: Missing lvl0 data: Terminal output

    .. code-block:: console

        [lvl2-14] [IK2] Forward Kinematics: MISSING ALL :( ['joint2_3', 'joint2_1', 'joint2_2']
        [lvl1-3] [J3] Joint data: Missing for ['joint3_3', 'joint3_1', 'joint3_2'].
        [lvl1-3] [J3] Joint data: MISSING ALL :(
        [lvl1-1] [J1] Joint data: Missing for ['joint1_1', 'joint1_3', 'joint1_2'].
        [lvl1-1] [J1] Joint data: MISSING ALL :(
        [lvl2-15] [IK3] Forward Kinematics: MISSING ALL :( ['joint3_2', 'joint3_3', 'joint3_1']
        [lvl2-16] [IK4] Forward Kinematics: MISSING ALL :( ['joint4_3', 'joint4_2', 'joint4_1']
        [lvl2-13] [IK1] Forward Kinematics: MISSING ALL :( ['joint1_1', 'joint1_3', 'joint1_2']
        [lvl1-2] [J2] Joint data: Missing for ['joint2_2', 'joint2_3', 'joint2_1'].
        [lvl1-2] [J2] Joint data: MISSING ALL :(
        [lvl1-4] [J4] Joint data: Missing for ['joint4_1', 'joint4_3', 'joint4_2'].
        [lvl1-4] [J4] Joint data: MISSING ALL :(

.. dropdown:: Available lvl0 data: Terminal output

    .. code-block:: console

        [lvl2-15] [IK3] Forward Kinematics: FULLY Ready :):
        [lvl2-15] Pose(time=1_757_306_662_886_225_483, xyz=[-528.58 -0.00 0.00], quat=[0.71 -0.71 0.00 -0.00])
        [lvl1-3] [J3] Joint data: Ready for ['joint3_2', 'joint3_1', 'joint3_3']
        [lvl1-3] [J3] Joint Data: FULLY READY :)
        [lvl2-16] [IK4] Forward Kinematics: FULLY Ready :):
        [lvl2-16] Pose(time=1_757_306_663_315_940_102, xyz=[0.38 -528.58 0.00], quat=[-0.50 0.50 0.50 -0.50])
        [lvl1-4] [J4] Joint data: Ready for ['joint4_1', 'joint4_3', 'joint4_2']
        [lvl1-4] [J4] Joint Data: FULLY READY :)
        [lvl2-13] [IK1] Forward Kinematics: FULLY Ready :):
        [lvl2-13] Pose(time=1_757_306_664_669_044_457, xyz=[528.58 -0.00 0.00], quat=[0.00 0.00 0.71 -0.71])
        [lvl2-14] [IK2] Forward Kinematics: FULLY Ready :):
        [lvl2-14] Pose(time=1_757_306_664_669_067_266, xyz=[-0.38 528.58 0.00], quat=[0.50 -0.50 0.50 -0.50])
        [lvl1-2] [J2] Joint data: Ready for ['joint2_3', 'joint2_2', 'joint2_1']
        [lvl1-2] [J2] Joint Data: FULLY READY :)
        [lvl1-1] [J1] Joint data: Ready for ['joint1_1', 'joint1_2', 'joint1_3']
        [lvl1-1] [J1] Joint Data: FULLY READY :)

If it's your first time launching, lvl1 is waiting for lvl0's data, which comes from the rviz simulation node (or you future robot). You can launch *rviz_simu* using the following:

.. code-block:: bash

   ros2 launch rviz_basic rviz_simu.launch.py # (separate terminal)

**You should see a robot!**

.. Caution::
    ``rviz_simu.launch.py`` launches rviz and a simulation node that imitates a motor's response. When using the real robot, you must not use this additional node (it will interfere with messages from the motors). You should launch rviz without simulation using ``rviz_vizu.launch.py``

After reading the documentation, you can have fun with the :ref:`tui`.

.. code-block:: bash
    
   bash operator.bash # (separate terminal)

Parameters and Launchers
-------------------------

A customizable launching system is provided. It can be used (and modified) by your own packages.

.. Important::
    Tutorial explaining the launch API is provided in :ref:`api-label`.


- \ :py:data:`motion_stack.core.utils.static_executor.default_param` defines the default parameters. 
  - Parameter documentation is in this file.
  - Those can be ROS2 parameters if you are using ROS2. However, those are part of th "core", hence defined in pure python.
- Launch APIs and tools are in :py:mod:`motion_stack.api.launch`. Those are exposed to packages outside the motion stack.

  - \ :py:mod:`motion_stack.api.launch.builder.LevelBuilder` will generate your nodes (and launch description) depending on:

    - Your URDF.
    - The multiple end effectors.
    - Parameters to overwrite.

  - You can import these in your own package and launcher:
    ``from motion_stack.api.launch.builder import LevelBuilder``

- Sample launchers for specific robots and configurations are in `src/motion_stack/launch <https://github.com/2lian/Motion-Stack/blob/main/src/motion_stack/launch/>`_. These Python scripts are not exposed to other packages.

  - `src/motion_stack/launch/moonbot_zero.launch.py <https://github.com/2lian/Motion-Stack/blob/main/src/motion_stack/launch/moonbot_zero.launch.py>`_ is the launcher for moonbot_zero.

.. Note::
    You do not need to use the launch API, it is just a wrapper around the standard ROS2 launch system.

