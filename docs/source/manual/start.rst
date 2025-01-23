Quick start
============

.. important::
    **Changing the code for your robot might be the most important feature of the motion stack**, do not underestimate the power of yourself doing a better job than me who never saw your robot. The :ref:`api-label` section explains how to make your own package, launcher and node using the motion stack API.

What's this? A package? A workspace?
-------------------------------------

This repo is a whole workspace, this is not a package.
You can easily take out and use the package `src/motion_stack <https://github.com/2lian/Moonbot-Motion-Stack/blob/main/src/motion_stack>`_ and `src/motion_stack_msg <https://github.com/2lian/Moonbot-Motion-Stack/blob/main/src/motion_stack_msg>`_ for your own workspace.
I think providing a fully working workspace instead of a lonely package is easier to understand.

Executing
---------

``.bash`` files are provided to build, source and launch the moonbot_zero example. You can open the files and see what commands are running. Nothing complicated, it is the standrad ROS2 launch system.

``launch_stack.bash`` will build everything then execute the launcher for moonbot_zero.

.. code-block:: bash

    bash launch_stack.bash

**You will notice that nothing is running, only waiting.**
This is because the nodes are waiting for other nodes before starting (in reality they wait for a service to be available).
If it's your first time launching, the lvl1 is waiting for lvl0 which is the rviz simulation node:

.. code-block:: bash

    bash launch_simu_rviz.bash  # (separate terminal)

**You should see a robot!**

.. Note::
    ``launch_simu_rviz.bash`` launches rviz and a simulation node that imitates a motor's response. When using the real robot, you must not use this additional node (it will interfere with messages from the motors). You should launch rviz alone using ``launch_only_rviz.bash``

Parameters and Launchers
-------------------------

A customizable launching system is provided. It can be used (and modified) by your own packages.

.. Note::
    You do not need to use the launch API, it is just a wrapper around the standard ROS2 launch system.

.. Important::
    Tutorial explaining the launch API is provided in :ref:`api-label`.


- \ :py:data:`motion_stack.api.launch.default_params.default_params` defines the default ROS2 parameters. Parameter documentation is in this file.
- Launch APIs and tools are in :py:mod:`motion_stack.api.launch`. Those are exposed to packages outside the motion stack.

  - \ :py:mod:`motion_stack.api.launch.builder.LevelBuilder` will generate your nodes (and launch description) depending on:

    - Your URDF.
    - The multiple end effectors.
    - Parameters to overwrite.

  - You can import these in your own package and launcher:
    ``from motion_stack.api.launch.builder import LevelBuilder``

- Sample launchers for specific robots and configurations are in `src/motion_stack/launch <https://github.com/2lian/Moonbot-Motion-Stack/blob/main/src/motion_stack/launch/>`_. These Python scripts are not exposed to other packages.

  - `src/motion_stack/launch/moonbot_zero.launch.py <https://github.com/2lian/Moonbot-Motion-Stack/blob/main/src/motion_stack/launch/moonbot_zero.launch.py>`_ is the launcher for moonbot_zero.
