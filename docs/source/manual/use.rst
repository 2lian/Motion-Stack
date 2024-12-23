How to use
===========

What's this? A package? A workspace?
-------------------------------------

This repo is a whole workspace, this is not a package.
You can easily take out and use the package `src/easy_robot_control <https://github.com/2lian/Moonbot-Motion-Stack/blob/main/src/easy_robot_control>`_ and `src/urdf_packer <https://github.com/2lian/Moonbot-Motion-Stack/blob/main/src/urdf_packer/>`_ for your own workspace.
I think providing a fully working workspace instead of a lonely package is easier to understand.

Parameter and Launchers
------------------------

A customizable launching system is provided. It can be used (and overloaded) by your own packages.

- Launch APIs and tools are in :ref:`easy_robot_control.launch<launch-init-label>`. Those are meant to be exposed to packages outside the motion stack.
  
  - \ :ref:`easy_robot_control.launch.default_params<default-params-label>` defines the default parameters. Each parameter's documentation is in this file.
  - \ :ref:`easy_robot_control.launch.builder<level-builder-label>` defines the ``LevelBuilder`` class. This will generate your nodes (and launch description) depending on:

    - The name of the robot.
    - The multiple end effectors.
    - Parameters to overwrite.
  - These tools should be imported in your own package and launcher: 
    ``from easy_robot_control.launch.builder import LevelBuilder`` 
    to generate the nodes and launch description specific to your robot. You are encouraged to change the behaviors of those tools, refere to :ref:`the launch API<api-label>` for detailed explanation.
- Sample launchers for specific robots and configurations are in `src/easy_robot_control/launch <https://github.com/2lian/Moonbot-Motion-Stack/blob/main/src/easy_robot_control/launch/>`_. These Python scripts are not exposed to other packages, you cannot import them.

  - `src/easy_robot_control/launch/moonbot_zero.launch.py <https://github.com/2lian/Moonbot-Motion-Stack/blob/main/src/easy_robot_control/launch/moonbot_zero.launch.py>`_ is the launcher for moonbot_zero.
  - You can make your own launcher, in a separate package, in your ``./src/YOUR_PKG/launch/YOUR_LAUNCHER.launch.py``. Take inspiration from ``moonbot_zero.launch.py``, you can import everything that ``moonbot_zero.launch.py`` imports.

**Changing the code for your robot might be the most important feature of the motion stack**, do not underestimate the power of yourself doing a better job than me who never saw your robot. The :ref:`api-label` section explains how to make your own package, and overload :ref:`easy_robot_control.launch<launch-init-label>` specifically for your robot.
