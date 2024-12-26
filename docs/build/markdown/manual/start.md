# How to start

#### IMPORTANT
**Changing the code for your robot might be the most important feature of the motion stack**, do not underestimate the power of yourself doing a better job than me who never saw your robot. The [API](api.md#api-label) section explains how to make your own package, launcher and node using the motion stack API.

## What’s this? A package? A workspace?

This repo is a whole workspace, this is not a package.
You can easily take out and use the package [src/easy_robot_control](https://github.com/2lian/Moonbot-Motion-Stack/blob/main/src/easy_robot_control) and [src/urdf_packer](https://github.com/2lian/Moonbot-Motion-Stack/blob/main/src/urdf_packer/) for your own workspace.
I think providing a fully working workspace instead of a lonely package is easier to understand.

## Executing

`.bash` files are provided to build, source and launch the moonbot_zero example. You can open the files and see what commands are running. Nothing complicated, it is the standrad ROS2 launch system.

`launch_stack.bash` will build everything then execute the launcher for moonbot_zero.

```bash
bash launch_stack.bash
```

**You will notice that nothing is running, only waiting.**
This is because the nodes are waiting for other nodes before starting (in reality they wait for a service to be available).
If it’s your first time launching, the lvl1 is waiting for lvl0 which is the rviz simulation node:

```bash
bash launch_simu_rviz.bash  # (separate terminal)
```

**You should see a robot!**

#### NOTE
`launch_simu_rviz.bash` launches rviz and a simulation node that imitates a motor’s response. When using the real robot, you must not use this additional node (it will interfere with messages from the motors). You should launch rviz alone using `launch_only_rviz.bash`

## Parameters and Launchers

A customizable launching system is provided. It can be used (and overloaded) by your own packages.

#### NOTE
Tutorial explaining the launch API is provided in [Launch API](api.md#launch-api-label).

- Launch APIs and tools are in [`easy_robot_control.launch`](../api/easy_robot_control.launch.md#module-easy_robot_control.launch). Those are meant to be exposed to packages outside the motion stack.
  - [`easy_robot_control.launch.default_params`](../api/easy_robot_control.launch.md#module-easy_robot_control.launch.default_params) defines the default parameters. Each parameter’s documentation is in this file.
  - [`easy_robot_control.launch.builder.LevelBuilder`](../api/easy_robot_control.launch.md#easy_robot_control.launch.builder.LevelBuilder) will generate your nodes (and launch description) depending on:
    - The name of the robot.
    - The multiple end effectors.
    - Parameters to overwrite.
  - These tools should be imported in your own package and launcher:
    `from easy_robot_control.launch.builder import LevelBuilder`
    to generate the nodes and launch description specific to your robot. You are encouraged to change the behaviors of those tools, refere to [Launch API](api.md#launch-api-label) for detailed explanation.
- Sample launchers for specific robots and configurations are in [src/easy_robot_control/launch](https://github.com/2lian/Moonbot-Motion-Stack/blob/main/src/easy_robot_control/launch/). These Python scripts are not exposed to other packages, you cannot import them.
  - [src/easy_robot_control/launch/moonbot_zero.launch.py](https://github.com/2lian/Moonbot-Motion-Stack/blob/main/src/easy_robot_control/launch/moonbot_zero.launch.py) is the launcher for moonbot_zero.
  - You can make your own launcher, in a separate package, in your `./src/YOUR_PKG/launch/YOUR_LAUNCHER.launch.py`. Take inspiration from `moonbot_zero.launch.py`, you can import everything that `moonbot_zero.launch.py` imports.
