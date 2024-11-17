# How to use

## What's this? A package? A workspace?

This repo is a whole workspace, this is not a package.
You can easily take out and use the package [`src/easy_robot_control`](/src/easy_robot_control/) and [`src/urdf_packer/`](/src/urdf_packer/) for your own workspace.
I think providing a fully working workspace instead of a lonely package is easier to understand.

## Settings

The setting system is a bit special, I want to be able to change one parameter, then an entirely different robot is loaded. 
- Settings file for the motion stack are inside [`/src/easy_robot_control/launch/`](/src/easy_robot_control/launch/)
  - [`/src/easy_robot_control/launch/default_params.py`](/src/easy_robot_control/launch/default_params.py) sets defaults parameters for all your robots.
  Explanation about all parameters are in here.
  - [`/src/easy_robot_control/launch/moonbot_zero.py`](/src/easy_robot_control/launch/moonbot_zero.py) these are the parameters that will be used for the `moonbot_zero` robot.
  - Please make a python file `/src/easy_robot_control/launch/<your robot>.py` inside [`/src/easy_robot_control/launch/`](/src/easy_robot_control/launch/) that corresponds to your robot, in the style of [`/src/easy_robot_control/launch/moonbot_zero.py`](/src/easy_robot_control/launch/moonbot_zero.py)
    - This file must create a variable `params` containing your launch parameters
    - This file must create a list of nodes in the `levels` parameter, this correspond to lvl1, lvl2, lvl ...
- [`general_launch_settings.py`](/general_launch_settings.py) will set what you want to launch.
  - in [`general_launch_settings.py`](/general_launch_settings.py) the variable `LAUNCHPY` should be set to the filename of the settings you want to use.
  So if you made this [`/src/easy_robot_control/launch/<your robot>.py`], the variable `LAUNCHPY` should be `<your robot>`.
  (you can use python to set that variable for you)
  - [/robot_launcher.launch.py](/robot_launcher.launch.py) will load your [`general_launch_settings.py`](/general_launch_settings.py) then load the specified `/src/easy_robot_control/launch/<your robot>.py`, and launch everything.
- [`/src/rviz_basic/launch/rviz.launch.py`](/src/rviz_basic/launch/rviz.launch.py): settings for the interface to Rviz, directly at the top of the launchfile

There are also setting .py files reloaded at runtime, while the node is running.
If import or pytest fails during runtime, the code will fallback to the .py version given at build time.
Please run `pytest <the runtime setting.py>` to get a pytest report about your .py.
Note that, import and pytest checking are very basic, they are only meant to avoid obvious user errors, complex errors can still be introduced and crash the node.
Also, launching with the provided .bash files will stop colcon build and ros2 launch if the tests fail.
- [\src\easy_robot_control\easy_robot_control\python_package_include\pure_remap.py](\src\easy_robot_control\easy_robot_control\python_package_include\pure_remap.py):
  - Remaps, the commands sent by and, the states received by, the joint node onto other joint names or topics.
  - Shapes all input/output individualy through python functions.
  (so you can apply gain, offset, limits and more to all joints)

(Because thisw feature is bad safety-wise, you can disable this runtime reload behavior directly in the node source code by setting `DISABLE_AUTO_RELOAD = True`)

## Launching

If you are having trouble launching the .bash files, open them and run the commands inside manually in your terminal.
(Those .bash will source your Ros2)

Once your [urdf is setup](/Documentation/URDF_use.md), you can launch `/launch_only_rviz.bash` and `/launch_stack.bash`.


`launch_stack.bash` will build everything then execute a launcher that launches other launchers (by default the motion stack and its joint state publisher for Rviz).
```bash
bash launch_stack.bash
```

You will notice that nothing is running, only waiting.
This is because the nodes are waiting for other nodes before continuing their execution.
If it's your first time launching, the rviz interface is waiting for Rviz, launch Rviz and everything should start in the  right order:
```bash
bash launch_only_rviz.bash  # (separate terminal)
```

You should see a robot doing some movement!

## Topics and example

To run those example ensure the robot is not automatically performing some movement, so disable the gait node of lvl 5 in [`general_launch_settings.py`](/general_launch_settings.py).
You can also launch only the levels you are interested in, this means launching up to lvl 1 to test lvl 1 features.

### Level 01: Joint node

Is the glue between the motion stack and lower lower levels like Rviz, simulation or real robot.
Features runtime remapping of messages and shaping functions in [/src/easy_robot_control/easy_robot_control/python_package_include/pure_remap.py](/src/easy_robot_control/easy_robot_control/python_package_include/pure_remap.py).

Topics:
- `ang_<JointName>_set` (Input) `Float64`: Angle command for the joint named `<JointName>` in the URDF.
- `spe_<JointName>_set` (Input) `Float64`: Speed command for the joint named `<JointName>` in the URDF.
- `eff_<JointName>_set` (Input) `Float64`: Effort command for the joint named `<JointName>` in the URDF.

- `joint_commands` (Output) `JointState`: All angle, speed and effort commands (for the motors) fused in one (or several) `JointState` messages according to [Ros2 doc](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html).
This can be interpreted by Rviz, IsaacSim and others.
- `joint_states` (Input) `JointState`: All angle, speed and effort reading (from the sensors of the robot) fused in one (or several) `JointState` messages according to [Ros2 doc](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html).
Only the position is actively used by the joint node.
- `read_<JointName>` (Output) `Float64`: angle reading of the joint named `<JointName>` in the URDF.

Send an angle of 1 rad:
```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 topic pub /leg0/ang_<JointName>_set std_msgs/msg/Float64 "{data: 1.0}" -1
```
Read the angle:
```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 topic echo /leg0/read_<JointName>
```

Set angle command:

<img src="https://github.com/Space-Robotics-Laboratory/moonbot_software/assets/70491689/183d3cb1-420e-4da9-a490-9b98621b79a5" width="400"/>

### Level 02: IK node

This node loads the urdf to get all the kinematic information about its assigned leg.

Topics:
- `set_ik_target` (Input) `Transform`: Target command for the end effector of the leg.
Relative to the body center (`base_link`).
    - If less than 6 DoF leg, quaternion data is ignored.
    - If a wheel is detected, y of the transform is the wheel rotation axis, z is colinear with the axis of the last joint, so x points toward the "forward" of the wheel.
- `roll` (Input) `Float64`: Speed command for all the detected wheels.
    - If several wheels, with axis flipped in the URDF, this will be corrected and all will roll in the same direction.
- `tip_pos` (Output) `Transform`: Publishes the Transform of the leg's end effector according to the joint angles reading.

- `ang_<JointName>_set` (Output) `Float64`: see level 01.
- `spe_<JointName>_set` (Output) `Float64`: see level 01.
This is only used for the wheel rolling, not the other joints.
- `read_<JointName>` (Input) `Float64`: see level 01.


```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 topic pub /leg0/set_ik_target_0 geometry_msgs/msg/Transform "{translation: {x: 400, y: 0, z: -100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}" -1
```

```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 topic echo /leg0/tip_pos_0
```

IK target:

<img src="https://github.com/Space-Robotics-Laboratory/moonbot_software/assets/70491689/669b9239-099e-4af0-a420-506093914845" width="400"/>


### Level 03: Leg node

Topics:
- `smart_roll` (Input) `Float64`: sets the speed of the wheels.
Depending on the last `point_toward` result, the roll direction needs to be flipped or not, hence the "smart".
- `tip_pos` (Input) `Transform`: See lvl 02.
- `set_ik_target` (Output) `Transform`: See lvl 02.

Services: 
- `rel_transl` (Input) `TFService`: Translates the tip of the leg linearly to the target.
(Relative to the base_link)
- `shift` (Input) `TFService`: Translates the tip of the leg linearly to the target.
(Relative to the current tip position)
- `rel_hop` (Input) `TFService`: jumps the tip of the leg to the traget.
Trajectory goes up, then moves above the target before going down onto the target.
(Relative to the base_link)
- `rot` (Input) `TFService`: Rotates the leg tip linearly, BUT !!! around the center specified by the TF.
(Relative to the base_link)
    - Use `shift` to rotate the leg tip with the center of rotation being the leg tip.
- `tip_pos` (Output) `ReturnVect3`: Returns the current position of the tip of the leg.

```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 service call /leg0/leg_0_shift custom_messages/srv/TFService "{tf: {translation: {x: 0, y: 0, z: 100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 service call /leg0/leg_0_tip_pos custom_messages/srv/ReturnVect3
```
Linear translations:

<img src="https://github.com/Space-Robotics-Laboratory/moonbot_software/assets/70491689/fd651f9c-3635-4757-a612-c663f727635e" width="300"/>
<img src="https://github.com/Space-Robotics-Laboratory/moonbot_software/assets/70491689/e7e17a1d-5f11-4bc3-b8ca-049189c212f7" width="300"/>

Leg hopping:

<img src="https://github.com/Space-Robotics-Laboratory/moonbot_software/assets/70491689/53dca6dc-381f-4ea3-8e5e-65317960c45c" width="400"/>


### Level 04

Service:
- `body_tfshift` (Input) `TFService`: Translates the body by the given TF.
- `get_targetset` (Input) `ReturnTargetSet`: Returns the current target set of the robot (list of ee coordinates)


```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 service call body_tfshift custom_messages/srv/TFService "{tf: {translation: {x: 0, y: 0, z: 100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
ros2 service call body_tfshift custom_messages/srv/TFService "{tf: {translation: {x: 0, y: 0, z: -100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
ros2 service call body_tfshift custom_messages/srv/TFService "{tf: {translation: {x: 0, y: 0, z: 100}, rotation: {x: 0.1, y: 0.0, z: 0.0, w: 1.0}}}"
ros2 service call body_tfshift custom_messages/srv/TFService "{tf: {translation: {x: 0, y: 0, z: 100}, rotation: {x: -0.1, y: 0.0, z: 0.0, w: 1.0}}}"
```

```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 service call /get_targetset custom_messages/srv/ReturnTargetSet
```

Body translation:

<img src="https://github.com/Space-Robotics-Laboratory/moonbot_software/assets/70491689/8f74a0f2-4a54-4997-bcdc-a1e6e6634cfc" width="400"/>
