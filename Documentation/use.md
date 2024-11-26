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

You should see a robot!

## Topics and example

To run those example ensure the robot is not automatically performing some movement, so disable the gait node of lvl 5 in [`general_launch_settings.py`](/general_launch_settings.py).
You can also launch only the levels you are interested in, this means launching up to lvl 1 to test lvl 1 features.

### Level 01: Joint node

Is the glue between the motion stack and lower levels like Rviz, simulation or real robot.
Its goal is to process joint states (sensor reading and motor commands).
Which joints are handled are decided based on the URDF and/or launch parameters. It can be responsible for only one joint, only one leg, only one robot or all joints it receives.

You can easily overload this node object in your own package and add functionalities to it.
A few tools are provided in `/src/easy_robot_control/easy_robot_control/injection/` (this will be explained in `TODO`).

Interface `JointState`: All angles, speeds and efforts describing several joints. Fused into one (or several) `JointState` messages according to [Ros2 doc](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html).

Topics:
- `joint_set` (Input from lvl2) `JointState`: Goal state for the joints
- `joint_read` (Output to lvl2) `JointState`: Current state of the joints
- `joint_commands` (Output to lvl0) `JointState`: Command sent to the motors
- `joint_states` (Input from lvl0) `JointState`: Sensors reading of the joints

Services:
- `advertise_joints` (Output) `JointState`: Returns the names (in the URDF) of all joints being handled by that node.
(Additionally returns the latest state data, with nan if no data is available.
But, this should not be used as a replacement to joint_read.)

List all joints handled by leg1 using:
```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 service call /leg1/advertise_joints custom_messages/srv/ReturnJointState
```
<!-- ``` -->
<!-- >>> -->
<!-- waiting for service to become available... -->
<!-- requester: making request: custom_messages.srv.ReturnJointState_Request() -->
<!---->
<!-- response: -->
<!-- custom_messages.srv.ReturnJointState_Response(\ -->
<!-- js=sensor_msgs.msg.JointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(\ -->
<!-- sec=1732604524, nanosec=228119773), frame_id=''), \ -->
<!-- name=['joint1-1', 'joint1-2', 'joint1-3'], \ -->
<!-- position=[0.0, 0.0, 0.0], \ -->
<!-- velocity=[nan, nan, nan], \ -->
<!-- effort=[nan, nan, nan])) -->
<!-- ``` -->

Read the angles:
```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 topic echo /leg1/joint_read
```
<!-- ``` -->
<!-- >>> -->
<!-- --- -->
<!-- header: -->
<!--   stamp: -->
<!--     sec: 1732604776 -->
<!--     nanosec: 75253027 -->
<!--   frame_id: '' -->
<!-- name: -->
<!-- - joint1-1 -->
<!-- - joint1-2 -->
<!-- - joint1-3 -->
<!-- position: -->
<!-- - 0.0 -->
<!-- - 0.0 -->
<!-- - 0.0 -->
<!-- velocity: [] -->
<!-- effort: [] -->
<!-- --- -->
<!-- ``` -->

Send an angle of 1 rad:
```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 topic pub /leg1/joint_set sensor_msgs/msg/JointState "{name: [joint1-2], position: [1.0], velocity: [], effort: []}"
```

Set angle command:

<img src="https://github.com/Space-Robotics-Laboratory/moonbot_software/assets/70491689/183d3cb1-420e-4da9-a490-9b98621b79a5" width="400"/>

### Level 02: IK node

This node loads the urdf to get all the kinematic information about its assigned leg.
It computes the IK of the given target and outputs the joint states toward lvl1.

Topics:
- `set_ik_target` (Input from lvl3) `Transform`: Target command for the end effector of the leg. Relative to the body center (`base_link`).
    - If less than 6 DoF leg, quaternion data is ignored.
- `tip_pos` (Output to lvl3) `Transform`: Publishes the Transform of the leg's end effector according to the joint angles reading.
- `joint_set` (Output to lvl1) `JointState`: see lvl1
- `joint_read` (Input from lvl1) `JointState`: see lvl1


```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 topic pub /leg1/set_ik_target geometry_msgs/msg/Transform "{translation: {x: 400, y: 0, z: -100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}" -1
```

```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 topic echo /leg1/tip_pos
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
ros2 service call /leg1/shift custom_messages/srv/TFService "{tf: {translation: {x: 0, y: 0, z: 100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 service call /leg1/tip_pos custom_messages/srv/ReturnVect3
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
