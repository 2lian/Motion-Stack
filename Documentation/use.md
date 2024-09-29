# How to use

## What's this? A package? A workspace?

This repo is a whole workspace, this is not a package. You can easily take out and use the package [`src/easy_robot_control`](/src/easy_robot_control/) and [`src/urdf_packer/`](/src/urdf_packer/) for your own workspace. I think providing a fully working workspace instead of a lonely package is easier to understand.

## Settings

There are 3 main python files for the settings (I prefer .py over .xml as this allows for functions, scripts, math operation, LSP and more). All those settings are then sent to the nodes as ros2 parameters.
- [`general_launch_settings.py`](/general_launch_settings.py): settings between multiple packages, not only the motion stack
- [`/src/easy_robot_control/launch/launch_setting.py`](/src/easy_robot_control/launch/launch_setting.py): settings for the motion stack
- [`/src/rviz_basic/launch/rviz.launch.py`](/src/rviz_basic/launch/rviz.launch.py): settings to interface with Rviz

There are also setting .py files changeable at runtime, while the node is running. If import or the sanity test fails during runtime, the code will fallback to the .py version given at build time. Please run `pytest <runtime_setting.py>` to get a report about your .py. Also, launching with the provided .bash files will stop colcon build if the tests fail.
- [\src\easy_robot_control\easy_robot_control\python_package_include\pure_remap.py](\src\easy_robot_control\easy_robot_control\python_package_include\pure_remap.py):
  - Remaps the commands sent by and the states received by the joint node onto other joint names or topics.
  - Shapes all input/output individualy through python functions. (so you can apply gain, offset, limits and more to all joints)

## Launching

If you are having trouble launching the .bash files, open them and run the commands inside manually in your terminal. (Those .bash will source your Ros2)

Once your [urdf is setup](/Documentation/URDF_use.md), you can launch `/launch_only_rviz.bash` and `/launch_stack.bash`.
```bash
. launch_only_rviz.bash
```

`launch_stack.bash` will build everything then execute a launcher that launches other launchers (by default the motion stack and its joint state publisher for Rviz).
```bash
. launch_stack.bash
```

## Topics and example

### Level 01: Interface node

Replace or use this node with the interface to your simulation or robot.

Topics:
- `ang_<JointName>_set` (Input) `Float64`: Angle command for the joint named `<JointName>` in the URDF.
- `spe_<JointName>_set` (Input) `Float64`: Speed command for the joint named `<JointName>` in the URDF.
- `eff_<JointName>_set` (Input) `Float64`: Effort command for the joint named `<JointName>` in the URDF.
- `joint_states` (Output) `JointState`: All angle, speed and effort commands fused in one (or several) `JointState` messages according to [Ros2 doc](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html). This can be interpreted by Rviz, IsaacSim and others.
- `read_<JointName>` (Output) `Float64`: angle reading of the joint named `<JointName>` in the URDF. In the Rviz interface, the read simply sends back the last angle command.

```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 topic pub ang_<JointName>_set std_msgs/msg/Float64 "{data: 0.0}" -1
```

```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 topic echo angle_<JointName>
```

Set angle command:

<img src="https://github.com/Space-Robotics-Laboratory/moonbot_software/assets/70491689/183d3cb1-420e-4da9-a490-9b98621b79a5" width="400"/>

### Level 02: IK node

This node loads the urdf to get all the kinematic information about its assigned leg.

Topics:
- `set_ik_target_<LegNumber>` (Input) `Transform`: Target command for the end effector of the leg. Relative to the body center (`base_link`).
    - If less than 6 DoF leg, quaternion data is ignored.
    - If a wheel is detected, y of the transform is the wheel rotation axis, z is colinear with the axis of the last joint, so x points toward the "forward" of the wheel.
- `roll_<LegNumber>` (Input) `Float64`: Speed command for all the detected wheels.
    - If several wheels, with axis flipped in the URDF, this will be corrected and all will roll in the same direction.
- `tip_pos_{leg_number}` (Output) `Transform`: Publishes the Transform of the leg's end effector according to the joint angles reading.
- `ang_<JointName>_set` (Output) `Float64`: see level 01.
- `spe_<JointName>_set` (Output) `Float64`: see level 01. This is only used for the wheel rolling, not the other joints.
- `read_<JointName>` (Input) `Float64`: see level 01.


```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 topic pub set_ik_target_0 geometry_msgs/msg/Transform "{translation: {x: 400, y: 0, z: -100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}{}" -1
```

```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 topic echo tip_pos_0
```

IK target:

<img src="https://github.com/Space-Robotics-Laboratory/moonbot_software/assets/70491689/669b9239-099e-4af0-a420-506093914845" width="400"/>


### Level 03: Leg node

Topics:
- `smart_roll_<LegNumber>` (Input) `Float64`: sets the speed of the wheels. Depending on the last `point_toward` result, the roll direction needs to be flipped or not, hence the "smart".
- `tip_pos_<leg_number>` (Input) `Transform`: See lvl 02.
- `set_ik_target_<LegNumber>` (Output) `Transform`: See lvl 02.

Services: 
- `leg_<LegNumber>_rel_transl` (Input) `TFService`: Translates the tip of the leg linearly to the target. (Relative to the base_link)
- `leg_<LegNumber>_shift` (Input) `TFService`: Translates the tip of the leg linearly to the target. (Relative to the current tip position)
- `leg_<LegNumber>_rel_hop` (Input) `TFService`: jumps the tip of the leg to the traget. Trajectory goes up, then moves above the target before going down onto the target. (Relative to the base_link)
- `leg_<LegNumber>_rot` (Input) `TFService`: Rotates the leg tip linearly, BUT !!! around the center specified by the TF. (Relative to the base_link)
    - Use `leg_<LegNumber>_shift` to rotate the leg tip with the center of rotation being the leg tip.

```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 service call leg_0_shift custom_messages/srv/Vect3 "{vector: {x: 0, y: 0, z: 100}}"
```
Linear translations:

<img src="https://github.com/Space-Robotics-Laboratory/moonbot_software/assets/70491689/fd651f9c-3635-4757-a612-c663f727635e" width="400"/>
<img src="https://github.com/Space-Robotics-Laboratory/moonbot_software/assets/70491689/e7e17a1d-5f11-4bc3-b8ca-049189c212f7" width="400"/>

Leg hopping:

<img src="https://github.com/Space-Robotics-Laboratory/moonbot_software/assets/70491689/53dca6dc-381f-4ea3-8e5e-65317960c45c" width="400"/>


### Level 04

- Service: `body_shift` [`custom_messages/srv/Vect3`] Translates the body by the given vector.


```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 service call body_shift custom_messages/srv/Vect3 "{vector: {x: 50, y: 50, z: 0}}"
```

Body translation:

<img src="https://github.com/Space-Robotics-Laboratory/moonbot_software/assets/70491689/8f74a0f2-4a54-4997-bcdc-a1e6e6634cfc" width="400"/>
