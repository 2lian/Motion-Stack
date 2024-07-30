# How to use

## Launching

If you are having trouble launching the .bash files, open them and run the commands inside them manually in your terminal.

Once your urdf is setup, you can launch `/launch_only_rviz.bash` and `/launch_stack.bash`.
```bash
. launch_only_rviz.bash
```

`launch_stack.bash` will build everything then execute a launcher that launches other launchers (by default the motion stack and its joint state publisher for Rviz).
```bash
. launch_only_rviz.bash
```
Please change the general settings of all those launchers directly in general_launch_settings.py. You can specify:
- The name of the robot's URDF you want to use
- The maximum level of the motion stack
- Interfaces you need
- The robot namespace (if given a list of namespaces, several robots will be launched)


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

Topics:
- `set_ik_target_<LegNumber>` (Input) `Transform`: Target command for the end effector of the leg. Relative to the body center (`base_link`).
    - If less than 6 DoF leg, quaternion data is ignored.
    - If a wheel is detected, y of the transform is the wheel rotation axis, z is colinear with the axis of the last joint, so x points toward the "forward" of the wheel.
- `roll_<LegNumber>` (Input) `Float64`: Speed command for all the detected wheels. 
    - If several wheels, with axis flipped in the URDF, this will be corrected and all will roll in the same direction.
- `tip_pos_{leg_number}` (Output) `Transform`: Publishes the Transform of the leg's end effector according to the joint angles reading.
- `ang_<JointName>_set` (Output) `Float64`: see level 01.
- `spe_<JointName>_set` (Output) `Float64`: see level 01.
- `read_<JointName>` (Input) `Float64`: see level 01.


```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 topic pub set_ik_target_0 geometry_msgs/msg/Vector3 "{x: 400, y: 0, z: -100}" -1
```

```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 topic echo tip_pos_0
```

IK target:

<img src="https://github.com/Space-Robotics-Laboratory/moonbot_software/assets/70491689/669b9239-099e-4af0-a420-506093914845" width="400"/>


### Level 03

- Service: `leg_0_rel_transl` [`custom_messages/srv/Vect3`] translates the tip of the leg in a straight line to the target.
- Service: `leg_0_rel_hop` [`custom_messages/srv/Vect3`] jumps the tip of the leg to the traget. Trajectory goes up, then moves above the target before going down onto the target.

```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 service call leg_0_rel_transl custom_messages/srv/Vect3 "{vector: {x: 400, y: 0, z: -100}}"
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
