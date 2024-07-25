# How to use

## Launching

If you are having trouble launching the .bash files, open them and run the commands inside them manually in your terminal.

Once your urdf is setup, you can launch `/launch_only_rviz.bash` and `/launch_stack.bash`.
```bash
. launch_only_rviz.bash
```

`launch_stack.bash` will build everything then execute a launcher that launches other launchers (by default the motion stack and its joint state publisher for Rviz).
```bash
. launch_stack.bash
```
Please change the general settings of all those launchers directly in `general_launch_settings.py`. You can specify: 
- The name of the robot's URDF you want to use
- The maximum level of the motion stack
- Interfaces you need
- The robot namespace. If given a list of namespaces, several robots (motion stack and interface) will be launched for each namespace.



## Commands

TODO:
- update lvl 01
- change Vector3 for the new version using tf2.

### Level 01 (interface):

This is a joint state publisher that interfaces with Rviz by default. Replace it with an interface to the motors or simulation software.

THE FOLLOWING IS OUTDATED (names have changed) FOR LEVEL01, instead: Directly listen to /joint_states to get angles in Ros2 JointStates format. Or use the topics looking like `/ang_<your joint name in the URDF>_set`

- Topic: `set_joint_{leg_number}_{joint number}_real` [`Float32`] to send an angle command to a joint.
- Topic: `angle_{leg_number}_{joint number}` [`Float32`] to listen to the angle of the joint.

```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 topic pub set_joint_0_1_real std_msgs/msg/Float64 "{data: 0.0}" -1
```

```bash
cd ${ROS2_MOONBOT_WS}
. install/setup.bash
ros2 topic echo angle_0_1
```

Set angle command:

<img src="https://github.com/Space-Robotics-Laboratory/moonbot_software/assets/70491689/183d3cb1-420e-4da9-a490-9b98621b79a5" width="400"/>

### Level 02

- Topic: `set_ik_target_{leg_number}` [`Vector3`] to send a tip postion to the IK and move the leg there.
- Topic: `tip_pos_{leg_number}` [`Vector3`] to listen to the position of the tip of the leg.

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
