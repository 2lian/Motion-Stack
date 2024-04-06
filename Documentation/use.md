# How to use

## Building

If you are having trouble launching the .bash files, open them and run the commands inside them manually in your terminal.

Build the messages used by this repo by running `build_messages.bash`. Those take a long time to compile, but you only need to recompile if you change them.
If you create new messages for your contribution to this repo, please add the build command to this `build_messages.bash`.

```bash
cd "${ROS2_MOONBOT_WS}" 
. build_messages.bash
```

Build and launch the Rviz package to see the robot using `BL_rviz.bash`. You do not need to restart rviz if you do not change stuff in the basic_rviz package, easy_robot_control will reconnect to it automatically (and other packages should also behave this way).

```bash
cd "${ROS2_MOONBOT_WS}" 
. BL_rviz.bash
```

If you are working with the real robot, please see this repo to interface with the motors: [dynamixel_hotplug_ros2_python](https://github.com/hubble14567/dynamixel_hotplug_ros2_python). As of now, you cannot have Rviz and the real robot at the same time because the data from both will interfere with each-other.

Build and launch easy_robot_control using `0XBRL_easy_control.bash`. As detailed in the design principles, several levels of control are available for ease of debugging:
- 02BRL_easy_control.bash: IK 
- 03BRL_easy_control.bash: Single leg movement
- 04BRL_easy_control.bash: Multi leg movement

```bash
cd "${ROS2_MOONBOT_WS}" 
. 04BRL_easy_control.bash
```

## Commands

### Level 01

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
