# Moonbot software by Elian

## Prerequisties

* Ubuntu 20.04
* ros2-foxy
* Python
* numpy

## How to make it work with the robot

I need one subscriber per joint, listening to the topic `set_angle_{leg_number}_{joint number}_real` , 
messages will be of type `float32`. The motor should go at the angle that is received by this subscriber. 
Yes it is full and simple angle control for now. Yes that's all I need. Here is the detail:
- Replace `{leg_number}` with the number of the leg going from 0 to 3.
- Replace `{joint_number}` with the number of the joint on the leg going from 0 to 2.
- You will have 12 subscribers.

- For the legs assignment, when seen from above, 
  - `leg 0` is on the right (east), 
  - `leg 1` is at the top (north),
  - `leg 2` is left (west), 
  - `leg 3` is at the bottom (south).

- The angles should be in radiant.

- The angle of 0 mean the leg is straight (like a starfish).

- Positive angle on the joints `number 0` (of any legs), means the motor turns in the trigonometric direction 
(counterclockwise) when seen from the top.
- For joints `number 1` and `number 2` (of any legs), positive angle must be so the leg tip goes up towards the sky (from every angle is at 0).

- Please make sure it is working by sending messages on the topic manually and checking if the motor is moving at the right place.


## Shortcuts

- `${ROS2_INSTALL_PATH}` should point to your ros2 installation path (something like /opt/ros/foxy). use this command to write this shortcut in you bashrc:
````bash
echo 'export ROS2_INSTALL_PATH=/opt/ros/foxy' >> ~/.bashrc
````
- `${ROS2_MOONBOT_WS}` should point to where you cloned this repo
````bash
echo 'export ROS2_MOONBOT_WS=path_to_this_repo' >> ~/.bashrc
````

## Run

- `build_messages.bash` to build the custom messages used by this workspace
- `BRL_rviz.bash` to build rviz and launch rviz. There is one subscriber listening to
`set_angle_{leg_number}_{joint number}_real` for each joint angle on rviz. 
THAT'S IT, Rviz is NOT REQUIERED 
(but you need to change bypass_rviz_check to True in ik_node.py, otherwise the node will wait for Rviz before starting)
- Then you have several level of control, building above each other
  - level 2: `02BRL_easy_control.bash` build + launches the nodes responsable for the IK of each leg. 
You can publish on `set_ik_target_{leg_num}` to place the leg tip somewhere
  - level 3: `03BRL_easy_control.bash` build + launches level 2 AND the nodes responsable for smooth leg motion. 
You can call the service `leg_{leg_num}_rel_transl` to translate the leg tip somewhere in a straight line.
  - level 4: `04BRL_easy_control.bash` build + launches level 3 AND a node responsable for multi-leg coordination.
For now, launching this node will immediately execute a callback for 3 gait cycle forward.

## Settings

Settings are changed by modifying `src/easy_robot_control/launch/launch_setting.py`.
Notably, in there you can change the dimensions of the robot and movement speed. 

The dimensions I am using now are the one in the Rviz model I have. But this Rviz is NOT NECESSARY, so you can change
the dimension to anything you'd like in `launch_setting.py`, only the Rviz visuals will become wrong. 
Everthing robot and IK related will be right