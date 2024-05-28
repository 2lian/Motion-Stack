# Moonbot software by Elian

## Prerequisties

* Ubuntu LTS 22.04
* ROS2-humble
* Python

## Guides

* [installation](Documentation/installation.md).
* [design principle and contribution](Documentation/design_principles.md).
* [how to use](Documentation/use.md).

## Ros2 Structure overview

The current basic structure can be interpreted as this tree:
```  
                      levels
  01    |     02      |     03   |   04   |     05    |


Motor X -- Joint 0 -- |
Motor X -- Joint 1 -- +- IK 0 -- Leg 0 -- |
Motor X -- Joint 2 -- |                   |
                                          |
Motor X -- Joint 0 -- |                   |       
Motor X -- Joint 1 -- +- IK 1 -- Leg 1 -- +- Mover -- ...
Motor X -- Joint 2 -- |                   |
                                          |
                                  ...  -- |
```

The power of this structure can be seen below. Packages responsible for a level can be swapped in/out for other packages responsible of the same levels.
When using the real robot [dynamixel_hotplug_ros2_python](https://github.com/hubble14567/dynamixel_hotplug_ros2_python) is used. When trying things without the robot [rviz_basic](src/rviz_basic) is used.

```  
                      levels
  01    |     02   |     03   |   04   |     05    |
---------------------packages----------------------
                   |       easy robot control      |
---------------------------------------------------
      rviz basic   |
---------------------------------------------------
dynamixel_hotpl... |
```

Levels 03, 04 and 05 are available in:
- [easy_robot_control](src/easy_robot_control): Simple and easy inverse kinematics and movements

Levels 01 and 02 are available in:
- [rviz_basic](src/rviz_basic): Displays the robot fixed in Rviz.
- [dynamixel_hotplug_ros2_python](https://github.com/hubble14567/dynamixel_hotplug_ros2_python): Controls the real robot.

# Files and Folders

- `src/easy_robot_control` is the main Ros2 Package of this repo, providing motion control.
- `src/custom_messages` contains Ros2 messages used by this repo.
- `src/rviz_basic` is the interface to Rviz. Use this as a template for an interface with other systems
- `launch_stack_rviz.launch.py` is a launchfile launching other launchfiles with specified namespaces and settings. It typically launches Rviz interface, and all levels of the stack, while providing an easy way to change what interface and levels are being launched. Modify this to launch only the Rviz interface, or lvl 03, or lvl 02 ...
- `launch_stack.bash` Sources, builds, everything then launches `launch_stack_rviz.launch.py`.
- `launch_only_rviz.bash` Sources and launches the Rviz gui.
- `launch_rqt.bash` Sources and launches the RQT gui.

# My stuff that you should not look at:

- `src/pcl_reader` Displays a pointcloud from a numpy array, usefull to display a map in Rviz.
- `src/ros2_numpy-foxy-devel`
- `multi_auto_place.bash` Places several robots at the desired position, and the IK automatically choses the best foothold from the map.
- `robot_place.bash` Manully places multiple robots.
