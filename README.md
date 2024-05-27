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
