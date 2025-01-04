# motion_stack.ros2 package

ROS2 nodes and utils, using the motion stack core.

It creates the interface between ROS2’s systems and and the pure pyhton core.
Timer, Messages, Publisher, Subscription … all of those ROS2 tools are created here then executes functions of the core.

#### WARNING
Non-ROS2-related opertation must NOT be implemented here.

## Submodules

## motion_stack.ros2.lvl1_node module

ROS2 node managing the core of lvl1.

#### NOTE
I implemented this, not in OOP style, but full imperative. This might end-up being a bad idea, very bad idea.

### motion_stack.ros2.lvl1_node.create_advertise_service(node, lvl1)

Creates the advertise_joints service and its callback.

Callback returns a ReturnJointState.Response wich is a JointState with the name of all joints managed by the node. Other field of JointState are not meant to be used, but are filled with the latest data.

* **Parameters:**
  * **node** (*Node*) – spinning node
  * **lvl1** ([*JointCore*](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore)) – lvl1 core

### motion_stack.ros2.lvl1_node.link_publishers(node, lvl1)

Creates the publishers.

| Topic          | Type       | Note                  |
|----------------|------------|-----------------------|
| joint_commands | JointState | sent to motors (lvl0) |
| joint_read     | JointState | sent to IK (lvl2)     |
* **Parameters:**
  * **node** (*Node*) – spinning node
  * **lvl1** ([*JointCore*](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore)) – lvl1 core

### motion_stack.ros2.lvl1_node.link_subscribers(node, lvl1)

Creates the subscribers.

| Topic        | Type       | Note                       |
|--------------|------------|----------------------------|
| joint_states | JointState | coming from sensors (lvl0) |
| joint_set    | JointState | coming from IK (lvl2)      |
* **Parameters:**
  * **node** (*Node*) – spinning node
  * **lvl1** ([*JointCore*](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore)) – lvl1 core

### motion_stack.ros2.lvl1_node.frequently_send_lvl2(node, lvl1)

Creates a timer to send the fresh sensor state to the IK (lvl2).

This timer, instead of sending as soon as possible, introduces a very small latency but reduces compute load.

#### NOTE
If the sensor data is not “fresh” (meaning it is too similar to the previous state)
it will not be sent.

* **Parameters:**
  * **node** (*Node*) – spinning node
  * **lvl1** ([*JointCore*](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore)) – lvl1 core

### motion_stack.ros2.lvl1_node.startup_action(lvl1)

Actions to be executed on startup.

This sends an empty JointState with just the joint names to lvl0.

Make it execute on startup by passing it to `link_startup_action()`.

example:

```default
rclpy.init()
node = Node("lvl1")
spinner = Ros2Spinner(node)
lvl1 = JointNode(spinner)
link_startup_action(node, lvl1, startup_action)
```

* **Parameters:**
  **lvl1** ([*JointCore*](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore)) – lvl1 core

### motion_stack.ros2.lvl1_node.main(\*args, \*\*kwargs)

## motion_stack.ros2.lvl2_node module

## motion_stack.ros2.lvl4_node module
