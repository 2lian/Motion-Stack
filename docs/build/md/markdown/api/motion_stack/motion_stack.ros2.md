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

### motion_stack.ros2.lvl1_node.make_advertise_service(node, lvl1)

* **Parameters:**
  * **node** (*Node*)
  * **lvl1** ([*JointNode*](motion_stack.core.md#motion_stack.core.lvl1_joint.JointNode))

### motion_stack.ros2.lvl1_node.inject_publishers(node, lvl1)

* **Parameters:**
  * **node** (*Node*)
  * **lvl1** ([*JointNode*](motion_stack.core.md#motion_stack.core.lvl1_joint.JointNode))

### motion_stack.ros2.lvl1_node.inject_subscribers(node, lvl1)

* **Parameters:**
  * **node** (*Node*)
  * **lvl1** ([*JointNode*](motion_stack.core.md#motion_stack.core.lvl1_joint.JointNode))

### motion_stack.ros2.lvl1_node.frequently_publish_lvl2(node, lvl1)

* **Parameters:**
  * **node** (*Node*)
  * **lvl1** ([*JointNode*](motion_stack.core.md#motion_stack.core.lvl1_joint.JointNode))

### motion_stack.ros2.lvl1_node.on_startup(node, lvl1)

* **Return type:**
  `Future`
* **Parameters:**
  * **node** (*Node*)
  * **lvl1** ([*JointNode*](motion_stack.core.md#motion_stack.core.lvl1_joint.JointNode))

### motion_stack.ros2.lvl1_node.main(\*args, \*\*kwargs)

## motion_stack.ros2.lvl2_node module

## motion_stack.ros2.lvl4_node module
