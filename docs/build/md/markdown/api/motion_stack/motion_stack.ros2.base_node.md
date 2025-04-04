# motion_stack.ros2.base_node package

Provides the API template to use the python core through ROS2 nodes.

Create your own node from zero or refer to [`ros2.default_node`](motion_stack.ros2.default_node.md#module-motion_stack.ros2.default_node) for the default implementation

## Submodules

## motion_stack.ros2.base_node.lvl1 module

Template for a ROS2 node running the python core of lvl1.

### *class* motion_stack.ros2.base_node.lvl1.Lvl1Node

Bases: `Node`, `ABC`

Abstract base class for ros2, to be completed by the user.

To see the default behavior implemented using this template, refer to `ros2.default_node.lvl1`.

#### core_class

Class from which the core is instantiated. Overwrite this with a modified core to change behavior not related to ROS2.

alias of [`JointCore`](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore)

#### core

**Type:**    [`JointCore`](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore)

Instance of the python core.

#### *abstractmethod* subscribe_to_lvl0(lvl0_input)

Starts transmitting incomming **sensor data** to the python core.

`lvl0_input` is a function that must be called when new sensor data is available. The data type must be a list of JState.

Tipical steps:

> - Subscribe to a topic.
> - In the subscription callback:
>   > - Convert the incomming messages to a List[JState].
>   > - Call `lvl0_input` using your processed messages.

#### IMPORTANT
This function is called **once** at startup to setup some kind of continuous process. This continuous process must then call `lvl0_input`.

#### NOTE
`lvl0_input` is typically [`JointCore.coming_from_lvl0()`](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore.coming_from_lvl0)

* **Parameters:**
  **lvl0_input** (*Callable* *[* *[**List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]* *]* *,* *Any* *]*) – Interface function of the joint core, to call (in a callback) when new sensor data is available.

#### *abstractmethod* subscribe_to_lvl2(lvl2_input)

Starts transmitting incomming **joint targets** to the python core.

`lvl2_input` is a function that must be called when new joint targets (typically resulting from IK) is available. The data type must be a list of JState.

Tipical steps:

> - Subscribe to a topic.
> - In the subscription callback:
>   > - Convert the incomming messages to a List[JState].
>   > - Call `lvl2_input` using your processed messages.

#### IMPORTANT
This function is called **once** at startup to setup some kind of continuous process. This continuous process must then call `lvl2_input`.

#### NOTE
`lvl2_input` is typically [`JointCore.coming_from_lvl2()`](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore.coming_from_lvl2)

* **Parameters:**
  * **lvl0_input** – Interface function of the joint core, to call (in a callback) when new joint targets are available.
  * **lvl2_input** (*Callable* *[* *[**List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]* *]* *,* *Any* *]*)

#### *abstractmethod* publish_to_lvl0(states)

This method is called every time some **motor commands** need to be sent to lvl0.

`states` should be processed then sent onto the next step (published by ROS2).

Tipical steps:

> - Make a publisher in the \_\_init_\_.
> - Process `state` in a message.
> - call publisher.publish with your message

#### NOTE
This method will typically be called by [`JointCore.send_to_lvl0()`](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore.send_to_lvl0)

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*) – Joint states to be sent.

#### *abstractmethod* publish_to_lvl2(states)

This method is called every time some **joint states** need to be sent to lvl2.

`states` should be processed then sent onto the next step (published by ROS2).

Tipical steps:

> - Make a publisher in the \_\_init_\_.
> - Process `state` in a message.
> - call publisher.publish with your message

#### NOTE
This method will typically be called by [`JointCore.send_to_lvl2()`](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore.send_to_lvl2)

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*) – Joint states to be sent.

#### *abstractmethod* frequently_send_to_lvl2(send_function)

Starts executing `send_function` regularly.

Fresh sensor states must be send regularly to lvl2 (IK) using send_function. When using speed mode, it is also necessary to regularly send speed.

Tipical steps:

> - Make a timer.
> - Call send_function in the timer.

#### NOTE
`send_function` is typically [`JointCore.send_sensor_up()`](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore.send_sensor_up) and  [`JointCore.send_command_down()`](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore.send_command_down)

* **Parameters:**
  **send_function** (*Callable* *[* *[* *]* *,* *None* *]*) – Function sending fresh sensor states to lvl2

#### *abstractmethod* startup_action(core)

This will be executed *once* during the first ros spin of the node.

You can keep this empty, but typically:

> - a message with only joint names and no data is sent to initialise lvl0 (if using Rviz this step is pretty much necessary).
> - “alive” services are started to signal that the node is ready.
* **Parameters:**
  **core** ([*JointCore*](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore))

#### *classmethod* spin()

Spins the node

## motion_stack.ros2.base_node.lvl2 module

Template for a ROS2 node running the python core of lvl2.

### *class* motion_stack.ros2.base_node.lvl2.Lvl2Node

Bases: `Node`, `ABC`

Abstract base class for ros2, to be completed by the user.

To see the default behavior implemented using this template, refer to `ros2.default_node.lvl1.DefaultLvl2`.

#### core_class

Class from which the core is instantiated. Overwrite this with a modified core to change behavior not related to ROS2.

alias of [`IKCore`](motion_stack.core.md#motion_stack.core.lvl2_ik.IKCore)

#### core

**Type:**    [`IKCore`](motion_stack.core.md#motion_stack.core.lvl2_ik.IKCore)

Instance of the python core.

#### *abstractmethod* subscribe_to_lvl1(lvl1_input)

Starts transmitting incomming **state data** to the python core.

`lvl1_input` is a function that must be called when new state data is available. The data type must be a list of JState.

Tipical steps:

> - Subscribe to a topic.
> - In the subscription callback:
>   > - Convert the incomming messages to a List[JState].
>   > - Call `lvl1_input` using your processed messages.

#### IMPORTANT
This function is called **once** at startup to setup some kind of continuous process. This continuous process must then call `lvl0_input`.

#### NOTE
`lvl1_input` is typically [`IKCore.state_from_lvl1()`](motion_stack.core.md#motion_stack.core.lvl2_ik.IKCore.state_from_lvl1)

* **Parameters:**
  **lvl1_input** (*Callable* *[* *[**List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]* *]* *,* *Any* *]*) – Interface function of the ik core, to call (in a callback) when new state data is available.

#### *abstractmethod* subscribe_to_lvl3(lvl3_input)

Starts transmitting incomming **ik targets** to the python core.

`lvl3_input` is a function that must be called when new ik target is available. The data type must be a Pose.

Tipical steps:

> - Subscribe to a topic.
> - In the subscription callback:
>   > - Convert the incomming messages to a Pose.
>   > - Call `lvl3_input` with the processed messages.

#### IMPORTANT
This function is called **once** at startup to setup some kind of continuous process. This continuous process must then call `lvl3_input`.

#### NOTE
`lvl3_input` is typically [`IKCore.ik_target()`](motion_stack.core.md#motion_stack.core.lvl2_ik.IKCore.ik_target)

* **Parameters:**
  **lvl3_input** (*Callable* *[* *[*[*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose) *]* *,* *Any* *]*) – Interface function of the joint core, to call (in a callback) when new ik target is available.

#### *abstractmethod* publish_to_lvl1(states)

This method is called every time some **joint targets** need to be sent to lvl1.

`states` should be processed then sent onto the next step (published by ROS2).

Tipical steps:

> - Make a publisher in the \_\_init_\_.
> - Process `state` into a message.
> - call publisher.publish with your message

#### NOTE
This method will typically be called by [`IKCore.send_to_lvl1()`](motion_stack.core.md#motion_stack.core.lvl2_ik.IKCore.send_to_lvl1)

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*) – Joint states to be sent.

#### *abstractmethod* publish_to_lvl3(pose)

This method is called every time some **end effector pose** needs to be sent to lvl3.

`pose` should be processed then sent onto the next step (published by ROS2).

Tipical steps:

> - Make a publisher in the \_\_init_\_.
> - Process `pose` into a message.
> - call publisher.publish with your message

#### NOTE
This method will typically be called by [`IKCore.send_to_lvl3()`](motion_stack.core.md#motion_stack.core.lvl2_ik.IKCore.send_to_lvl3)

* **Parameters:**
  * **states** – Joint states to be sent.
  * **pose** ([*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose))

#### *abstractmethod* startup_action(lvl2)

This will be executed *once* during the first ros spin of the node.

You can keep this empty, but typically:

> - a message with only joint names and no data is sent to initialise lvl0 (if using Rviz this step is pretty much necessary).
> - “alive” services are started to signal that the node is ready.
* **Parameters:**
  **lvl2** ([*IKCore*](motion_stack.core.md#motion_stack.core.lvl2_ik.IKCore))

#### *classmethod* spin()

Spins the node
