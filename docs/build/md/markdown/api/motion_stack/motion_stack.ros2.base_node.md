# motion_stack.ros2.base_node package

Provides the API template to use the python core through ROS2 nodes.

Create your own node from zero or refer to [`ros2.default_node`](motion_stack.ros2.default_node.md#module-motion_stack.ros2.default_node) for the default implementation

## Submodules

## motion_stack.ros2.base_node.lvl1 module

Template for a ROS2 node of Lvl1.

### *class* motion_stack.ros2.base_node.lvl1.Lvl1Node

Bases: `Node`, `ABC`

Abstract base class for ros2, to be completed by the user.

To see the default behavior implemented using this template, refer to `ros2.default_node.lvl1`.

#### core_class

Class from which the core is instantiated. Overwrite this with a modified core to change behavior not related to ROS2.

alias of [`JointCore`](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore)

#### lvl1

**Type:**    [`JointCore`](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore)

Instance of the python core.

#### *abstract* subscribe_to_lvl0(lvl0_input)

Starts the subscriber for lvl0 (angle sensors), transmitting incomming messages onto `lvl0_input`.

This function will be called ONCE for you, providing you the lvl0_input interface function of the joint core. It is your job to handle this interface function as you see fit.

Tipical steps:

> - Subscribe to a topic.
> - In the subscription callback:
>   > - Convert the incomming messages to a List[JState].
>   > - Call `lvl0_input` using your processed messages.

#### NOTE
`lvl0_input` is typically [`JointCore.coming_from_lvl0()`](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore.coming_from_lvl0)

* **Parameters:**
  **lvl0_input** (*Callable* *[* *[**List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]* *]* *,* *Any* *]*) – Interface function of the joint core to call in a callback using the processed message data.

#### *abstract* subscribe_to_lvl2(lvl2_input)

Starts the subscriber for lvl2 (IK commands), transmitting incomming messages onto `lvl2_input`.

This function will be called ONCE for you, providing you the lvl2_input interface function of the joint core. It is your job to handle this interface function as you see fit.

Tipical steps:

> - Subscribe to a topic.
> - In the subscription callback:
>   > - Convert the incomming messages to a List[JState].
>   > - Call `lvl2_input` using your processed messages.

#### NOTE
`lvl2_input` is typically [`JointCore.coming_from_lvl2()`](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore.coming_from_lvl2)

* **Parameters:**
  * **lvl0_input** – Interface function of the joint core to call in a callback using the processed message data.
  * **lvl2_input** (*Callable* *[* *[**List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]* *]* *,* *Any* *]*)

#### *abstract* publish_to_lvl0(states)

This method is called every time some states need to be sent to lvl0 (motor command).

It is your job to process then send the state data as you see fit.

Tipical steps:

> - Make a publisher in the \_\_init_\_.
> - Process `state` in a message.
> - call publisher.publish with your message

#### NOTE
This function will typically be executed by [`JointCore.send_to_lvl0()`](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore.send_to_lvl0)

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*) – Joint states to be sent.

#### *abstract* publish_to_lvl2(states)

This method is called every time some states need to be sent to lvl2 (IK state).

It is your job to process then send the state data as you see fit.

Tipical steps:

> - Make a publisher in the \_\_init_\_.
> - Process `state` in a message.
> - call publisher.publish with your message

#### NOTE
This function will typically be executed by [`JointCore.send_to_lvl2()`](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore.send_to_lvl2)

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*) – Joint states to be sent.

#### *abstract* frequently_send_to_lvl2(send_function)

Starts executing `send_function` regularly.

Fresh sensor states must be send regularly to lvl2 (IK) using send_function.
It is you job to call `send_function` regularly.

Tipical steps:

> - Make a timer.
> - Call send_function in the timer.

#### NOTE
`send_function` is typically [`JointCore.send_sensor_up()`](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore.send_sensor_up)

* **Parameters:**
  **send_function** (*Callable* *[* *[* *]* *,* *None* *]*) – Function sending fresh sensor states to lvl2

#### *abstract* startup_action(lvl1)

This will be executed once when the node starts.

You can keep this empty, but typically, a message with only joint names and not data is sent to initialise lvl0 (if using Rviz this step is pretty much necessary).

* **Parameters:**
  **lvl1** ([*JointCore*](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore))

#### *classmethod* spin()

Spins the node
