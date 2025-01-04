# motion_stack.api.ros2 package

Public ROS2 API for injection and overloading ofthe default (private) node package

## Submodules

## motion_stack.api.ros2.communication module

### *namedtuple* motion_stack.api.ros2.communication.Interf(type, name)

Bases: `NamedTuple`

Ros2 interface class with type and name

Interf(type, name)

* **Fields:**
  <a id="motion_stack.api.ros2.communication.Interf.name"></a>
  1.  **type** – Alias for field number 0
  2.  **name** – Alias for field number 1

### *class* motion_stack.api.ros2.communication.lvl1

Bases: `object`

#### *class* output

Bases: `object`

#### motor_command *= (<class 'sensor_msgs.msg._joint_state.JointState'>, 'joint_command')*

**Type:**    [`Interf`](#motion_stack.api.ros2.communication.Interf)

#### ik_command *= (<class 'sensor_msgs.msg._joint_state.JointState'>, 'joint_read')*

**Type:**    [`Interf`](#motion_stack.api.ros2.communication.Interf)

#### advertise *= (<class 'motion_stack_msgs.srv._return_joint_state.ReturnJointState'>, 'advertise_joints')*

**Type:**    [`Interf`](#motion_stack.api.ros2.communication.Interf)

#### *class* input

Bases: `object`

#### motor_sensor *= (<class 'sensor_msgs.msg._joint_state.JointState'>, 'joint_state')*

**Type:**    [`Interf`](#motion_stack.api.ros2.communication.Interf)

#### ik_command *= (<class 'sensor_msgs.msg._joint_state.JointState'>, 'joint_set')*

**Type:**    [`Interf`](#motion_stack.api.ros2.communication.Interf)

## motion_stack.api.ros2.lvl1 module

Template to make a ROS2 node of Lvl1, and the default node.

### *class* motion_stack.api.ros2.lvl1.Lvl1Node

Bases: `Node`, `ABC`

Abstract base class for ros2, to be completed by the user.

To see the default behavior implemented using this template, refer to [`lvl1.Lvl1Default`](#motion_stack.api.ros2.lvl1.Lvl1Default) source code.

#### lvl1

**Type:**    [`JointCore`](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore)

Pure python core of lvl1

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

#### spin()

Spins the node

### *class* motion_stack.api.ros2.lvl1.Lvl1Default

Bases: [`Lvl1Node`](#motion_stack.api.ros2.lvl1.Lvl1Node)

Default implementation of the Joint node of lvl1.

**Publishers:**

| Topic          | Type       | Note                  |
|----------------|------------|-----------------------|
| joint_commands | JointState | sent to motors (lvl0) |
| joint_read     | JointState | sent to IK (lvl2)     |

**Subscribers:**

| Topic        | Type       | Note                       |
|--------------|------------|----------------------------|
| joint_states | JointState | coming from sensors (lvl0) |
| joint_set    | JointState | coming from IK (lvl2)      |

Timers:
: - Sends to lvl2, freq.= ROS2_PARAMETER[`mvmt_update_rate`].

Startup:
: - Sends empty message to lvl0 with only joint names.

#### subscribe_to_lvl2(lvl2_input)

* **Parameters:**
  **lvl2_input** (*Callable* *[* *[**List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]* *]* *,* *Any* *]*)

#### subscribe_to_lvl0(lvl0_input)

* **Parameters:**
  **lvl0_input** (*Callable* *[* *[**List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]* *]* *,* *Any* *]*)

#### publish_to_lvl0(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*)

#### publish_to_lvl2(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*)

#### frequently_send_to_lvl2(send_function)

* **Parameters:**
  **send_function** (*Callable* *[* *[* *]* *,* *None* *]*)

#### startup_action(lvl1)

* **Parameters:**
  **lvl1** ([*JointCore*](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore))
