# motion_stack.ros2 package

ROS2 specific API and nodes, for the motion stack core.

Links ROS2 systems and and the pure pyhton core.
Timer, Messages, Publisher, Subscription … all of those ROS2 tools are created here then executes functions of the core.

- [`ros2.base_node`](motion_stack.ros2.base_node.md#module-motion_stack.ros2.base_node) Provides the API template to use the python core through ROS2 nodes.
- [`ros2.default_node`](motion_stack.ros2.default_node.md#module-motion_stack.ros2.default_node) uses this API to make the default nodes.

#### WARNING
Non-ROS2-related opertation must NOT be implemented here.

## Subpackages

* [motion_stack.ros2.base_node package](motion_stack.ros2.base_node.md)
  * [Submodules](motion_stack.ros2.base_node.md#submodules)
  * [motion_stack.ros2.base_node.lvl1 module](motion_stack.ros2.base_node.md#module-motion_stack.ros2.base_node.lvl1)
* [motion_stack.ros2.default_node package](motion_stack.ros2.default_node.md)
  * [Submodules](motion_stack.ros2.default_node.md#submodules)
  * [motion_stack.ros2.default_node.lvl1 module](motion_stack.ros2.default_node.md#module-motion_stack.ros2.default_node.lvl1)
* [motion_stack.ros2.utils package](motion_stack.ros2.utils.md)
  * [Submodules](motion_stack.ros2.utils.md#submodules)
  * [motion_stack.ros2.utils.conversion module](motion_stack.ros2.utils.md#module-motion_stack.ros2.utils.conversion)
  * [motion_stack.ros2.utils.executor module](motion_stack.ros2.utils.md#module-motion_stack.ros2.utils.executor)
  * [motion_stack.ros2.utils.joint_state module](motion_stack.ros2.utils.md#module-motion_stack.ros2.utils.joint_state)
  * [motion_stack.ros2.utils.linking module](motion_stack.ros2.utils.md#module-motion_stack.ros2.utils.linking)

## Submodules

## motion_stack.ros2.communication module

Holds ros2 communication interface data.

It provides the names and types of every interface (topics, services, actions) used by the motion stack. So no need to remember the right name with the right spelling, import this and use communication.lvl1.output.motor_command.name

### *namedtuple* motion_stack.ros2.communication.Interf(type, name)

Bases: `NamedTuple`

Ros2 interface class with type and name

Interf(type, name)

* **Fields:**
  <a id="motion_stack.ros2.communication.Interf.name"></a>
  1.  **type** – Alias for field number 0
  2.  **name** – Alias for field number 1

### *class* motion_stack.ros2.communication.lvl1

Bases: `object`

#### alive *= (<class 'std_srvs.srv._empty.Empty'>, 'joint_alive')*

**Type:**    [`Interf`](#motion_stack.ros2.communication.Interf)

#### *class* output

Bases: `object`

#### motor_command *= (<class 'sensor_msgs.msg._joint_state.JointState'>, 'joint_commands')*

**Type:**    [`Interf`](#motion_stack.ros2.communication.Interf)

#### ik_command *= (<class 'sensor_msgs.msg._joint_state.JointState'>, 'joint_read')*

**Type:**    [`Interf`](#motion_stack.ros2.communication.Interf)

#### advertise *= (<class 'motion_stack_msgs.srv._return_joint_state.ReturnJointState'>, 'advertise_joints')*

**Type:**    [`Interf`](#motion_stack.ros2.communication.Interf)

#### *class* input

Bases: `object`

#### motor_sensor *= (<class 'sensor_msgs.msg._joint_state.JointState'>, 'joint_states')*

**Type:**    [`Interf`](#motion_stack.ros2.communication.Interf)

#### ik_command *= (<class 'sensor_msgs.msg._joint_state.JointState'>, 'joint_set')*

**Type:**    [`Interf`](#motion_stack.ros2.communication.Interf)
