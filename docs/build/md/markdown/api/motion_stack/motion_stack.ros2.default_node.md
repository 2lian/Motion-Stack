# motion_stack.ros2.default_node package

Default ROS2 nodes provided by the motion stack.

Created using [`ros2.base_node`](motion_stack.ros2.base_node.md#module-motion_stack.ros2.base_node) as the API.
Once you understand the API, feel free to overwrite methods of these default nodes in your own class. With this you can change or add behaviors.

## Submodules

## motion_stack.ros2.default_node.lvl1 module

### *class* motion_stack.ros2.default_node.lvl1.DefaultLvl1

Bases: [`Lvl1Node`](motion_stack.ros2.base_node.md#motion_stack.ros2.base_node.lvl1.Lvl1Node)

Default implementation of the Joint node of lvl1.

Refer to `ros2.base_node.lvl1` for documentation on linking ros2 and python core of lvl1. This only makes use of this base to create the default implementation and give an example.

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

**Service server:**

| Topic            | Type             | Note                                   |
|------------------|------------------|----------------------------------------|
| advertise_joints | ReturnJointState | JointState with the name of all joints |

**Timers:**
: - Sends to lvl2, freq.= ROS2_PARAMETER[`mvmt_update_rate`].

**Startup:**
: - Sends empty message to lvl0 with only joint names.

#### alive_srv *= (<class 'std_srvs.srv._empty.Empty'>, 'joint_alive')*

**Type:**    [`Interf`](motion_stack.ros2.md#motion_stack.ros2.communication.Interf)

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

#### startup_action(core)

* **Parameters:**
  **core** ([*JointCore*](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore))

### motion_stack.ros2.default_node.lvl1.create_advertise_service(node, lvl1)

Creates the advertise_joints service and its callback.

Callback returns a ReturnJointState.Response wich is a JointState with the name of all joints managed by the node. Other field of JointState are not meant to be used, but are filled with the latest data.

* **Parameters:**
  * **node** (*Node*) – spinning node
  * **lvl1** ([*JointCore*](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore)) – lvl1 core

### motion_stack.ros2.default_node.lvl1.main(\*args, \*\*kwargs)

## motion_stack.ros2.default_node.lvl2 module

### *class* motion_stack.ros2.default_node.lvl2.DefaultLvl2

Bases: [`Lvl2Node`](motion_stack.ros2.base_node.md#motion_stack.ros2.base_node.lvl2.Lvl2Node)

Default implementation of the Joint node of lvl2.

Refer to [`ros2.base_node.lvl2.Lvl2Node`](motion_stack.ros2.base_node.md#motion_stack.ros2.base_node.lvl2.Lvl2Node) for documentation on linking ros2 and python core of lvl1. This only makes use of this base to create the default implementation and give an example.

#### alive_srv *= (<class 'std_srvs.srv._empty.Empty'>, 'ik_alive')*

**Type:**    [`Interf`](motion_stack.ros2.md#motion_stack.ros2.communication.Interf)

#### subscribe_to_lvl1(lvl1_input)

* **Parameters:**
  **lvl1_input** (*Callable* *[* *[**List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]* *]* *,* *Any* *]*)

#### subscribe_to_lvl3(lvl3_input)

* **Parameters:**
  **lvl3_input** (*Callable* *[* *[*[*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose) *]* *,* *Any* *]*)

#### publish_to_lvl1(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*)

#### publish_to_lvl3(pose)

* **Parameters:**
  **pose** ([*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose))

#### startup_action(lvl2)

* **Parameters:**
  **lvl2** ([*IKCore*](motion_stack.core.md#motion_stack.core.lvl2_ik.IKCore))

### motion_stack.ros2.default_node.lvl2.main(\*args, \*\*kwargs)

## motion_stack.ros2.default_node.trial module

### *class* motion_stack.ros2.default_node.trial.TestNode

Bases: `Node`

#### *async* joints_ready()

#### *async* ik_ready()

#### json_step(n)

* **Parameters:**
  **n** (*int*)

#### *async* execute_json()

#### *async* zero()

#### *async* ik_square()

#### *async* ik_circle(samples=20)

* **Parameters:**
  **samples** (*int*)

#### *async* api_demo()

#### main()

#### startup()

#### loop()

### motion_stack.ros2.default_node.trial.main(\*args)
