# motion_stack.api.ros2 package

## Submodules

## motion_stack.api.ros2.joint_api module

### *class* motion_stack.api.ros2.joint_api.JointHandler(node, limb_number)

Bases: `object`

ROS2 API to send/receive joint command/state to lvl1.

One instance is limited to a single limb.

#### NOTE
To safely execute joint movement to a target, do not directly use this class, but  use [`JointSyncerRos`](#motion_stack.api.ros2.joint_api.JointSyncerRos).

* **Parameters:**
  * **node** (*Node*) – Spinning node.
  * **limb_number** (*int*) – Limb number on which to interface with the joints.

#### tracked

**Type:**    `Set`[`str`]

Joint available on the limb

#### limb_number

**Type:**    `int`

limb number

#### new_state_cbk

**Type:**    List[Callable[[‘JointHandler’]]]

Callback executed when the state sensor updates. Argument is this object instance.

#### ready

**Type:**    `Future`

Future becoming done when sensor data is available on all tracked joints

#### *property* states

Accumulated joint state (sensor).

* **Return type:**
  `List`[[`JState`](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState)]

#### ready_up(tracked=None)

Starts timer looking for available joints and their data.

* **Parameters:**
  **tracked** (*Set* *[**str* *]*  *|* *None*) – Joints required to be considered ready.
* **Returns:**
  - [0] Future done when all available joints have data.
  - [1] Future done when the leg replies with the names of the available joints
* **Return type:**
  `Tuple`[`Future`, `Future`]

#### send(states)

Sends joint command to lvl1.

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*)

### *class* motion_stack.api.ros2.joint_api.JointSyncerRos(joint_handlers)

Bases: [`JointSyncer`](motion_stack.api.md#motion_stack.api.joint_syncer.JointSyncer)

Controls and syncronises several joints, safely executing trajectory to a target.

#### IMPORTANT
This class is a ROS2 implementation of the base class: [`api.joint_syncer.JointSyncer`](motion_stack.api.md#motion_stack.api.joint_syncer.JointSyncer). Refere to it for documentation.

* **Parameters:**
  **joint_handlers** (*List* *[*[*JointHandler*](#motion_stack.api.ros2.joint_api.JointHandler) *]*) – ROS2 objects handling joint communications of several limbs.

#### execute()

Executes one step of the task/trajectory.

This must be called frequently in a ros Timer or something else of your liking.

#### *property* sensor

#### IMPORTANT
This class is a ROS2 implementation of the base class: [`api.joint_syncer.JointSyncer`](motion_stack.api.md#motion_stack.api.joint_syncer.JointSyncer). Refere to it for documentation.

* **Return type:**
  `Dict`[`str`, [`JState`](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState)]

#### send_to_lvl1(states)

#### IMPORTANT
This class is a ROS2 implementation of the base class: [`api.joint_syncer.JointSyncer`](motion_stack.api.md#motion_stack.api.joint_syncer.JointSyncer). Refere to it for documentation.

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*)

#### *property* future_type

#### IMPORTANT
This class is a ROS2 implementation of the base class: [`api.joint_syncer.JointSyncer`](motion_stack.api.md#motion_stack.api.joint_syncer.JointSyncer). Refere to it for documentation.

* **Return type:**
  `Future`

## motion_stack.api.ros2.leg_api module

## motion_stack.api.ros2.offsetter module

### motion_stack.api.ros2.offsetter.setup_lvl0_offsetter(node, angle_recovery_path=None, offset_path=None)

* **Return type:**
  `Tuple`[`Timer`, `Service`]
* **Parameters:**
  * **node** ([*Lvl1Node*](motion_stack.ros2.base_node.md#motion_stack.ros2.base_node.lvl1.Lvl1Node))
  * **angle_recovery_path** (*str* *|* *None*)
  * **offset_path** (*str* *|* *None*)

## motion_stack.api.ros2.state_to_topic module

Provides StatesToTopics, to be injected in a Node.
see the class docstring for details

### motion_stack.api.ros2.state_to_topic.default_joint_to_topic_name(attribute, joint_name)

Return the topic name associated with an attribute and joint.

#### NOTE
This is the default implementation. You might want to make your own.

* **Parameters:**
  * **attribute** (*str*) – position, velocity or effort
  * **joint_name** (*str*) – name of the joint
* **Returns:**
  name of the associated topic
* **Return type:**
  `str`

### *class* motion_stack.api.ros2.state_to_topic.StatesToTopic(ros_node, joint_to_topic_name=<function 'default_joint_to_topic_name'>)

Bases: `object`

Publishes joint states onto individual topics.

Features:
: - Publishes a list of JState or a JointStates onto individual Float64 topics
  - Provide joint_to_topic_name with the the naming convention you need
  - Lazily creates the topics as they are published
    : - topics will not be created at startup, but the first time they are used
      - publish a state with np.nan instead of None to force the creation.

* **Parameters:**
  * **ros_node** (*Node*) – ROS2 node
  * **joint_to_topic_name** (*Callable* *[* *[**str* *,* *str* *]* *,* *str* *]*) – Function returning the topic name associated with an attribute and joint.

#### *classmethod* setup_lvl0_command(lvl1_ros_node, joint_to_topic_name=<function 'default_joint_to_topic_name'>)

All joints will have their own individual float topic.

Applies [`state_to_topic.StatesToTopic`](#motion_stack.api.ros2.state_to_topic.StatesToTopic) to outgoing motor commands of lvl1.

* **Parameters:**
  * **lvl1_ros_node** ([*Lvl1Node*](motion_stack.ros2.base_node.md#motion_stack.ros2.base_node.lvl1.Lvl1Node)) – ROS2 node running lvl1
  * **joint_to_topic_name** (*Callable* *[* *[**str* *,* *str* *]* *,* *str* *]*) – Function returning the topic name associated with an attribute and joint.
* **Return type:**
  [`StatesToTopic`](#motion_stack.api.ros2.state_to_topic.StatesToTopic)

#### publish(states)

publishes a list of JState over float topics (lazily created).

* **Parameters:**
  **states** (*Iterable* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*  *|* *JointState*)
