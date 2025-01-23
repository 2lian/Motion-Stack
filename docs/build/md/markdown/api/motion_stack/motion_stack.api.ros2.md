# motion_stack.api.ros2 package

## Submodules

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
