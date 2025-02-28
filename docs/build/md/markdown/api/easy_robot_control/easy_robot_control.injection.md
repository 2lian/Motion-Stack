# easy_robot_control.injection package

<a id="injection-label"></a>

Provides objects adding functionalities to nodes through dependency injection.

## Submodules

## easy_robot_control.injection.offsetter module

### *class* easy_robot_control.injection.offsetter.OffsetterLvl0(parent, angle_path=None, offset_path=None)

Bases: `object`

Position offseter for lvl0, to be injected into a JointNode.
Usefull if your URDF and robot are not aligned.

Features
: - Inject this into any JointNode
  - Apply an angle offset to any joint of the lvl0 input/output.
  - Use a service to apply the offset at runtime
  - (Optional) Load offsets from a csv on disk.
  - (Optional) Save current angles multiplied by -1 every 3s in a csv on disk.This saved angle can tell you the last shutdown position of the robot, if you need to recover the offsets from it.

#### NOTE
- You should inject this object into a JointNode at the end of initialization.
- You do not need  to call any of those function, just inject this object.

* **Parameters:**
  * **parent** ([*JointNode*](easy_robot_control.md#easy_robot_control.joint_state_interface.JointNode))
  * **angle_path** (*str* *|* *None*) – if None, does not save the current angles on disk
  * **offset_path** (*str* *|* *None*) – if None, does not save or load offsets from disk

### Examples

- Injecting in a JointNode:
  ```default
  class Example(JointNode):
      def __init__(self):
          super().__init__()
          self.offsetter = OffsetterLvl0(
              self, angle_path=ANGLE_PATH, offset_path=OFFSET_PATH
          )
  ```
- Service call:
  ```default
  ros2 service call /leg1/set_offset motion_stack_msgs/srv/SendJointState "{js: {name: [joint1-2], position: [1], velocity: [], effort: []}}"
  ```

#### update_mapper(mapper_in=None, mapper_out=None)

Applies the offsets to a StateRemapper.

* **Parameters:**
  * **mapper_in** ([*StateRemapper*](easy_robot_control.utils.md#easy_robot_control.utils.state_remaper.StateRemapper) *|* *None*) – original map to which offset should be added
  * **mapper_out** ([*StateRemapper*](easy_robot_control.utils.md#easy_robot_control.utils.state_remaper.StateRemapper) *|* *None*) – affected subshaper of this map will change
* **Return type:**
  `None`

#### update_and_save_offset(js_offset)

Held offset values will be updated then saved on disk.

#### NOTE
Preferably use this to not lose the offset in case of restart

* **Parameters:**
  **js_offset** (*List* *[*[*JState*](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState) *]*) – list of offsets
* **Returns:**
  True if all offsets have a joint to be applied to
  String for user debugging
* **Return type:**
  `Tuple`[`bool`, `str`]

#### save_angle_as_offset(handlers=None)

Saves current position as the offset to recover to incase of powerloss.

#### NOTE
- Saved in self.angle_path
- To use those saves as offsets, replace the file <self.offset_path> with <self.angle_path>

* **Parameters:**
  **handlers** (*Dict* *[**str* *,* [*JointHandler*](easy_robot_control.md#easy_robot_control.joint_state_interface.JointHandler) *]*  *|* *None*)

#### load_offset()

Loads offset from offset csv. Skips unknown joints.

#### save_current_offset(to_save=None)

DO NOT DO THIS AUTOMATICALLY, IT COULD BE DESTRUCTIVE OF VALUABLE INFO.
overwrites the offset csv with the currently running offsets

* **Parameters:**
  **to_save** (*Dict* *[**str* *,* *float* *]*  *|* *None*)

#### update_offset(js_offset)

Updates offset in memory

* **Parameters:**
  **js_offset** (*List* *[*[*JState*](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState) *]*) – list of offsets
* **Returns:**
  True if all offsets have a joint to be applied to
  String for user debugging
* **Return type:**
  `Tuple`[`bool`, `str`]

## easy_robot_control.injection.topic_pub module

Provides StatesToTopics, to be injected in a Node.
see the class docstring for details

### *class* easy_robot_control.injection.topic_pub.StatesToTopic(joint_node)

Bases: `object`

Publishes joint states onto individual topics, to be injected in a Node.

Features:
: - Publishes a list of JState or a JointStates onto individual Float64 topics
  - Overload make_topic_name with the the naming convention you need
  - Lazily creates the topics as they are publishedtopics
    : - topics will not be created at startup, but the first time they are used
      - publish np.nan and/or overload \_pub_attribute to change this

====================== Injection sample code ====================
from easy_robot_control.injection.topic_pub import StatesToTopic
from rclpy.node import Node

class MyStatesToTopic(StatesToTopic):
: def make_topic_name(self, attribute: str, joint_name: str) -> str:
  : topic_name = f”canopen_motor/{joint_name}_joint_{attribute}_controller/command”
    return topic_name

class Example(Node):
: def \_\_init_\_(self):
  : super()._\_init_\_()
    self.topic_pub = StatesToTopic(self)
  <br/>
  def send_to_lvl0(self, states):
  : self.topic_pub.publish(states)

* **Parameters:**
  **joint_node** (*Node*)

#### make_topic_name(attribute, joint_name)

Return the topic name to create.
Overload this with the topic naming style you need, for example

return f”canopen_motor/{joint_name}_joint_{attribute}_controller/command”

the output will be sanitize by \_\_make_topic_name.

* **Parameters:**
  * **attribute** (*str*) – position, velocity or effort
  * **joint_name** (*str*) – name of the joint
* **Returns:**
  name of the associated topic
* **Return type:**
  `str`

#### publish(states)

publishes a list of JState over float topics (lazily created).

* **Parameters:**
  **states** (*Iterable* *[*[*JState*](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState) *]*  *|* *JointState*)
