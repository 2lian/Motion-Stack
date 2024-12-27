# easy_robot_control.injection.topic_pub module

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
  * **attribute** (`str`) – position, velocity or effort
  * **joint_name** (`str`) – name of the joint
* **Return type:**
  `str`
* **Returns:**
  name of the associated topic

#### publish(states)

publishes a list of JState over float topics (lazily created).

* **Parameters:**
  **states** (*Iterable* *[*[*JState*](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState) *]*  *|* *JointState*)
