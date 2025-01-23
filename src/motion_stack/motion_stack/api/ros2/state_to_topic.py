"""Provides StatesToTopics, to be injected in a Node.
see the class docstring for details
"""

from typing import Callable, Dict, Final, Iterable, List, Optional, Union

from rclpy.node import Node, Publisher
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from motion_stack.core.utils.robot_parsing import replace_incompatible_char_ros2
from motion_stack.ros2.base_node.lvl1 import Lvl1Node
from motion_stack.ros2.utils.joint_state import JState, ros2js

JSAtrr = str
JointName = str


def default_joint_to_topic_name(attribute: str, joint_name: str) -> str:
    """Return the topic name associated with an attribute and joint.

    Note:
        This is the default implementation. You might want to make your own.

    Args:
        attribute: position, velocity or effort
        joint_name: name of the joint

    Returns:
        name of the associated topic
    """
    topic_name = f"driver/{joint_name}/{attribute}"
    return topic_name


class StatesToTopic:
    """Publishes joint states onto individual topics.

    Features:
        - Publishes a list of JState or a JointStates onto individual Float64 topics
        - Provide joint_to_topic_name with the the naming convention you need
        - Lazily creates the topics as they are published
            - topics will not be created at startup, but the first time they are used
            - publish a state with np.nan instead of None to force the creation.

    Args:
        ros_node: ROS2 node
        joint_to_topic_name: Function returning the topic name associated with an attribute and joint.
    """

    def __init__(
        self,
        ros_node: Node,
        joint_to_topic_name: Callable[[str, str], str] = default_joint_to_topic_name,
    ) -> None:
        self._node = ros_node

        JS_ATTRIBUTES: Final[set[str]] = set(JState.__annotations__.keys()) - {
            "name",
            "time",
        }
        self.attributes: Iterable[str] = set(JS_ATTRIBUTES)
        self.make_topic_name = joint_to_topic_name

        self._pub2_dict: Dict[JSAtrr, Dict[JointName, Publisher]] = {}
        self._create_publisher = self._node.create_publisher

    @classmethod
    def setup_lvl0_command(
        cls,
        lvl1_ros_node: Lvl1Node,
        joint_to_topic_name: Callable[[str, str], str] = default_joint_to_topic_name,
    ) -> "StatesToTopic":
        """All joints will have their own individual float topic.

        Applies :py:class:`.state_to_topic.StatesToTopic` to outgoing motor commands of lvl1.

        Args:
            lvl1_ros_node: ROS2 node running lvl1
            joint_to_topic_name: Function returning the topic name associated with an attribute and joint.
        """
        s2t = cls(lvl1_ros_node, joint_to_topic_name)
        lvl1_ros_node.core.send_to_lvl0_callbacks.append(s2t.publish)
        return s2t

    def _make_topic_name(self, attribute: str, joint_name: str) -> str:
        # attribute = replace_incompatible_char_ros2(attribute)
        # joint_name = replace_incompatible_char_ros2(joint_name)
        raw = self.make_topic_name(attribute, joint_name)
        sanitized = replace_incompatible_char_ros2(raw)
        if raw != sanitized:
            self._node.get_logger().warn(
                f"Invalid ros2 topic name. Renamed: '{raw}' -> '{sanitized}'"
            )

        return sanitized

    def _get_create_motor_pub(self, attr: str, name: str) -> Publisher:
        """Return the publisher correspondong to the joint and attribute (pos, vel..).
        If does not exists, creates it

        Args:
            attr: position, velocity or effort
            name: joint name

        Returns:
            pub on topic "make_topic_name(attr, name)"
        """
        pudic: Optional[Dict[str, Publisher]] = self._pub2_dict.get(attr)
        if pudic is None:
            pudic = {}
            self._pub2_dict[attr] = pudic
        pub = pudic.get(name)
        if pub is None:
            pub = self._create_publisher(
                Float64,
                self._make_topic_name(attr, name),
                10,
            )
            self._pub2_dict[attr][name] = pub
        return pub

    def _pub_attribute(self, attr: str, state: JState):
        """publishes the given attribute of  the state"""
        value = getattr(state, attr, None)
        if value is None:  # if no data, does not publish
            return
        pub = self._get_create_motor_pub(attr, state.name)
        pub.publish(Float64(data=float(value)))

    def publish(self, states: Union[Iterable[JState], JointState]):
        """publishes a list of JState over float topics (lazily created)."""
        if isinstance(states, JointState):
            states = ros2js(states)
        for state in states:
            if state.name is None:
                continue
            for attr in self.attributes:
                self._pub_attribute(attr, state)
