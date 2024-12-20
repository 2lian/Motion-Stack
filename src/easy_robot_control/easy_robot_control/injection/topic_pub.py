"""Provides StatesToTopics, to be injected in a Node.
see the class docstring for details
"""

from typing import Dict, Final, Iterable, List, Optional, Union

from rclpy.node import Node, Publisher
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from easy_robot_control.EliaNode import replace_incompatible_char_ros2
from easy_robot_control.utils.joint_state_util import JState, js_from_ros

JSAtrr = str
JointName = str


class StatesToTopic:
    """Publishes joint states onto individual topics, to be injected in a Node.

    Features:
        - Publishes a list of JState or a JointStates onto individual Float64 topics
        - Overload make_topic_name with the the naming convention you need
        - Lazily creates the topics as they are publishedtopics
            - topics will not be created at startup, but the first time they are used
            - publish np.nan and/or overload _pub_attribute to change this

    ====================== Injection sample code ====================
    from easy_robot_control.injection.topic_pub import StatesToTopic
    from rclpy.node import Node

    class MyStatesToTopic(StatesToTopic):
        def make_topic_name(self, attribute: str, joint_name: str) -> str:
            topic_name = f"canopen_motor/{joint_name}_joint_{attribute}_controller/command"
            return topic_name

    class Example(Node):
        def __init__(self):
            super().__init__()
            self.topic_pub = StatesToTopic(self)

        def send_to_lvl0(self, states):
            self.topic_pub.publish(states)
    """

    def __init__(self, joint_node: Node) -> None:
        self.__parent = joint_node

        JS_ATTRIBUTES: Final[set[str]] = set(JState.__annotations__.keys()) - {
            "name",
            "time",
        }
        self.attributes: Iterable[str] = set(JS_ATTRIBUTES)

        self.__pub2_dict: Dict[JSAtrr, Dict[JointName, Publisher]] = {}
        self.__create_publisher = self.__parent.create_publisher

    def make_topic_name(self, attribute: str, joint_name: str) -> str:
        """Return the topic name to create.
        Overload this with the topic naming style you need, for example

        return f"canopen_motor/{joint_name}_joint_{attribute}_controller/command"

        the output will be sanitize by __make_topic_name.

        Args:
            attribute: position, velocity or effort
            joint_name: name of the joint

        Returns:
            name of the associated topic
        """
        topic_name = f"driver/{joint_name}/{attribute}"
        return topic_name

    def __make_topic_name(self, attribute: str, joint_name: str) -> str:
        # attribute = replace_incompatible_char_ros2(attribute)
        # joint_name = replace_incompatible_char_ros2(joint_name)
        raw = self.make_topic_name(attribute, joint_name)
        sanitized = replace_incompatible_char_ros2(raw)
        if raw != sanitized:
            self.__parent.get_logger().warn(
                f"Invalid ros2 topic name. Renamed: '{raw}' -> '{sanitized}'"
            )

        return sanitized

    def __get_create_motor_pub(self, attr: str, name: str) -> Publisher:
        """Return the publisher correspondong to the joint and attribute (pos, vel..).
        If does not exists, creates it

        Args:
            attr: position, velocity or effort
            name: joint name

        Returns:
            pub on topic "make_topic_name(attr, name)"
        """
        pudic: Optional[Dict[str, Publisher]] = self.__pub2_dict.get(attr)
        if pudic is None:
            pudic = {}
            self.__pub2_dict[attr] = pudic
        pub = pudic.get(name)
        if pub is None:
            pub = self.__create_publisher(
                Float64,
                self.__make_topic_name(attr, name),
                10,
            )
            self.__pub2_dict[attr][name] = pub
        return pub

    def _pub_attribute(self, attr: str, state: JState):
        """publishes the given attribute of  the state"""
        value = getattr(state, attr, None)
        if value is None:  # if no data, does not publish
            return
        pub = self.__get_create_motor_pub(attr, state.name)
        pub.publish(Float64(data=float(value)))

    def publish(self, states: Union[Iterable[JState], JointState]):
        """publishes a list of JState over float topics (lazily created)."""
        if isinstance(states, JointState):
            states = js_from_ros(states)
        for state in states:
            if state.name is None:
                continue
            for attr in self.attributes:
                self._pub_attribute(attr, state)
