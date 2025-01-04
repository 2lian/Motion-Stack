"""Template to make a ROS2 node of Lvl1, and the default node."""

from abc import ABC, abstractmethod
from typing import Any, Callable, List

from rclpy.node import Node
from sensor_msgs.msg import JointState

from .communication import lvl1
from ...ros2.lvl1_node import create_advertise_service
from ...ros2.utils.linking import CallablePublisher, link_startup_action

from ...core.lvl1_joint import JointCore
from ...core.utils.joint_state import JState
from ...ros2.utils import joint_state as js_utils
from ...ros2.utils.executor import Ros2Spinner, error_catcher, my_main


class Lvl1Node(Node, ABC):
    """Abstract base class for ros2, to be completed by the user.

    To see the default behavior implemented using this template, refer to :py:class:`.lvl1.Lvl1Default` source code.
    """

    _spinner: Ros2Spinner
    lvl1: JointCore  #: Pure python core of lvl1

    def __init__(self):
        super().__init__("lvl1")
        self._spinner = Ros2Spinner(self)
        self.lvl1 = JointCore(self._spinner)
        self._link_publishers()
        self._link_subscribers()
        self._on_startup()
        self._spinner.wait_for_lower_level()

    @abstractmethod
    def subscribe_to_lvl0(self, lvl0_input: Callable[[List[JState]], Any]):
        r"""Starts the subscriber for lvl0 (angle sensors), transmitting incomming messages onto ``lvl0_input``.

        This function will be called ONCE for you, providing you the lvl0_input interface function of the joint core. It is your job to handle this interface function as you see fit.

        Tipical steps:

            - Subscribe to a topic.
            - In the subscription callback:

                - Convert the incomming messages to a List[JState].
                - Call ``lvl0_input`` using your processed messages.

        .. Note::

            \ ``lvl0_input`` is typically :py:meth:`.JointCore.coming_from_lvl0`

        Args:
            lvl0_input: Interface function of the joint core to call in a callback using the processed message data.
        """
        ...

    @abstractmethod
    def subscribe_to_lvl2(self, lvl2_input: Callable[[List[JState]], Any]):
        r"""Starts the subscriber for lvl2 (IK commands), transmitting incomming messages onto ``lvl2_input``.

        This function will be called ONCE for you, providing you the lvl2_input interface function of the joint core. It is your job to handle this interface function as you see fit.

        Tipical steps:

            - Subscribe to a topic.
            - In the subscription callback:

                - Convert the incomming messages to a List[JState].
                - Call ``lvl2_input`` using your processed messages.

        .. Note::

            \ ``lvl2_input`` is typically :py:meth:`.JointCore.coming_from_lvl2`

        Args:
            lvl0_input: Interface function of the joint core to call in a callback using the processed message data.
        """
        ...

    def _link_subscribers(self):
        self.subscribe_to_lvl0(lvl0_input=error_catcher(self.lvl1.coming_from_lvl0))
        self.subscribe_to_lvl2(lvl2_input=error_catcher(self.lvl1.coming_from_lvl2))

    @abstractmethod
    def publish_to_lvl0(self, states: List[JState]):
        r"""This method is called every time some states need to be sent to lvl0 (motor command).

        It is your job to process then send the state data as you see fit.

        Tipical steps:

            - Make a publisher in the __init__.
            - Process ``state`` in a message.
            - call publisher.publish with your message

        .. Note::

            \ This function will typically be executed by :py:meth:`.JointCore.send_to_lvl0`

        Args:
            states: Joint states to be sent.
        """
        ...

    @abstractmethod
    def publish_to_lvl2(self, states: List[JState]):
        r"""This method is called every time some states need to be sent to lvl2 (IK state).

        It is your job to process then send the state data as you see fit.

        Tipical steps:

            - Make a publisher in the __init__.
            - Process ``state`` in a message.
            - call publisher.publish with your message

        .. Note::

            \ This function will typically be executed by :py:meth:`.JointCore.send_to_lvl2`

        Args:
            states: Joint states to be sent.
        """
        ...

    def _link_publishers(self):
        self.lvl1.send_to_lvl0_callbacks.append(self.publish_to_lvl0)
        self.lvl1.send_to_lvl2_callbacks.append(self.publish_to_lvl2)

    @abstractmethod
    def frequently_send_to_lvl2(self, send_function: Callable[[], None]):
        """Starts executing ``send_function`` regularly.

        Fresh sensor states must be send regularly to lvl2 (IK) using send_function.
        It is you job to call ``send_function`` regularly.

        Tipical steps:

            - Make a timer.
            - Call send_function in the timer.

        .. Note::

            \ ``send_function`` is typically :py:meth:`.JointCore.send_sensor_up`

        Args:
            send_function: Function sending fresh sensor states to lvl2
        """
        ...

    def _link_timers(self):
        self.frequently_send_to_lvl2(self.lvl1.send_sensor_up)

    @abstractmethod
    def startup_action(self, lvl1: JointCore):
        """This will be executed once when the node starts.

        You can keep this empty, but typically, a message with only joint names and not data is sent to initialise lvl0 (if using Rviz this step is pretty much necessary).
        """
        pass

    def _on_startup(self):
        link_startup_action(self, self.startup_action, self.lvl1)

    def spin(self):
        """Spins the node"""
        my_main(self, multi_threaded=False)


class Lvl1Default(Lvl1Node):
    """Default implementation of the Joint node of lvl1.

    **Publishers:**

    ==============  ==========  ====
    Topic           Type        Note
    ==============  ==========  ====
    joint_commands  JointState  sent to motors (lvl0)
    joint_read      JointState  sent to IK (lvl2)
    ==============  ==========  ====

    **Subscribers:**

    ==============  ==========  ====
    Topic           Type        Note
    ==============  ==========  ====
    joint_states    JointState  coming from sensors (lvl0)
    joint_set       JointState  coming from IK (lvl2)
    ==============  ==========  ====

    Timers:
        - Sends to lvl2, freq.= ROS2_PARAMETER[``mvmt_update_rate``].

    Startup:
        - Sends empty message to lvl0 with only joint names.
    """

    def __init__(self):
        super().__init__()
        raw_publisher: Callable[[JointState], None] = CallablePublisher(
            node=self,
            topic_type=lvl1.output.motor_command.type,
            topic_name=lvl1.output.motor_command.name,
        )
        self.to_lvl0 = js_utils.JSCallableWrapper(raw_publisher)
        raw_publisher: Callable[[JointState], None] = CallablePublisher(
            node=self,
            topic_type=lvl1.output.ik_command.type,
            topic_name=lvl1.output.ik_command.name,
        )
        self.to_lvl2 = js_utils.JSCallableWrapper(raw_publisher)
        create_advertise_service(self, self.lvl1)

    def subscribe_to_lvl2(self, lvl2_input: Callable[[List[JState]], Any]):
        """"""
        self.create_subscription(
            lvl1.input.ik_command.type, lvl1.input.ik_command.name, lvl2_input, 10
        )

    def subscribe_to_lvl0(self, lvl0_input: Callable[[List[JState]], Any]):
        """"""
        self.create_subscription(
            lvl1.input.motor_sensor.type, lvl1.input.motor_sensor.name, lvl0_input, 10
        )

    def publish_to_lvl0(self, states: List[JState]):
        """"""
        self.to_lvl0(states)

    def publish_to_lvl2(self, states: List[JState]):
        """"""
        self.to_lvl2(states)

    def frequently_send_to_lvl2(self, send_function: Callable[[], None]):
        """"""
        self.create_timer(1 / self.lvl1.ms_param["mvmt_update_rate"], send_function)

    def startup_action(self, lvl1: JointCore):
        """"""
        lvl1.send_to_lvl0(
            [JState(time=lvl1.now(), name=n) for n in lvl1.jointHandlerDic.keys()]
        )
