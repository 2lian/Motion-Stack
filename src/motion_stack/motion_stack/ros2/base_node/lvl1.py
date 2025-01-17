"""Template for a ROS2 node of Lvl1."""

from abc import ABC, abstractmethod
from typing import Any, Callable, List

import rclpy
from rclpy.node import Node

from ...core.lvl1_joint import JointCore
from ...core.utils.joint_state import JState
from ..utils.executor import Ros2Spinner, error_catcher, my_main
from ..utils.linking import link_startup_action


class Lvl1Node(Node, ABC):
    """Abstract base class for ros2, to be completed by the user.

    To see the default behavior implemented using this template, refer to :py:class:`.ros2.default_node.lvl1`.
    """

    _spinner: Ros2Spinner
    core_class = JointCore  #: Class from which the core is instantiated. Overwrite this with a modified core to change behavior not related to ROS2.
    lvl1: core_class  #: Instance of the python core.

    def __init__(self):
        super().__init__("lvl1")
        self._spinner = Ros2Spinner(self)
        self.lvl1 = self.core_class(self._spinner)
        self._spinner.wait_for_lower_level()
        self._link_publishers()
        self._link_subscribers()
        self._link_timers()
        self._link_sensor_check()
        self._on_startup()

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
        def execute():
            self.lvl1.send_sensor_up()
            self.lvl1.send_command_down()
        self.frequently_send_to_lvl2(error_catcher(execute))

    def _link_sensor_check(self):
        fut = rclpy.Future()

        @error_catcher
        def execute():
            all_done = self.lvl1.sensor_check_verbose()
            if all_done:
                fut.set_result(True)

        tmr = self.create_timer(1 / 10, execute)
        fut.add_done_callback(lambda *_: self.destroy_timer(tmr))

    @abstractmethod
    def startup_action(self, lvl1: JointCore):
        """This will be executed once when the node starts.

        You can keep this empty, but typically, a message with only joint names and not data is sent to initialise lvl0 (if using Rviz this step is pretty much necessary).
        """
        ...

    def _on_startup(self):
        link_startup_action(self, self.startup_action, self.lvl1)

    @classmethod
    def spin(cls):
        """Spins the node"""
        my_main(cls, multi_threaded=False)
