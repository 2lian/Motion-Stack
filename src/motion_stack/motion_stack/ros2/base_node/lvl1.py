"""Template for a ROS2 node running the python core of lvl1."""

from abc import ABC, abstractmethod
from typing import Any, Callable, List

import rclpy
from rclpy.node import Node
from rclpy.task import Future

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
    core: core_class  #: Instance of the python core.

    def __init__(self):
        super().__init__("lvl1")
        self._spinner = Ros2Spinner(self)
        self.core = self.core_class(self._spinner)
        # self._spinner.wait_for_lower_level()
        self._link_publishers()
        self._link_subscribers()
        self._link_timers()
        self._link_sensor_check()
        self._on_startup()

    @abstractmethod
    def subscribe_to_lvl0(self, lvl0_input: Callable[[List[JState]], Any]):
        r"""Starts transmitting incomming **sensor data** to the python core.

        \ ``lvl0_input`` is a function that must be called when new sensor data is available. The data type must be a list of JState.

        Tipical steps:

            - Subscribe to a topic.
            - In the subscription callback:

                - Convert the incomming messages to a List[JState].
                - Call ``lvl0_input`` using your processed messages.

        .. Important::

            This function is called **once** at startup to setup some kind of continuous process. This continuous process must then call ``lvl0_input``.

        .. Note::

            \ ``lvl0_input`` is typically :py:meth:`.JointCore.coming_from_lvl0`

        Args:
            lvl0_input: Interface function of the joint core, to call (in a callback) when new sensor data is available.
        """
        ...

    @abstractmethod
    def subscribe_to_lvl2(self, lvl2_input: Callable[[List[JState]], Any]):
        r"""Starts transmitting incomming **joint targets** to the python core.

        \ ``lvl2_input`` is a function that must be called when new joint targets (typically resulting from IK) is available. The data type must be a list of JState.


        Tipical steps:

            - Subscribe to a topic.
            - In the subscription callback:

                - Convert the incomming messages to a List[JState].
                - Call ``lvl2_input`` using your processed messages.

        .. Important::

            This function is called **once** at startup to setup some kind of continuous process. This continuous process must then call ``lvl2_input``.

        .. Note::

            \ ``lvl2_input`` is typically :py:meth:`.JointCore.coming_from_lvl2`

        Args:
            lvl0_input: Interface function of the joint core, to call (in a callback) when new joint targets are available.
        """
        ...

    def _link_subscribers(self):
        @error_catcher
        def from_lvl0(states: List[JState]) -> None:
            self.core.coming_from_lvl0(states)
            self.core.send_sensor_up()

        @error_catcher
        def from_lvl2(states: List[JState]) -> None:
            self.core.coming_from_lvl2(states)
            self.core.send_command_down()

        self.subscribe_to_lvl0(lvl0_input=from_lvl0)
        self.subscribe_to_lvl2(lvl2_input=from_lvl2)

    @abstractmethod
    def publish_to_lvl0(self, states: List[JState]):
        r"""This method is called every time some **motor commands** need to be sent to lvl0.

        ``states`` should be processed then sent onto the next step (published by ROS2).

        Tipical steps:

            - Make a publisher in the __init__.
            - Process ``state`` in a message.
            - call publisher.publish with your message

        .. Note::

            \ This method will typically be called by :py:meth:`.JointCore.send_to_lvl0`

        Args:
            states: Joint states to be sent.
        """
        ...

    @abstractmethod
    def publish_to_lvl2(self, states: List[JState]):
        r"""This method is called every time some **joint states** need to be sent to lvl2.

        ``states`` should be processed then sent onto the next step (published by ROS2).

        Tipical steps:

            - Make a publisher in the __init__.
            - Process ``state`` in a message.
            - call publisher.publish with your message

        .. Note::

            \ This method will typically be called by :py:meth:`.JointCore.send_to_lvl2`

        Args:
            states: Joint states to be sent.
        """
        ...

    def _link_publishers(self):
        self.core.send_to_lvl0_callbacks.append(self.publish_to_lvl0)
        self.core.send_to_lvl2_callbacks.append(self.publish_to_lvl2)

    @abstractmethod
    def frequently_send_to_lvl2(self, send_function: Callable[[], None]):
        r"""Starts executing ``send_function`` regularly.

        Fresh sensor states must be send regularly to lvl2 (IK) using send_function. When using speed mode, it is also necessary to regularly send speed.

        Tipical steps:

            - Make a timer.
            - Call send_function in the timer.

        .. Note::

            \ ``send_function`` is typically :py:meth:`.JointCore.send_sensor_up` and  :py:meth:`.JointCore.send_command_down`

        Args:
            send_function: Function sending fresh sensor states to lvl2
        """
        ...

    def _link_timers(self):
        def execute():
            self.core.send_sensor_up()
            self.core.send_command_down()

        self.frequently_send_to_lvl2(error_catcher(execute))

    def _link_sensor_check(self):
        fut = Future()

        @error_catcher
        def execute():
            all_done = self.core.sensor_check_verbose()
            if all_done:
                fut.set_result(True)

        tmr = self.create_timer(1 / 3, execute)
        fut.add_done_callback(lambda *_: self.destroy_timer(tmr))

    @abstractmethod
    def startup_action(self, core: JointCore):
        """This will be executed *once* during the first ros spin of the node.

        You can keep this empty, but typically:

            - a message with only joint names and no data is sent to initialise lvl0 (if using Rviz this step is pretty much necessary).
            - "alive" services are started to signal that the node is ready.
        """
        ...

    def _on_startup(self):
        link_startup_action(self, self.startup_action, self.core)

    @classmethod
    def spin(cls):
        """Spins the node"""
        my_main(cls, multi_threaded=False)
