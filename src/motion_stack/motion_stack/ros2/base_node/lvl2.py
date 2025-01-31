"""Template for a ROS2 node running the python core of lvl2."""

from abc import ABC, abstractmethod
from typing import Any, Callable, List

import rclpy
from rclpy.node import Node

from motion_stack.core.lvl2_ik import IKCore
from motion_stack.core.utils.pose import Pose

from ...core.utils.joint_state import JState
from ..utils.executor import Ros2Spinner, error_catcher, my_main
from ..utils.linking import link_startup_action


class Lvl2Node(Node, ABC):
    """Abstract base class for ros2, to be completed by the user.

    To see the default behavior implemented using this template, refer to :py:class:`.ros2.default_node.lvl1.DefaultLvl2`.
    """

    _spinner: Ros2Spinner
    core_class = IKCore  #: Class from which the core is instantiated. Overwrite this with a modified core to change behavior not related to ROS2.
    core: core_class  #: Instance of the python core.

    def __init__(self):
        super().__init__("lvl2")
        self._spinner = Ros2Spinner(self)
        self.core = self.core_class(self._spinner)
        self._spinner.wait_for_lower_level()
        self._link_publishers()
        self._link_subscribers()
        self._on_startup()

    @abstractmethod
    def subscribe_to_lvl1(self, lvl1_input: Callable[[List[JState]], Any]):
        r"""Starts transmitting incomming **state data** to the python core.

        \ ``lvl1_input`` is a function that must be called when new state data is available. The data type must be a list of JState.

        Tipical steps:

            - Subscribe to a topic.
            - In the subscription callback:

                - Convert the incomming messages to a List[JState].
                - Call ``lvl1_input`` using your processed messages.

        .. Important::

            This function is called **once** at startup to setup some kind of continuous process. This continuous process must then call ``lvl0_input``.

        .. Note::

            \ ``lvl1_input`` is typically :py:meth:`.IKCore.state_from_lvl1`

        Args:
            lvl1_input: Interface function of the ik core, to call (in a callback) when new state data is available.
        """
        ...

    @abstractmethod
    def subscribe_to_lvl3(self, lvl3_input: Callable[[Pose], Any]):
        r"""Starts transmitting incomming **ik targets** to the python core.

        \ ``lvl3_input`` is a function that must be called when new ik target is available. The data type must be a Pose.


        Tipical steps:

            - Subscribe to a topic.
            - In the subscription callback:

                - Convert the incomming messages to a Pose.
                - Call ``lvl3_input`` with the processed messages.

        .. Important::

            This function is called **once** at startup to setup some kind of continuous process. This continuous process must then call ``lvl3_input``.

        .. Note::

            \ ``lvl3_input`` is typically :py:meth:`.IKCore.ik_target`

        Args:
            lvl3_input: Interface function of the joint core, to call (in a callback) when new ik target is available.
        """
        ...

    def _link_subscribers(self):
        self.subscribe_to_lvl1(lvl1_input=error_catcher(self.core.state_from_lvl1))
        self.subscribe_to_lvl3(lvl3_input=error_catcher(self.core.ik_target))

    @abstractmethod
    def publish_to_lvl1(self, states: List[JState]):
        r"""This method is called every time some **joint targets** need to be sent to lvl1.

        ``states`` should be processed then sent onto the next step (published by ROS2).

        Tipical steps:

            - Make a publisher in the __init__.
            - Process ``state`` into a message.
            - call publisher.publish with your message

        .. Note::

            \ This method will typically be called by :py:meth:`.IKCore.send_to_lvl1`

        Args:
            states: Joint states to be sent.
        """
        ...

    @abstractmethod
    def publish_to_lvl3(self, pose: Pose):
        r"""This method is called every time some **end effector pose** needs to be sent to lvl3.

        ``pose`` should be processed then sent onto the next step (published by ROS2).

        Tipical steps:

            - Make a publisher in the __init__.
            - Process ``pose`` into a message.
            - call publisher.publish with your message

        .. Note::

            \ This method will typically be called by :py:meth:`.IKCore.send_to_lvl3`

        Args:
            states: Joint states to be sent.
        """
        ...

    def _link_publishers(self):
        self.core.send_to_lvl1_callbacks.append(self.publish_to_lvl1)
        self.core.send_to_lvl3_callbacks.append(self.publish_to_lvl3)

    @abstractmethod
    def startup_action(self, lvl2: IKCore):
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
