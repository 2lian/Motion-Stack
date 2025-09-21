import asyncio
from abc import ABC, abstractmethod
from typing import Any, Callable, Dict, List, Union

from motion_stack.core.lvl1_joint import JointCore
from motion_stack.core.utils.joint_state import JState, JStateBuffer
from motion_stack.core.utils.static_executor import PythonSpinner
from motion_stack.zenoh.utils.communication import JointStatePub, JointStateSub


class Lvl1Node(ABC):
    """Abstract base class for ros2, to be completed by the user.

    To see the default behavior implemented using this template, refer to :py:class:`.ros2.default_node.lvl1`.
    """

    def __init__(self):
        self._link_publishers()
        self._link_subscribers()
        self._link_timers()
        self._link_sensor_check()
        self._on_startup()

        self.lvl0_pub = JointStatePub(key="TODO/lvl0/joint_command")
        self.lvl0_sub = JointStateSub(key="TODO/lvl0/joint_state")
        self.lvl2_pub = JointStatePub(key="TODO/lvl2/joint_read")
        self.lvl2_sub = JointStateSub(key="TODO/lvl2/joint_set")
        self.sensor_buf = JStateBuffer(JState(""))
        self.command_buf = JStateBuffer(JState(""))

        self.d_required = set()
        self.d_displayed = set()

    @abstractmethod
    def subscribe_to_lvl0(
        self, lvl0_input: Callable[[Union[Dict[str, JState], List[JState]]], Any]
    ):
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

        async def sensor_loop():
            nonlocal lvl0_input
            while 1:
                js = await self.lvl0_sub.listen()
                lvl0_input({js.name: js})

        asyncio.create_task(sensor_loop())

    @abstractmethod
    def subscribe_to_lvl2(
        self, lvl2_input: Callable[[Union[Dict[str, JState], List[JState]]], Any]
    ):
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

        async def sensor_loop():
            nonlocal lvl2_input
            while 1:
                js = await self.lvl2_sub.listen()
                lvl2_input({js.name: js})
                

        asyncio.create_task(sensor_loop())

    def _link_subscribers(self):

        def lvl0_input(js: Dict[str, JState]):
                b = self.sensor_buf
                b.push(js)
                is_urgent = b._find_urgent(b.last_sent, js, b.delta)
                if is_urgent:
                    asyncio.get_event_loop().call_later(
                        0.001, lambda **_: lvl2_output(b.pull_urgent())
                    )

        self.subscribe_to_lvl0(lvl0_input)

        def lvl2_input(js: Dict[str, JState]):
                b = self.command_buf
                b.push(js)
                is_urgent = b._find_urgent(b.last_sent, js, b.delta)
                if is_urgent:
                    asyncio.get_event_loop().call_later(
                        0.001, lambda **_: lvl0_output(b.pull_urgent())
                    )

        self.subscribe_to_lvl2(lvl2_input)

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
        self.lvl0_pub.publish(states)

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
        self.lvl2_pub.publish(states)

    def _link_publishers(self): ...

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

    def _link_sensor_check(self):

        async def happy_new_state():
            while 1:
                b = self.sensor_buf
                available = set(b.accumulated.keys())
                new = self.d_displayed - available
                print(f"YEY new: {new}")
                await asyncio.sleep(1 / 3)

        asyncio.create_task(happy_new_state())

        def angry_new_state():
            b = self.sensor_buf
            available = set(b.accumulated.keys())
            missing = self.d_required - available
            print(f"OH NYO missing: {missing}")

        asyncio.get_event_loop().call_later(3, angry_new_state)

    @abstractmethod
    def startup_action(self):
        """This will be executed *once* during the first ros spin of the node.

        You can keep this empty, but typically:

            - a message with only joint names and no data is sent to initialise lvl0 (if using Rviz this step is pretty much necessary).
            - "alive" services are started to signal that the node is ready.
        """
        ...

    def _on_startup(self):
        asyncio.get_event_loop().call_soon(self.startup_action)
