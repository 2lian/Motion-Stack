"""Defines an executor to be replaced by ros2 or other one.

For now the executor have minimal responsibility, limited to time and launch parameters.
"""

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Tuple, get_args, get_origin

import numpy as np

from .printing import TCOL
from .time import Time

r"""
The parameters of the motion stack in python form, with their type and default value.

.. code-block:: python
   :linenos:

    default_param: List[Tuple[str, type, Any]] = [
        ("urdf", str, ""),
        #: raw urdf string
        ("leg_number", int, 0),
        # number associated with a leg, similar to a workspace
        ("end_effector_name", str, ""),
        # end effector associated with a leg, the kinematic chain used for IK will
        # go from the root link of the URDF (usually base_link) to the end effector
        # link (specified in this parameter). the URDF will be parsed to find this
        # link name. Make sure it exists. you can also provide a number (as a string)
        # instead of a link_name. If you do this the Nth longest kinematic path
        # (sequence of link where each link is connected to exactly one other link)
        # from the root of the URDF will be used for IK Basically, if you use only
        # one limb, set this as "0", and it will pick the right ee.
        ("start_effector_name", str, ""),
        # setting this manually, works with the motion stack, but not for Rviz and
        # ros2's tf, so be carefull. In ros, the baselink must be the root of the
        # tf tree, it cannot have a parent and there can only be one baselink.
        # Leaving this empty and properly setting your URDF baselink is
        # recommended.
        ("mvmt_update_rate", float, 10.0),
        # update rate used through out the stack
        (
            "joint_buffer",
            List[float],
            [
                0.5, # time: seconds
                np.deg2rad(0.05), # position: rad
                np.deg2rad(0.01), # velocity: rad/s
                np.deg2rad(0.001), # effort: N.m
            ],
        ),
        # lvl1 incorporates a joint state buffer to not repeat states that are
        # identical. If the difference between current and previously sent state
        # exceed any of those values, a state message is sent.
        ("angle_syncer_delta", float, np.deg2rad(20)),
        # The difference command-sensor cannot be further than this delta.
        # Small values slow down execution but are safer.
        # Using zero disables this feature.
        ("add_joints", List[str], [""]),
        # manually adds joints for lvl1 if they are not in the urdf
        ("ignore_limits", bool, False),
        # joint limits set in the URDF will be ignored
        ("limit_margin", float, 0.0),
        # adds a additional margin to the limits of the URDF (in rad)
        ("speed_mode", bool, False),
        # lvl1 will send speed commands to the motors, using angle readings as
        # feedback for a PID.
        ("control_rate", float, 30.0),  # update rate for speed mode PID only
    ]
"""
default_param: List[Tuple[str, type, Any]] = [
    ("urdf", str, ""),
    #: raw urdf string
    ("leg_number", int, 0),
    # number associated with a leg, similar to a workspace
    ("end_effector_name", str, ""),
    # end effector associated with a leg, the kinematic chain used for IK will
    # go from the root link of the URDF (usually base_link) to the end effector
    # link (specified in this parameter). the URDF will be parsed to find this
    # link name. Make sure it exists. you can also provide a number (as a string)
    # instead of a link_name. If you do this the Nth longest kinematic path
    # (sequence of link where each link is connected to exactly one other link)
    # from the root of the URDF will be used for IK Basically, if you use only
    # one limb, set this as "0", and it will pick the right ee.
    ("start_effector_name", str, ""),
    # setting this manually, works with the motion stack, but not for Rviz and
    # ros2's tf, so be carefull. In ros, the baselink must be the root of the
    # tf tree, it cannot have a parent and there can only be one baselink.
    # Leaving this empty and properly setting your URDF baselink is
    # recommended.
    ("mvmt_update_rate", float, 10.0),
    # update rate used through out the stack
    (
        "joint_buffer",
        List[float],
        [
            0.5, # time: seconds
            np.deg2rad(0.05), # position: rad
            np.deg2rad(0.01), # velocity: rad/s
            np.deg2rad(0.001), # effort: N.m
        ],
    ),
    # lvl1 incorporates a joint state buffer to not repeat states that are
    # identical. If the difference between current and previously sent state
    # exceed any of those values, a state message is sent.
    ("angle_syncer_delta", float, np.deg2rad(20)),
    # The difference command-sensor cannot be further than this delta.
    # Small values slow down execution but are safer.
    # Using zero disables this feature.
    ("add_joints", List[str], [""]),
    # manually adds joints for lvl1 if they are not in the urdf
    ("ignore_limits", bool, False),
    # joint limits set in the URDF will be ignored
    ("limit_margin", float, 0.0),
    # adds a additional margin to the limits of the URDF (in rad)
    ("speed_mode", bool, False),
    # lvl1 will send speed commands to the motors, using angle readings as
    # feedback for a PID.
    ("control_rate", float, 30.0),  # update rate for speed mode PID only
]


def extract_inner_type(list_type: type):
    """
    Extracts the inner type from a typing.List, such as List[float].

    :param list_type: A type hint, e.g., List[float].
    :return: The inner type of the list, e.g., float.
    """
    if get_origin(list_type) is list or get_origin(list_type) is List:
        inner_types = get_args(list_type)  # Get the arguments of the generic type
        if len(inner_types) == 1:  # Ensure there's one type argument
            return inner_types[0]
    raise ValueError(f"Provided type '{list_type}' is not a valid List type.")


class Spinner(ABC):
    alias: str = ""

    @abstractmethod
    def now(self) -> Time: ...
    @abstractmethod
    def error(self, *args, **kwargs) -> None: ...
    @abstractmethod
    def warn(self, *args, **kwargs) -> None: ...
    @abstractmethod
    def info(self, *args, **kwargs) -> None: ...
    @abstractmethod
    def debug(self, *args, **kwargs) -> None: ...
    @abstractmethod
    def get_parameter(self, name: str, value_type: type, default=None) -> Any: ...


class PythonSpinner(Spinner):
    def __init__(self) -> None:
        self.__now: Time = Time(0)

    def now(self) -> Time:
        return self.__now

    def change_time(self, time: Time):
        self.__now = Time(time)

    def error(self, *args) -> None:
        print(TCOL.FAIL + str(*args) + TCOL.ENDC)

    def warn(self, *args) -> None:
        print(TCOL.WARNING + str(*args) + TCOL.ENDC)

    def info(self, *args) -> None:
        print(*args)

    def debug(self, *args) -> None:
        print(TCOL.OKBLUE + str(*args) + TCOL.ENDC)


class FlexNode:
    spinner: Spinner  #: must be initialized and spinning already
    startup_time: Time

    def __init__(self, spinner: Spinner):
        self.spinner = spinner
        self.startup_time = self.now()
        self.__ms_param = None

    def now(self) -> Time:
        return self.spinner.now()

    def error(self, *args):
        self.spinner.error(*args)

    def warn(self, *args):
        self.spinner.warn(*args)

    def info(self, *args):
        self.spinner.info(*args)

    def debug(self, *args):
        self.spinner.debug(*args)

    @property
    def ms_param(self) -> Dict[str, Any]:
        if self.__ms_param is None:
            self.__ms_param = self._make_ms_param()
        return self.__ms_param

    def _make_ms_param(self) -> Dict[str, Any]:
        params: Dict[str, Any] = {}
        for p, t, v in default_param:
            if p in params.keys():
                self.error(f"Parameter {p} defined twice. Second one ignored.")
            params[p] = self.spinner.get_parameter(p, t, v)
        return params
