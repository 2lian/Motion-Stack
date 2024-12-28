from abc import ABC, abstractmethod
from time import time_ns
from typing import Any, Dict, List, Optional

from .printing import TCOL
from .time import Time

default_param = [
    ("leg_number", int, 0),
    ("std_movement_time", float, 0.5),
    ("control_rate", float, 30),
    ("mvmt_update_rate", float, 10),
    ("ignore_limits", bool, False),
    ("limit_margin", float, 0),
    ("speed_mode", bool, False),
    ("add_joints", List[str], [""]),
    ("ignore_limits", bool, False),
    ("start_coord", List[float], [0, 0, 0]),
    # ("mirror_angles", bool, False),
    ("urdf_path", str, ""),
    ("start_effector_name", str, ""),
    ("end_effector_name", str, ""),
]


class Spinner(ABC):
    @abstractmethod
    def now(self) -> Time: ...
    @abstractmethod
    def error(self) -> None: ...
    @abstractmethod
    def warn(self) -> None: ...
    @abstractmethod
    def info(self) -> None: ...
    @abstractmethod
    def debug(self) -> None: ...
    @abstractmethod
    def get_parameter(self, name: str, value_type, default=None): ...


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
            self.__ms_param = self.__make_ms_param()
        return self.__ms_param

    def __make_ms_param(self) -> Dict[str, Any]:
        params: Dict[str, Any] = {}
        for p, t, v in default_param:
            params[p] = self.spinner.get_parameter(p, t, v)
        return params
