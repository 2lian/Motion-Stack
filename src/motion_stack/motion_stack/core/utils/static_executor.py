from abc import ABC, abstractmethod
from typing import Any, Dict, List, Tuple, get_args, get_origin

from .printing import TCOL
from .time import Time


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


default_param: List[Tuple[str, type, Any]] = [
    ("leg_number", int, 0),
    ("std_movement_time", float, 0.5),
    ("control_rate", float, 30.0),
    ("mvmt_update_rate", float, 10.0),
    ("ignore_limits", bool, False),
    ("limit_margin", float, 0.0),
    ("speed_mode", bool, False),
    ("add_joints", List[str], [""]),
    ("start_coord", List[float], [0.0, 0.0, 0.0]),
    # ("mirror_angles", bool, False),
    ("urdf_path", str, ""),
    ("urdf", str, ""),
    ("start_effector_name", str, ""),
    ("end_effector_name", str, ""),
]


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
