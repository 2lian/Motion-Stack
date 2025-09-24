from dataclasses import dataclass
from typing import Any, Callable, Dict, List, OrderedDict, Tuple

from motion_stack.core.utils.joint_state import JState, JStateBuffer


@dataclass
class Lvl1Param:
    urdf: str
    namespace: str
    end_effector_name: str
    start_effector_name: str
    mvmt_update_rate: float
    joint_buffer: JState
    add_joint: List[str]
    ignore_limits: bool
    limit_margin: float


RecCallable = Callable[[Dict[str, JState], "RecCallable"]]


class JointPipeline:
    def __init__(self, params: Lvl1Param) -> None:
        self.pipeline_up: List[Tuple[str, RecCallable]] = []
        self._sensor_buf: JStateBuffer = JStateBuffer(params.joint_buffer)

    @property
    def state(self) -> Dict[str, JState]:
        return self._sensor_buf.accumulated
