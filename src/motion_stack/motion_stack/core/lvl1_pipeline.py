from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, OrderedDict, Tuple

from motion_stack.core.utils import joint_mapper
from motion_stack.core.utils.joint_state import (
    BatchedAsyncioBuffer,
    JState,
    JStateBuffer,
    MultiJState,
    multi_to_js_dict,
)


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


class JointPipeline:
    def __init__(self, params: Lvl1Param, callback: Callable[[Dict[str, JState]]]) -> None:
        self.callback = callback
        self._sensor_buf: JStateBuffer = JStateBuffer(params.joint_buffer)
        self._batched_buf = BatchedAsyncioBuffer(self._sensor_buf, self.output)
        self.pre_maps: List[Callable[[Dict[str, JState]]]] = []
        self.post_maps: List[Callable[[Dict[str, JState]]]] = []

    @property
    def state(self) -> Dict[str, JState]:
        return self._sensor_buf.accumulated

    def input(self, states: MultiJState):
        js = multi_to_js_dict(states)
        js = self.pre_process(js)
        self._batched_buf.input(js)

    def output(self, states: Dict[str, JState]):
        out = self.post_process(states)
        self.callback(out)

    def post_process(self, states: Dict[str, JState]) -> Dict[str, JState]:
        js_copy = deepcopy(states)
        for map_func in self.post_maps:
            map_func(js_copy)
        return js_copy

    def pre_process(self, states: Dict[str, JState]) -> Dict[str, JState]:
        js_copy = deepcopy(states)
        for map_func in self.pre_maps:
            map_func(js_copy)
        return js_copy
