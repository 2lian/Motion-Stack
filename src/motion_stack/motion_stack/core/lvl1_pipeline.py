import logging
from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Callable, Dict, Final, List, Optional, OrderedDict, Tuple

from colorama import Fore, Style

from motion_stack.api.injection.remapper import StateRemapper
from motion_stack.core.utils import joint_mapper, static_executor
from motion_stack.core.utils.joint_state import (
    BatchedAsyncioBuffer,
    JState,
    JStateBuffer,
    MultiJState,
    multi_to_js_dict,
)
from motion_stack.core.utils.printing import list_cyanize
from motion_stack.core.utils.time import Time

from .utils.static_executor import default_param_dict


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
    batch_time: float = 0.001
    slow_pub_time: float = 0.5


lvl1_default: Final[Lvl1Param] = Lvl1Param(
    urdf=default_param_dict["urdf"][0],
    namespace=f"ms/{default_param_dict['leg_number']}"[0],
    end_effector_name=default_param_dict["end_effector_name"][0],
    start_effector_name=default_param_dict["start_effector_name"][0],
    mvmt_update_rate=default_param_dict["mvmt_update_rate"][0],
    joint_buffer=JState(
        name="",
        time=Time(static_executor.default_param_dict["joint_buffer"][0][0]),
        position=(static_executor.default_param_dict["joint_buffer"][0][1]),
        velocity=(static_executor.default_param_dict["joint_buffer"][0][2]),
        effort=(static_executor.default_param_dict["joint_buffer"][0][3]),
    ),
    add_joint=default_param_dict["add_joints"][0],
    ignore_limits=default_param_dict["ignore_limits"][0],
    limit_margin=default_param_dict["limit_margin"][0],
)


logger = logging.getLogger(__name__)


class JointPipeline:
    def __init__(
        self,
        buffer_delta: JState,
        callback: Callable[[Dict[str, JState]], Any],
        batch_time: Optional[float] = None,
    ) -> None:
        self.callback = callback
        self._sensor_buf: JStateBuffer = JStateBuffer(buffer_delta)
        self._batched_buf = BatchedAsyncioBuffer(
            self._sensor_buf, self._output, batch_time=batch_time
        )
        self.pre_maps: List[Callable[[Dict[str, JState]], Dict[str, JState]]] = []
        self.post_maps: List[Callable[[Dict[str, JState]], Dict[str, JState]]] = []

    @property
    def state(self) -> Dict[str, JState]:
        return self._sensor_buf.accumulated

    def input(self, states: MultiJState):
        js = multi_to_js_dict(states)
        js = self._pre_process(js)
        self._batched_buf.input(js)

    def _output(self, states: Dict[str, JState]):
        out = self._post_process(states)
        self.callback(out)

    def _post_process(self, states: Dict[str, JState]) -> Dict[str, JState]:
        js_copy = deepcopy(states)
        for map_func in self.post_maps:
            js_copy = map_func(js_copy)
        return js_copy

    def _pre_process(self, states: Dict[str, JState]) -> Dict[str, JState]:
        js_copy = deepcopy(states)
        for map_func in self.pre_maps:
            js_copy = map_func(js_copy)
        return js_copy


class Lvl1Pipeline:
    def __init__(self, params: Lvl1Param = lvl1_default) -> None:
        self.params: Lvl1Param = params
        self.sensor_pipeline = JointPipeline(
            params.joint_buffer, self.send_to_lvl2, batch_time=params.batch_time
        )
        self.inject_logs_for_newjoint()
        self.command_pipeline = JointPipeline(
            params.joint_buffer, self.send_to_lvl0, batch_time=params.batch_time
        )
        self.lvl0_remap = StateRemapper()  # empty
        self.lvl2_remap = StateRemapper()  # empty
        self.send_to_lvl0_callbacks: List[Callable[[Dict[str, JState]], Any]] = []
        self.send_to_lvl2_callbacks: List[Callable[[Dict[str, JState]], Any]] = []
        self.necessary_joints = set(params.add_joint)
        self.create_pipelines()

    def create_pipelines(self):
        self.sensor_pipeline.pre_maps = self.sensor_preprocess()
        self.sensor_pipeline.post_maps = self.sensor_postprocess()
        self.command_pipeline.pre_maps = self.command_preprocess()
        self.command_pipeline.post_maps = self.command_postprocess()

    def command_postprocess(
        self,
    ) -> List[Callable[[Dict[str, JState]], Dict[str, JState]]]:
        # todo joints limits
        def mapping(states: Dict[str, JState]) -> Dict[str, JState]:
            return self.lvl0_remap.map(states)

        return [mapping]

    def command_preprocess(
        self,
    ) -> List[Callable[[Dict[str, JState]], Dict[str, JState]]]:
        def mapping(states: Dict[str, JState]) -> Dict[str, JState]:
            return self.lvl2_remap.unmap(states)

        return [mapping]

    def sensor_postprocess(
        self,
    ) -> List[Callable[[Dict[str, JState]], Dict[str, JState]]]:
        def mapping(states: Dict[str, JState]) -> Dict[str, JState]:
            return self.lvl2_remap.map(states)

        return [mapping]

    def sensor_preprocess(
        self,
    ) -> List[Callable[[Dict[str, JState]], Dict[str, JState]]]:
        def mapping(states: Dict[str, JState]) -> Dict[str, JState]:
            return self.lvl0_remap.unmap(states)

        return [mapping]

    def inject_logs_for_newjoint(self):
        displayed = set()
        joints_happy = False
        original = self.sensor_pipeline._sensor_buf.push

        def injected(states):
            nonlocal joints_happy
            available = set(self.sensor_pipeline.state.keys())
            new = displayed - available
            if len(new) == 0:
                return original(states)
            logger.info(
                f"Joint data: {Fore.BLUE}Ready{Fore.RESET} for {list_cyanize(new)}{Fore.RESET}"
            )
            if joints_happy:
                return original(states)
            missing = available - self.necessary_joints
            if missing == set():
                logger.info(
                    f"Joint data: {Style.BRIGHT}{Fore.GREEN}FULLY READY :){Fore.RESET}"
                )
                joints_happy = True
            return original(states)

        self.sensor_pipeline._sensor_buf.push = injected

    def display_missing_joints(self):
        available = set(self.sensor_pipeline.state.keys())
        missing = available - self.necessary_joints
        found = available & self.necessary_joints
        if found == set():
            logger.warning(f"Joint data: Missing ALL : %s. ", missing)
        elif missing:
            logger.warning(f"Joint data: Missing for %s. ", missing)

    def send_to_lvl0(self, states: Dict[str, JState]):
        """Sends states to lvl0 (commands for motors).
        This function is executed every time data needs to be sent down.

        .. Important::

            Change/overide this method with what you need.

            Or put what you want to execute in self.send_to_lvl0_callbacks
        """
        for f in self.send_to_lvl0_callbacks:
            f(states)

    def send_to_lvl2(self, states: Dict[str, JState]):
        """Sends states to lvl2 (commands for motors).
        This function is executed every time data needs to be sent down.

        .. Important::

            Change/overide this method with what you need.

            Or put what you want to execute in self.send_to_lvl2_callbacks
        """
        for f in self.send_to_lvl2_callbacks:
            f(states)

    def coming_from_lvl0(self, states: MultiJState):
        """Processes incomming sensor states from lvl0 motors.
        Call this function after processing the data into a List[JState]

        .. Caution::

            Overloading this is not advised, but if you do, always do
            super().coming_from_lvl0(states) before your code.
            Unless you know what you are doing
        """
        self.sensor_pipeline.input(states)

    def coming_from_lvl2(self, states: MultiJState):
        """Processes incomming commands from lvl2 ik.
        Call this function after processing the data into a List[JState]

        .. Caution::

            Overloading this is not advised, but if you do, always do
            super().coming_from_lvl0(states) before your code.
            Unless you know what you are doing
        """
        self.command_pipeline.input(states)
