import asyncio
import dataclasses
import logging
from pprint import pprint
import time
from dataclasses import asdict, dataclass
from typing import (
    Any,
    Callable,
    Dict,
    Final,
    List,
    Optional,
    OrderedDict,
    Self,
    Set,
    Tuple,
    Union,
)

import asyncio_for_robotics as afor
import dacite
import numpy as np
import ujson
from asyncio_for_robotics.core.sub import BaseSub
from colorama import Fore, Style
from roboticstoolbox.tools.urdf.urdf import Joint as RTBJoint

from ..api.injection.remapper import StateRemapper
from .utils import joint_mapper, static_executor
from .utils.joint_state import JState, JStateBuffer, subdict
from .utils.printing import list_cyanize
from .utils.robot_parsing import get_limit, load_set_urdf_raw, make_ee
from .utils.static_executor import default_param_dict
from .utils.time import Time

JStateBatch = Dict[str, JState]


@dataclass
class Lvl1Param:
    urdf: str = ""
    namespace: str = "ms"
    end_effector_name: Union[None, str, int] = "ALL"
    start_effector_name: Union[None, str] = None
    mvmt_update_rate: float = 100
    joint_buffer: JState = dataclasses.field(
        default_factory=lambda *_: JState(
            name="",
            time=Time.sn(sec=0.5),
            position=np.deg2rad(0.05),
            velocity=np.deg2rad(0.01),
            effort=np.deg2rad(0.001),
        )
    )
    add_joint: List[str] = dataclasses.field(default_factory=lambda *_: [])
    ignore_limits: bool = False
    limit_margin: float = 0
    batch_time: float = 0.001
    slow_pub_time: float = 0.5
    continuous_read_hz: float = 24
    drop_ousiders: bool = True

    def to_json(self):
        return ujson.dumps(dataclasses.asdict(self), indent=2)

    @classmethod
    def from_json(cls, json_str: str) -> Self:
        j_dic = ujson.loads(json_str)
        return dacite.from_dict(cls, j_dic, dacite.Config())


lvl1_default: Final[Lvl1Param] = Lvl1Param()

@dataclass
class Lvl1Info:
    joints: List[str] = dataclasses.field(default_factory=lambda *_: [])
    namespace: str = "ms"
    end_effector_name: Union[None, str, int] = "ALL"
    start_effector_name: Union[None, str] = None
    mvmt_update_rate: float = 100
    joint_buffer: JState = dataclasses.field(
        default_factory=lambda *_: JState(
            name="",
            time=Time.sn(sec=0.5),
            position=np.deg2rad(0.05),
            velocity=np.deg2rad(0.01),
            effort=np.deg2rad(0.001),
        )
    )
    add_joint: List[str] = dataclasses.field(default_factory=lambda *_: [])
    ignore_limits: bool = False
    limit_margin: float = 0
    batch_time: float = 0.001
    slow_pub_time: float = 0.5
    continuous_read_hz: float = 24
    drop_ousiders: bool = True

    def to_json(self):
        return ujson.dumps(dataclasses.asdict(self))

    @classmethod
    def from_json(cls, json_str: str) -> Self:
        j_dic = ujson.loads(json_str)
        return dacite.from_dict(cls, j_dic, dacite.Config())


logger = logging.getLogger(__name__)


class JointPipeline:
    def __init__(
        self, input_sub: BaseSub[JStateBatch], buffer_delta: JState, batch_time: float
    ) -> None:
        """Asynchronous processing pipeline for joint state batches.

        The pipeline ingests raw joint state data, applies optional
        user-defined processing, and ouputs it on a afor sub.

        Data has two paths:
            - Fast. For data data that changed by more than buffer_delta. This
                  has a very small batch_time and is sent on the output
                  immediately.
            - Slow. For all new data.

        Args:
            input_sub: Subscription providing incoming raw joint state batches.
            buffer_param: State change delta that will trigger fast output.
            batch_time: Time window (in seconds) used to batch urgent data
                before flushing.
        """
        #: Subscription providing raw incoming joint state batches.
        self.input_sub: BaseSub[JStateBatch] = input_sub

        #: Emits the latest pre-processed joint state batches.
        #: This stream is unbuffered and may be updated at high frequency.
        self.internal_sub: BaseSub[JStateBatch] = BaseSub()

        #: Internal buffer tracking joint states and their scheduling class
        #: (e.g. accumulated, new, urgent) based on `buffer_delta`.
        self.internal_state: JStateBuffer = JStateBuffer(buffer_delta)

        #: Emits buffered batches that are ready for output processing,
        #: including both urgent (fast path) and periodic (slow path) flushes.
        self.buffered_sub: BaseSub[JStateBatch] = BaseSub()

        #: Final output stream exposed to external consumers after post-processing.
        self.output_sub: BaseSub[JStateBatch] = BaseSub()

        #: Rate at which to publish non-urgent states
        self.slow_rate: float = 2

        #: Event signaling the presence of urgent data that should be flushed
        self._flush_trigger: asyncio.Event = asyncio.Event()

        self._batch_time: float = batch_time
        self._queue_size = int(5e3)

    async def run(self):
        """Run the pipeline.

        This coroutine must be awaited to start all internal processing loops.
        The pipeline terminates if any task in the task group raises.
        """
        async with asyncio.TaskGroup() as tg:
            tg.create_task(self._input_loop(), name="pipeline_input")
            tg.create_task(self._flush_loop(), name="pipeline_flush")
            tg.create_task(self._slow_loop(), name="pipeline_slow")
            tg.create_task(self._output_loop(), name="pipeline_output")

    async def pre_process(self, jsb: JStateBatch) -> JStateBatch:
        """Hook for user-defined pre-processing of incoming data.

        This method is executed asynchronously and may reorder or delay data
        relative to arrival time.

        Args:
            jsb: Incoming joint state batch.

        Returns:
            Pre-processed joint state batch.
        """
        return jsb

    async def post_process(self, jsb: JStateBatch) -> JStateBatch:
        """Hook for user-defined post-processing before output publication.

        Args:
            jsb: Joint state batch after buffering and scheduling.

        Returns:
            Post-processed joint state batch.
        """
        return jsb

    async def _input_loop(self):
        """Subscribes to raw input data, schedule pre-processing, and trigger flushing.

        Incoming data is dispatched to parallel pre-processing tasks so that
        slow pre-processing does not block ingestion.
        """
        async with asyncio.TaskGroup() as tg:
            async for jsb in self.input_sub.listen_reliable(
                queue_size=self._queue_size
            ):
                # this is equivalent to calling _parallel_preproc here, but if the
                # pre_process takes some time for certain inputs, the other inputs
                # are not blocked. So the pre_process is capable of changing timing
                # and ordering.
                tg.create_task(self._parallel_preproc(jsb), name="pipeline_input_para")
                # await self._parallel_preproc(jsb)

    async def _parallel_preproc(self, jsb: JStateBatch):
        """Apply pre-processing and push results into the internal buffer.

        Urgent data triggers a batched flush via the flush event.
        """
        internal = await self.pre_process(jsb)
        self.internal_state.push(internal)
        self.internal_sub._input_data_asyncio(internal)
        if not self._flush_trigger.is_set():
            if len(self.internal_state.marked_urgent()) != 0:
                self._flush_trigger.set()

    async def _flush_loop(self):
        """Flush urgent buffered data after a batching delay.

        When triggered, waits `batch_time` seconds to accumulate urgent data
        before forwarding it to the buffered subscription.
        """
        while 1:
            await self._flush_trigger.wait()
            await asyncio.sleep(self._batch_time)
            if not self._flush_trigger.is_set():
                # something flushed during sleep (slow_loop?)
                continue
            self._flush_trigger.clear()
            to_send = self.internal_state.pull_urgent()
            if len(to_send) == 0:
                continue
            self.buffered_sub._input_data_asyncio(to_send)

    async def _slow_loop(self):
        """Periodically flush non-urgent buffered data."""
        async for t_ns in afor.Rate(frequency=self.slow_rate).listen():
            self._flush_trigger.clear()
            to_send = self.internal_state.pull_new()
            if len(to_send) == 0:
                continue
            self.buffered_sub._input_data_asyncio(to_send)

    async def _output_loop(self):
        """Apply post-processing and forward data to the output subscription.

        Post-processing is executed in parallel to avoid blocking the pipeline.
        """
        async with asyncio.TaskGroup() as tg:
            async for jsb in self.buffered_sub.listen_reliable(
                queue_size=self._queue_size
            ):
                tg.create_task(
                    self._parallel_postproc(jsb), name="pipeline_output_para"
                )
                # await self._parallel_postproc(jsb)

    async def _parallel_postproc(self, jsb: JStateBatch):
        """Apply post-processing to a single batch and publish it."""
        output = await self.post_process(jsb)
        self.output_sub._input_data_asyncio(output)


class JointCore:
    def __init__(
        self,
        sensor_sub: BaseSub[JStateBatch],
        command_sub: BaseSub[JStateBatch],
        params: Lvl1Param = lvl1_default,
    ) -> None:
        self.PARAMS: Lvl1Param = params
        self.sensor_sub: BaseSub[JStateBatch] = sensor_sub
        self.command_sub: BaseSub[JStateBatch] = command_sub
        self.continuous_js_output: BaseSub[JStateBatch] = BaseSub()
        self.sensor_pipeline: JointPipeline
        self.command_pipeline: JointPipeline
        self.joints_objects: List[RTBJoint]
        self.limits: Dict[str, Tuple[float, float]] = dict()  # needs improvement
        self.joints_objects, _, _, _ = self.setup_urdf()
        self.joints_of_interest: Set[str] = {k.name for k in self.joints_objects}

        self.lvl0_remap = StateRemapper()  # empty
        self.lvl2_remap = StateRemapper()  # empty
        self.create_sensor_pipelines()
        self.create_command_pipelines()

    def get_info(self) -> Lvl1Info:
        d = asdict(self.PARAMS)
        del d["urdf"]
        return Lvl1Info(joints=list(self.joints_of_interest), **d)

    @property
    def motor_output(self) -> BaseSub[JStateBatch]:
        return self.command_pipeline.output_sub

    @property
    def joint_read_output(self) -> BaseSub[JStateBatch]:
        return self.sensor_pipeline.output_sub

    async def run(self):
        async with asyncio.TaskGroup() as tg:
            tg.create_task(self.sensor_pipeline.run())
            tg.create_task(self.command_pipeline.run())
            tg.create_task(self.monitor_sensor())
            tg.create_task(self.send_empty_command())
            tg.create_task(self.continuous_joint_read())

    async def continuous_joint_read(self):
        async for t_ns in afor.Rate(frequency=self.PARAMS.continuous_read_hz).listen():
            self.continuous_js_output._input_data_asyncio(
                self.sensor_pipeline.internal_state.accumulated
            )

    async def send_empty_command(self):
        """Sends a command to motors with no data.

        Usefull to initialize lvl0 by giving only the joint names."""
        needed = {k.name for k in self.joints_objects}
        missing = lambda: needed - set(
            self.sensor_pipeline.internal_state.accumulated.keys()
        )
        while len(self.sensor_pipeline.internal_state.accumulated.keys()) <= 0:
            now = Time(nano=time.time_ns())
            js: JStateBatch = {j: JState(name=j, time=now) for j in missing()}
            js = self.lvl0_remap.map(js)
            self.command_pipeline.output_sub.input_data(js)
            await asyncio.sleep(1)

    async def monitor_sensor(self):
        active = set()
        incoming = set()
        raw_input = set()
        needed = self.joints_of_interest

        async def mon_input():
            nonlocal raw_input
            async for js_batch in self.sensor_pipeline.input_sub.listen_reliable():
                raw_input |= {
                    j_name for j_name, js in js_batch.items() if js.position is not None
                }

        async def mon():
            nonlocal incoming
            async for js_batch in self.sensor_pipeline.internal_sub.listen_reliable():
                incoming |= {
                    j_name for j_name, js in js_batch.items() if js.position is not None
                }
                if len(needed - active) == 0:
                    return

        async def good_display():
            nonlocal active, incoming
            async for t_ns in afor.Rate(10).listen():
                new = incoming - active
                if new != set():
                    print(
                        f"Joint data: {Fore.BLUE}Ready{Fore.RESET} for {list_cyanize(new)}"
                    )
                active |= incoming
                incoming = set()
                if len(needed - active) == 0:
                    print(
                        f"{Style.BRIGHT}Joint Data: {Fore.GREEN}FULLY READY :){Fore.RESET}{Style.RESET_ALL}"
                    )
                    return

        async def bad_display():
            await asyncio.sleep(3)
            if len(needed - active) == 0:
                return
            print(
                f"Joint data: {Fore.YELLOW}Missing {Fore.RESET}for {list_cyanize(needed-active)}. "
            )
            if len(active) == 0:
                print(f"Joint data: {Fore.RED}MISSING ALL :({Fore.RESET}")  # )
            if len(raw_input) != 0:
                print(
                    f"Joint data: {Style.BRIGHT}{Fore.MAGENTA}Tip{Fore.RESET}{Style.RESET_ALL} Before pre-processing I hear {list_cyanize(raw_input)}"
                )
            else:
                print(
                    f"Joint data: {Style.BRIGHT}{Fore.MAGENTA}Tip{Fore.RESET}{Style.RESET_ALL} No incoming messages containing position"
                )
            return

        async with asyncio.TaskGroup() as tg:
            tg.create_task(mon())
            tg.create_task(good_display())
            tg.create_task(bad_display())
            tg.create_task(mon_input())

    def setup_urdf(self):
        if self.PARAMS.urdf == "":
            (_model, _, _, joints_objects, ee) = (None, None, None, [], None)
            self.PARAMS.start_effector_name = "NO_URDF"
            self.PARAMS.end_effector_name = "NO_URDF"
        else:
            (_model, _, _, joints_objects, ee) = load_set_urdf_raw(
                self.PARAMS.urdf,
                make_ee(self.PARAMS.end_effector_name),
                self.PARAMS.start_effector_name,
            )
        joints_objects = [
            k
            for k in joints_objects
            if ((k.joint_type not in {"fixed"}) and (k.name != ""))
        ]
        joints_objects += [
            RTBJoint(
                joint_type="continuous",
                parent=None,
                child=None,
                name=jn,
                # limit=[0, 0],
            )
            for jn in self.PARAMS.add_joint
            if jn != ""
        ]
        _ee_n = ee.name if ee is not None else "all joints"

        if self.PARAMS.start_effector_name in {None, ""}:
            _baselink_name = _model.base_link.name
        else:
            _baselink_name = self.PARAMS.start_effector_name
        print(
            f"Base_link: {Fore.CYAN}{_baselink_name}{Fore.RESET}\n"
            f"End effector:  {Fore.CYAN}{_ee_n}{Fore.RESET}"
        )
        _limits_undefined = set()
        for j in joints_objects:
            self.limits[j.name] = (
                get_limit(j) if not self.PARAMS.ignore_limits else (-np.inf, np.inf)
            )
            if self.limits[j.name] == (-np.inf, np.inf):
                _limits_undefined.add(j.name)
        if len(_limits_undefined) == len(joints_objects):
            print(f"Joint limits: {Fore.YELLOW}All Undefined{Fore.RESET} ")
        elif len(_limits_undefined) == 0:
            print(f"Joint limits: {Fore.BLUE}All Defined{Fore.RESET} ")
        else:
            print(f"Joint limits: {Fore.YELLOW}Some Undefined{Fore.RESET} ")
        return joints_objects, _ee_n, _baselink_name, _limits_undefined

    def drop_outsiders(self, jsb: JStateBatch) -> JStateBatch:
        if not self.PARAMS.drop_ousiders:
            return jsb
        return subdict(jsb, self.joints_of_interest)

    def create_sensor_pipelines(self):
        self.sensor_pipeline = JointPipeline(
            self.sensor_sub, self.PARAMS.joint_buffer, self.PARAMS.batch_time
        )
        self.sensor_pipeline.pre_process = self.sensor_preproc
        self.sensor_pipeline.post_process = self.sensor_postproc
        self.sensor_pipeline.slow_rate = 1 / self.PARAMS.slow_pub_time

    async def sensor_preproc(self, jsb: JStateBatch) -> JStateBatch:
        jsb = self.lvl0_remap.unmap(jsb)
        jsb = self.drop_outsiders(jsb)
        return jsb

    async def sensor_postproc(self, jsb: JStateBatch) -> JStateBatch:
        jsb = self.lvl2_remap.map(jsb)
        return jsb

    @property
    def sensor_output(self) -> BaseSub[JStateBatch]:
        return self.sensor_pipeline.output_sub

    def create_command_pipelines(self):
        self.command_pipeline = JointPipeline(
            self.command_sub, self.PARAMS.joint_buffer, self.PARAMS.batch_time
        )
        self.command_pipeline.pre_process = self.command_preproc
        self.command_pipeline.post_process = self.command_postproc
        self.command_pipeline.slow_rate = 1 / self.PARAMS.slow_pub_time

    async def command_preproc(self, jsb: JStateBatch) -> JStateBatch:
        jsb = self.lvl2_remap.unmap(jsb)
        jsb = self.drop_outsiders(jsb)
        return jsb

    async def command_postproc(self, jsb: JStateBatch) -> JStateBatch:
        jsb = jsb.copy()
        jsb = joint_mapper.position_clamp(jsb, self.limits)
        return self.lvl0_remap.map(jsb)

    @property
    def command_output(self) -> BaseSub[JStateBatch]:
        return self.command_pipeline.output_sub
