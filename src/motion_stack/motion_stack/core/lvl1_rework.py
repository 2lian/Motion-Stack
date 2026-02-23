import asyncio
import logging
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Final, List, Optional, OrderedDict, Tuple, Union

import asyncio_for_robotics as afor
import numpy as np
from asyncio_for_robotics.core.sub import BaseSub
from colorama import Fore, Style
from roboticstoolbox.tools.urdf.urdf import Joint as RTBJoint

from ..api.injection.remapper import StateRemapper
from .utils import joint_mapper, static_executor
from .utils.joint_state import JState, JStateBuffer
from .utils.printing import list_cyanize
from .utils.robot_parsing import get_limit, load_set_urdf_raw, make_ee
from .utils.static_executor import default_param_dict
from .utils.time import Time


@dataclass
class Lvl1Param:
    urdf: str
    namespace: str
    end_effector_name: Union[None, str, int]
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
    namespace=f"ms/limb_{default_param_dict['leg_number'][0]}",
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

JStateBatch = Dict[str, JState]


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
            tg.create_task(self._input_loop())
            tg.create_task(self._flush_loop())
            tg.create_task(self._slow_loop())
            tg.create_task(self._output_loop())

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
                tg.create_task(self._parallel_preproc(jsb))

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
                tg.create_task(self._parallel_postproc(jsb))

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
        self.sensor_pipeline: JointPipeline
        self.command_pipeline: JointPipeline
        self.joints_objects: List[RTBJoint]

        self.lvl0_remap = StateRemapper()  # empty
        self.lvl2_remap = StateRemapper()  # empty
        # self.send_to_lvl0_callbacks: List[Callable[[Dict[str, JState]], Any]] = []
        # self.send_to_lvl2_callbacks: List[Callable[[Dict[str, JState]], Any]] = []
        self.create_sensor_pipelines()
        self.create_command_pipelines()

        self.limits: Dict[str, Tuple[float, float]] = dict()
        self.setup_urdf()

    async def run(self):
        async with asyncio.TaskGroup() as tg:
            tg.create_task(self.sensor_pipeline.run())
            tg.create_task(self.command_pipeline.run())
            tg.create_task(self.monitor_sensor())
            tg.create_task(self.send_empty_command())

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
        needed = {k.name for k in self.joints_objects}

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
                        f"Joint Data: {Style.BRIGHT}{Fore.GREEN}FULLY READY :){Fore.RESET}"
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
            return

        async with asyncio.TaskGroup() as tg:
            tg.create_task(mon())
            tg.create_task(good_display())
            tg.create_task(bad_display())

    def setup_urdf(self):
        (_model, _, _, self.joints_objects, ee) = load_set_urdf_raw(
            self.PARAMS.urdf,
            make_ee(self.PARAMS.end_effector_name),
            self.PARAMS.start_effector_name,
        )
        self.joints_objects = [
            k
            for k in self.joints_objects
            if ((k.joint_type not in {"fixed"}) and (k.name != ""))
        ]
        self.joints_objects += [
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
        for j in self.joints_objects:
            self.limits[j.name] = (
                get_limit(j) if not self.PARAMS.ignore_limits else (-np.inf, np.inf)
            )
            if self.limits[j.name] == (-np.inf, np.inf):
                _limits_undefined.add(j.name)
        if len(_limits_undefined) == len(self.joints_objects):
            print(f"Joint limits: {Fore.YELLOW}All Undefined{Fore.RESET} ")
        elif len(_limits_undefined) == 0:
            print(f"Joint limits: {Fore.BLUE}All Defined{Fore.RESET} ")
        else:
            print(f"Joint limits: {Fore.YELLOW}Some Undefined{Fore.RESET} ")
        return _ee_n, _baselink_name, _limits_undefined

    def create_sensor_pipelines(self):
        self.sensor_pipeline = JointPipeline(
            self.sensor_sub, self.PARAMS.joint_buffer, self.PARAMS.batch_time
        )
        self.sensor_pipeline.pre_process = self.sensor_preproc
        self.sensor_pipeline.post_process = self.sensor_postproc
        self.sensor_pipeline.slow_rate = 1 / self.PARAMS.slow_pub_time

    async def sensor_preproc(self, jsb: JStateBatch) -> JStateBatch:
        return self.lvl0_remap.unmap(jsb)

    async def sensor_postproc(self, jsb: JStateBatch) -> JStateBatch:
        return self.lvl2_remap.map(jsb)

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
        return jsb

    async def command_postproc(self, jsb: JStateBatch) -> JStateBatch:
        jsb = jsb.copy()
        jsb = joint_mapper.position_clamp(jsb, self.limits)
        return self.lvl0_remap.map(jsb)

    @property
    def command_output(self) -> BaseSub[JStateBatch]:
        return self.command_pipeline.output_sub
