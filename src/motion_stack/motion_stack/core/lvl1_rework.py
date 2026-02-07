import asyncio
import logging
import traceback
from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Callable, Dict, Final, List, Optional, OrderedDict, Tuple

import asyncio_for_robotics as afor
from asyncio_for_robotics.core.sub import BaseSub
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

    async def execute_pipeline(self):
        """Run the pipeline.

        This coroutine must be awaited to start all internal processing loops.
        The pipeline terminates if any task in the task group raises.
        """
        async with asyncio.TaskGroup() as tg:
            tg.create_task(self._input_loop())
            tg.create_task(self._flush_loop())
            tg.create_task(self._slow_loop())
            tg.create_task(self._output_loop())

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
        """Periodically flush non-urgent buffered data.
        """
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
