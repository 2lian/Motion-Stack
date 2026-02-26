import asyncio
from contextlib import suppress
from pprint import pprint
from typing import Dict, List, Optional, Set

import asyncio_for_robotics.ros2 as afor
import numpy as np
import rclpy
from asyncio_for_robotics import BaseSub
from rclpy import qos
from std_msgs.msg import String

from ...core.lvl1_rework import JointPipeline, JStateBatch, Lvl1Info
from ...core.utils.joint_state import JState
from ...core.utils.time import Time
from ...ros2.communication import lvl1 as comms
from ...ros2.utils.joint_state import publish_jstate, ros2js_batch

INFORMATION_TOPIC = afor.TopicInfo(
    comms.output.info.name,
    comms.output.info.type,
    qos.QoSProfile(
        reliability=qos.ReliabilityPolicy.RELIABLE,
        history=qos.HistoryPolicy.KEEP_ALL,
        durability=qos.DurabilityPolicy.TRANSIENT_LOCAL,
    ),
)


class JointScanner(afor.ConverterSub[Lvl1Info]):
    def __init__(self) -> None:
        ros_sub = afor.Sub(**INFORMATION_TOPIC.as_kwarg())
        self.buffer: List[Lvl1Info] = []
        super().__init__(ros_sub, lambda msg: self.cbk(msg))

    def cbk(self, msg: String) -> Lvl1Info:
        inf = Lvl1Info.from_json(msg.data)
        self.buffer.append(inf)
        return inf


class JointHandler:
    def __init__(self, namespace: str, scanner: Optional[JointScanner] = None) -> None:
        self.namespace = namespace
        self.new_joint_sub: BaseSub[Set[str]] = BaseSub()
        self.available_joints: Set[str] = set()
        self._raw_sub = afor.Sub(
            **afor.TopicInfo(
                f"/{namespace}/{comms.output.joint_state.name}",
                comms.output.joint_state.type,
                comms.output.joint_state.qos,
            ).as_kwarg()
        )
        with afor.auto_session().lock() as node:
            self._raw_pub = node.create_publisher(
                **afor.TopicInfo(
                    f"/{namespace}/{comms.input.joint_target.name}",
                    comms.input.joint_target.type,
                    comms.input.joint_target.qos,
                ).as_kwarg()
            )
        self._sub = afor.ConverterSub(self._raw_sub, ros2js_batch)
        self._pipeline = JointPipeline(
            self._sub,
            buffer_delta=JState(
                name="",
                time=Time.sn(sec=0.5),
                position=np.deg2rad(0.05),
                velocity=np.deg2rad(0.01),
                effort=np.deg2rad(0.001),
            ),
            batch_time=0.005,
        )

    async def _check_for_change(self):
        async for k in self._pipeline.internal_sub.listen():
            now = set(self._pipeline.internal_state.accumulated.keys())
            if now != self.available_joints:
                self.new_joint_sub._input_data_asyncio(now - self.available_joints)
                self.available_joints = now

    async def run(self):
        async with asyncio.TaskGroup() as tg:
            tg.create_task(self._pipeline.run())
            tg.create_task(self._check_for_change())

    @property
    def joint_sub(self):
        return self._pipeline.output_sub

    @property
    def accumulated(self) -> JStateBatch:
        return self._pipeline.internal_state.accumulated

    def send(self, states: JStateBatch):
        publish_jstate(self._raw_pub, list(states.values()))

    def reset(self):
        self.available_joints = set()
        self._pipeline.internal_state.accumulated = dict()

async def tell_me_new_joints(joint_handler: JointHandler):
    async for joints in joint_handler.new_joint_sub.listen_reliable():
        print(f"Got new joints: {joints}")


async def main():
    dj = JointScanner()
    ns = (await dj.wait_for_value()).namespace
    all_handlers: Dict[str, JointHandler] = {}
    async with asyncio.TaskGroup() as tg:
        async for inf in dj.listen_reliable():
            ns = inf.namespace
            if ns in all_handlers:
                print(f"Handler already exists for {ns}. Resetting")
                all_handlers[ns].reset()
                continue
            print("Found limb: ", ns)
            jh = JointHandler(ns, dj)
            all_handlers[ns] = jh
            tg.create_task( jh.run())
            tg.create_task(tell_me_new_joints(jh))


if __name__ == "__main__":
    rclpy.init()
    try:
        with suppress(KeyboardInterrupt):
            asyncio.run(main())
    finally:
        rclpy.shutdown()
