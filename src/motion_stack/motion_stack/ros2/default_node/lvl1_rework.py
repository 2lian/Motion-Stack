import asyncio
from contextlib import suppress
from typing import Any, Callable, List

import asyncio_for_robotics.ros2 as afor
import numpy as np
import rclpy
from asyncio_for_robotics import BaseSub
from motion_stack_msgs.srv import ReturnJointState
from rclpy.node import Node
from roboticstoolbox.robot.Robot import xacro
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

from ...core.lvl1_rework import JointCore, JStateBatch, lvl1_default
from ...core.utils.joint_state import JState
from ..communication import lvl1 as comms
from ..utils.joint_state import ros2js_wrap, stateOrderinator3000


async def publish_command(topic: afor.TopicInfo, sub: BaseSub[JStateBatch]):
    with afor.auto_session().lock() as node:
        pub = node.create_publisher(**topic.as_kwarg())
    async for jsb in sub.listen_reliable():
        for js in stateOrderinator3000(jsb.values()):
            pub.publish(js)


async def async_main():
    PARAMS = lvl1_default
    with open("/home/elian/Motion-Stack/m_zero.urdf") as f:
        PARAMS.urdf = f.read()
    sensor_sub = afor.Sub(
        **afor.TopicInfo(
            f"{comms.input.motor_sensor.name}",
            comms.input.motor_sensor.type,
            comms.input.motor_sensor.qos,
        ).as_kwarg()
    )
    command_sub = afor.Sub(
        **afor.TopicInfo(
            f"{comms.input.joint_target.name}",
            comms.input.joint_target.type,
            comms.input.joint_target.qos,
        ).as_kwarg()
    )
    core = JointCore(sensor_sub, command_sub, PARAMS)
    async with asyncio.TaskGroup() as tg:
        tg.create_task(core.run())
        tg.create_task(
            publish_command(
                afor.TopicInfo(
                    f"{comms.output.motor_command.name}",
                    comms.output.motor_command.type,
                    comms.output.motor_command.qos,
                ),
                core.command_pipeline.output_sub,
            )
        )


def main(*args, **kwargs):
    rclpy.init()
    ses = afor.auto_session()
    try:
        with suppress(KeyboardInterrupt):
            asyncio.run(async_main())
    finally:
        ses.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
