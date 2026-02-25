import asyncio
from contextlib import suppress
from typing import List

import asyncio_for_robotics.ros2 as afor
import numpy as np
import rclpy
from asyncio_for_robotics import BaseSub
from motion_stack_msgs.srv import ReturnJointState
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

from motion_stack.ros2 import communication

from ...core.lvl1_rework import JointCore, JStateBatch, Lvl1Param, lvl1_default
from ...core.utils.joint_state import JState, Time
from ..communication import lvl1 as comms
from ..utils.conversion import time_to_ros
from ..utils.joint_state import ros2js_batch, ros2js_wrap, stateOrderinator3000


def params_from_ros(node: Node) -> Lvl1Param:
    node.declare_parameter("ms_params", value="{}")
    json_str = node.get_parameter("ms_params").get_parameter_value().string_value
    return Lvl1Param.from_json(json_str)


async def publish_js_from_sub(topic: afor.TopicInfo, sub: BaseSub[JStateBatch]):
    with afor.auto_session().lock() as node:
        pub = node.create_publisher(**topic.as_kwarg())
    async for jsb in sub.listen_reliable():
        if len(jsb) == 0:
            continue
        most_recent = max(
            [(k.time.nano if k.time is not None else 0) for k in jsb.values()]
        )
        for js in stateOrderinator3000(jsb.values()):
            js.header.stamp = time_to_ros(Time(most_recent)).to_msg()
            pub.publish(js)


async def joint_alive_serv():
    serv: afor.Server[Empty.Request, Empty.Response] = (
        afor.Server(
            **afor.TopicInfo(
                comms.alive.name,
                comms.alive.type,
                comms.alive.qos,
            ).as_kwarg()
        )
    )
    async for responder in serv.listen_reliable():
        responder.response
        responder.send()

async def joint_advert_serv(core: JointCore):
    serv: afor.Server[ReturnJointState.Request, ReturnJointState.Response] = (
        afor.Server(
            **afor.TopicInfo(
                comms.output.advertise.name,
                ReturnJointState,
                comms.output.advertise.qos,
            ).as_kwarg()
        )
    )
    async for responder in serv.listen_reliable():
        internal = core.sensor_pipeline.internal_state.accumulated
        names: List[str] = [h.name for h in internal.values()]
        none2nan = lambda x: x if x is not None else np.nan
        responder.response.js = JointState(
            name=names,
            position=[none2nan(h.position) for h in internal.values()],
            velocity=[none2nan(h.velocity) for h in internal.values()],
            effort=[none2nan(h.effort) for h in internal.values()],
        )
        if len(internal) > 0:
            most_recent = max(
                [(k.time.nano if k.time is not None else 0) for k in internal.values()]
            )
        else:
            most_recent = 0

        responder.response.js.header.stamp = time_to_ros(Time(most_recent)).to_msg()
        responder.send()


async def async_main():
    with afor.auto_session().lock() as node:
        PARAMS = params_from_ros(node)

    with open("/home/elian/Motion-Stack/m_zero.urdf") as f:
        PARAMS.urdf = f.read()
    sensor_sub_raw: afor.Sub[JointState] = afor.Sub(
        **afor.TopicInfo(
            f"{comms.input.motor_sensor.name}",
            comms.input.motor_sensor.type,
            comms.input.motor_sensor.qos,
        ).as_kwarg()
    )
    sensor_sub = afor.ConverterSub(sensor_sub_raw, ros2js_batch)
    command_sub_raw: afor.Sub[JointState] = afor.Sub(
        **afor.TopicInfo(
            f"{comms.input.joint_target.name}",
            comms.input.joint_target.type,
            comms.input.joint_target.qos,
        ).as_kwarg()
    )
    command_sub = afor.ConverterSub(command_sub_raw, ros2js_batch)
    core = JointCore(sensor_sub, command_sub, PARAMS)
    async with asyncio.TaskGroup() as tg:
        tg.create_task(core.run())
        tg.create_task(
            publish_js_from_sub(
                afor.TopicInfo(
                    f"{comms.output.motor_command.name}",
                    comms.output.motor_command.type,
                    comms.output.motor_command.qos,
                ),
                core.command_pipeline.output_sub,
            )
        )
        tg.create_task(
            publish_js_from_sub(
                afor.TopicInfo(
                    f"{comms.output.joint_state.name}",
                    comms.output.joint_state.type,
                    comms.output.joint_state.qos,
                ),
                core.sensor_pipeline.output_sub,
            )
        )
        tg.create_task(
            publish_js_from_sub(
                afor.TopicInfo(
                    f"{comms.output.continuous_joint_state.name}",
                    comms.output.continuous_joint_state.type,
                    comms.output.continuous_joint_state.qos,
                ),
                core.continuous_js_output,
            )
        )
        tg.create_task(joint_advert_serv(core))
        tg.create_task(joint_alive_serv())


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
