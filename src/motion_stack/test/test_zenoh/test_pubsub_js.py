import asyncio
import random
import time
from typing import Awaitable

import pytest
import zenoh

import motion_stack.zenoh.utils.communication as comms
from motion_stack.core.utils.joint_state import JState, JStateBuffer
from motion_stack.core.utils.time import Time
from motion_stack.zenoh.utils.auto_session import auto_session


@pytest.mark.asyncio
async def test_send_receive():
    sub = comms.JointStateSub(key="ms/test/joint_test/**")
    pub = comms.JointStatePub(key="ms/test/joint_test")
    sent_data = JState(
        name="test",
        position=1,
        time=Time(sec=12),
    )

    async def periodic():
        nonlocal pub
        while True:
            pub.publish(sent_data)
            print(f"{time.time()} published")
            await asyncio.sleep(0.1)

    pub_task = asyncio.create_task(periodic())

    try:
        received_data = await asyncio.wait_for(sub.listen(), 5)
        assert received_data == sent_data
    except TimeoutError:
        assert False, f"sub did not receive response in time"
    finally:
        pub_task.cancel()
        try:
            await pub_task
        except asyncio.CancelledError:
            pass
        sub.close()
        pub.close()


async def listen_for(sub: comms.JointStateSub, timeout: Awaitable) -> JStateBuffer:
    buff = JStateBuffer(JState("", position=1, time=Time(sec=1)))

    async def fill_buf():
        while 1:
            msg = await sub.listen()
            buff.push(msg)

    task = asyncio.create_task(fill_buf())

    try:
        await timeout
    finally:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    return buff


@pytest.mark.asyncio
async def test_pubsub_buffer():
    sub = comms.JointStateSub(key="ms/test/joint_test/**")
    pub = comms.JointStatePub(key="ms/test/joint_test")

    pos_vals = set(range(1, 40))
    joint_names = {f"joint{id}" for id in range(10)}

    js_send_queue = [
        JState(name=id, position=n, time=Time(sec=n))
        for id in joint_names
        for n in pos_vals
    ]
    random.seed(0)
    random.shuffle(js_send_queue)

    async def periodic():
        nonlocal pub, js_send_queue
        while len(js_send_queue) > 0:
            pub.publish(js_send_queue.pop())
            await asyncio.sleep(0.0001)

    pub_task = asyncio.create_task(periodic())

    try:
        buff = await listen_for(sub, timeout=pub_task)
        urg = buff.pull_urgent()
        assert set(urg.keys()) == joint_names, "should have all joints"
        for k, v in urg.items():
            assert v == JState(
                name=k, position=max(pos_vals), time=Time(sec=max(pos_vals))
            ), "all joints should be at max val"
        is_something = buff.pull_urgent()
        assert not is_something, "second call should be empty"
        is_something = buff.pull_new()
        assert not is_something, "second call should be empty"
        acc = buff.accumulated
        assert set(acc.keys()) == joint_names, "accumulated should have everything"
        for k, v in acc.items():
            assert v == JState(
                name=k, position=max(pos_vals), time=Time(sec=max(pos_vals))
            ), "all joints should be at max"
    finally:
        pub_task.cancel()
        try:
            await pub_task
        except asyncio.CancelledError:
            pass
        sub.close()
        pub.close()
