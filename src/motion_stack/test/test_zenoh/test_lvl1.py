import asyncio
import logging
import os
import random
import time
from copy import deepcopy
from typing import Awaitable

import numpy as np
import pytest
import zenoh

import motion_stack.zenoh.utils.communication as comms
from motion_stack.core.logger import setup_logger
from motion_stack.core.lvl1_pipeline import lvl1_default
from motion_stack.core.utils.joint_state import (
    JState,
    JStateBuffer,
    js_changed,
    js_diff,
    multi_to_js_dict,
)
from motion_stack.core.utils.math import patch_numpy_display_light
from motion_stack.core.utils.time import Time
from motion_stack.zenoh.default_node.lvl1 import DefaultLvl1

from .test_pubsub_js import listen_for

patch_numpy_display_light(4)

logger = logging.getLogger("motion_stack." + __name__)


# @pytest.mark.only
@pytest.mark.parametrize(
    "two_to_zero",
    [
        True,
        False,
    ],
)
async def test_joints_go_through(two_to_zero: bool):
    pos_vals = set(range(1, 40))
    joint_names = {f"joint{id}" for id in range(3)}

    js_data = [
        JState(name=id, position=n, time=Time(sec=n))
        for id in joint_names
        for n in pos_vals
    ]
    random.seed(0)
    random.shuffle(js_data)

    delta = JState("", Time(sec=0), 0)
    params = deepcopy(lvl1_default)
    params.joint_buffer = delta
    params.add_joint = list(joint_names)
    node = DefaultLvl1(params)
    if two_to_zero:
        pub_to_input = comms.JointStatePub(node.lvl2_sub.key.removesuffix("/**"))
    else:
        pub_to_input = comms.JointStatePub(node.lvl0_sub.key.removesuffix("/**"))
    logger.debug(f"Publishing on: {pub_to_input.key}")
    if two_to_zero:
        sub_of_output = comms.JointStateSub(node.lvl0_pub.key + "/**")
    else:
        sub_of_output = comms.JointStateSub(node.lvl2_pub.key + "/**")
    logger.debug(f"Listening on: {pub_to_input.key}")

    async def periodic():
        nonlocal pub_to_input, js_data
        while len(js_data) > 0:
            js = js_data.pop()
            pub_to_input.publish(js)
            # logger.debug(f"publishing on {in_pub.key}/{js}")
            await asyncio.sleep(0.0001)
        logger.debug("publishing done")

    pub_task = asyncio.create_task(periodic())

    try:
        logger.debug("waiting for pubtask to be done")
        buff = await listen_for(sub_of_output, timeout=pub_task)
        logger.debug("pubtask done")
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
        pub_to_input.close()
        sub_of_output.close()
        node.close()


# @pytest.mark.only
@pytest.mark.parametrize(
    "delta",
    [
        JState("", Time(sec=0), 0),
        JState("", Time(sec=5), 0),
        JState("", Time(sec=0), 3),
        JState("", Time(sec=10), 3),
        JState("", None, 3),
    ],
)
@pytest.mark.parametrize("two_to_zero", [True, False])
async def test_buffer_delta(delta: JState, two_to_zero: bool):
    pos_vals = set(range(1, 40))
    joint_names = {f"joint{id}" for id in range(3)}

    js_data = [
        JState(name=id, position=n, time=Time(sec=n))
        for id in joint_names
        for n in pos_vals
    ]
    random.seed(0)
    random.shuffle(js_data)

    params = deepcopy(lvl1_default)
    params.joint_buffer = delta
    params.add_joint = list(joint_names)
    node = DefaultLvl1(params)
    if two_to_zero:
        in_pub = comms.JointStatePub(node.lvl2_sub.key.removesuffix("/**"))
    else:
        in_pub = comms.JointStatePub(node.lvl0_sub.key.removesuffix("/**"))
    logger.debug(f"Publishing on: {in_pub.key}")
    if two_to_zero:
        out_sub = comms.JointStateSub(node.lvl0_pub.key + "/**")
    else:
        out_sub = comms.JointStateSub(node.lvl2_pub.key + "/**")
    logger.debug(f"Listening on: {in_pub.key}")

    async def periodic():
        nonlocal in_pub, js_data
        while len(js_data) > 0:
            await asyncio.sleep(0.0001)
            js = js_data.pop()
            in_pub.publish(js)
            # logger.debug(f"publishing on {in_pub.key}/{js}")
        logger.debug("publishing done")

    minibuf = {}

    async def listen_loop():
        nonlocal minibuf
        while 1:
            try:
                new = await asyncio.wait_for(out_sub.listen_all(), timeout=1)
                new = multi_to_js_dict(new)
            except TimeoutError:
                assert False, f"did not get data on {out_sub.key}"
            for k, v in new.items():
                prev = minibuf.get(v.name)
                if prev is None:
                    continue
                assert v.time >= prev.time
                assert js_changed(v, prev, delta)
            minibuf.update(multi_to_js_dict(new))

    pub_task = asyncio.create_task(periodic())
    listen_task = asyncio.create_task(listen_loop())

    try:
        await pub_task
        listen_task.cancel()
        assert joint_names == minibuf.keys(), "failed to gedata about all joints"
    finally:
        pub_task.cancel()
        try:
            await pub_task
        except asyncio.CancelledError:
            pass
        in_pub.close()
        out_sub.close()
        node.close()


# @pytest.mark.only
@pytest.mark.parametrize("two_to_zero", [True, False])
async def test_batching(two_to_zero: bool):
    pos_vals = set(range(1, 40))
    joint_names = {f"joint{id}" for id in range(3)}

    js_data = [
        JState(name=id, position=n, time=Time(sec=n))
        for id in joint_names
        for n in pos_vals
    ]
    random.seed(0)
    random.shuffle(js_data)

    params = deepcopy(lvl1_default)
    params.batch_time = 0.1
    params.joint_buffer = JState("", Time(sec=0), 0)
    params.add_joint = list(joint_names)
    node = DefaultLvl1(params)
    if two_to_zero:
        in_pub = comms.JointStatePub(node.lvl2_sub.key.removesuffix("/**"))
    else:
        in_pub = comms.JointStatePub(node.lvl0_sub.key.removesuffix("/**"))
    logger.debug(f"Publishing on: {in_pub.key}")
    if two_to_zero:
        out_sub = comms.JointStateSub(node.lvl0_pub.key + "/**")
    else:
        out_sub = comms.JointStateSub(node.lvl2_pub.key + "/**")
    logger.debug(f"Listening on: {in_pub.key}")

    async def periodic():
        nonlocal in_pub, js_data
        while len(js_data) > 0:
            await asyncio.sleep(0.000)
            js = js_data.pop()
            in_pub.publish(js)
            # logger.debug(f"publishing on {in_pub.key}/{js}")
        logger.debug("publishing done")

    pub_task = asyncio.create_task(periodic())

    try:
        await pub_task
        assert (
            out_sub.queue.empty()
        ), f"lvl1 should only publish  {params.batch_time=} after the first message"
        await asyncio.sleep(params.batch_time)
        assert (
            not out_sub.queue.empty()
        ), f"lvl1 have published after {params.batch_time=}"
    finally:
        pub_task.cancel()
        try:
            await pub_task
        except asyncio.CancelledError:
            pass
        in_pub.close()
        out_sub.close()
        node.close()


# @pytest.mark.only
@pytest.mark.parametrize(
    "two_to_zero",
    [
        # True,
        False,
    ],
)
async def test_regular(two_to_zero: bool):
    joint_count = 1

    delta = JState("", Time(sec=10), 1, 1, 1)
    ha = 0.1
    params = deepcopy(lvl1_default)
    params.joint_buffer = delta
    params.slow_pub_time = ha
    node = DefaultLvl1(params)
    if two_to_zero:
        in_pub = comms.JointStatePub(node.lvl2_sub.key.removesuffix("/**"))
    else:
        in_pub = comms.JointStatePub(node.lvl0_sub.key.removesuffix("/**"))
    logger.debug(f"Publishing on: {in_pub.key}")
    if two_to_zero:
        out_sub = comms.JointStateSub(node.lvl0_pub.key + "/**")
    else:
        out_sub = comms.JointStateSub(node.lvl2_pub.key + "/**")
    logger.debug(f"Listening on: {in_pub.key}")

    async def periodic():
        nonlocal in_pub
        while 1:
            js = [
                JState(f"joint_{n}", Time(sec=time.time()), 1.23)
                for n in range(joint_count)
            ]
            in_pub.publish(js)
            await asyncio.sleep(0.01)
        logger.debug("publishing done")

    pub_task = asyncio.create_task(periodic())

    try:
        try:
            await asyncio.wait_for(out_sub.listen(), 1)
            while not out_sub.queue.empty():
                await out_sub.listen_all()
        except TimeoutError:
            assert False, "lvl1 should publish somthing the first time"

        assert params.slow_pub_time  == pytest.approx(ha)

        timings = []
        meas = 5
        for k in range(joint_count * meas):
            await asyncio.wait_for(
                out_sub.listen(), timeout=params.slow_pub_time  * 1.2
            )
            timings.append(time.time())

        t = np.array(timings)
        dt = np.diff(t)

        logger.info(f"delta time: {dt}")

        is_long = dt > (params.slow_pub_time  * 0.8)
        assert np.sum(is_long) >= meas - 1

        while not out_sub.queue.empty():
            await out_sub.listen_all()
        pub_task.cancel()
        await asyncio.sleep(params.slow_pub_time  * 1.1)

        while not out_sub.queue.empty():
            await out_sub.listen_all()

        with pytest.raises(TimeoutError) as e:
            await asyncio.wait_for(out_sub.listen(), params.slow_pub_time  * 1.1)

    finally:
        pub_task.cancel()
        try:
            await pub_task
        except asyncio.CancelledError:
            pass
        in_pub.close()
        out_sub.close()
        node.close()

# @pytest.mark.only
async def test_not_regular():
    with pytest.raises(Exception):
        await asyncio.wait_for(test_regular(two_to_zero=True), 3)
