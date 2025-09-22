import asyncio
import logging
import os
import random
import time
from typing import Awaitable

import pytest
import zenoh

import motion_stack.zenoh.utils.communication as comms
from motion_stack.core.utils.joint_state import JState, JStateBuffer
from motion_stack.core.utils.time import Time
from motion_stack.zenoh.base_node.lvl1 import Lvl1Node
from motion_stack.zenoh.utils.auto_session import auto_session

from .test_pubsub_js import listen_for

@pytest.mark.asyncio
async def test_lvl0_to_lvl2():
    pos_vals = set(range(1, 40))
    joint_names = {f"joint{id}" for id in range(10)}

    js_data = [
        JState(name=id, position=n, time=Time(sec=n))
        for id in joint_names
        for n in pos_vals
    ]
    random.seed(0)
    random.shuffle(js_data)

    node = Lvl1Node()
    node.d_required = joint_names
    lvl0_key = node.lvl0_sub.key.removesuffix("/**")
    in_pub = comms.JointStatePub(lvl0_key)
    logging.debug(f"Publishing on: {in_pub.key}")
    lvl2_key = node.lvl2_pub.key
    out_sub = comms.JointStateSub(lvl2_key+"/**")
    logging.debug(f"Listening on: {in_pub.key}")

    async def periodic():
        nonlocal in_pub, js_data
        while len(js_data) > 0:
            js = js_data.pop()
            in_pub.publish(js)
            # logging.debug(f"publishing on {in_pub.key}/{js}")
            await asyncio.sleep(0.0001)
        logging.debug("publishing done")

    pub_task = asyncio.create_task(periodic())

    try:
        print("waiting for pubtask to be done")
        buff = await listen_for(out_sub, timeout=pub_task)
        print("pubtask done")
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
        in_pub.close()
        out_sub.close()
        node.close()

@pytest.mark.asyncio
async def test_lvl2_to_lvl0():
    pos_vals = set(range(1, 40))
    joint_names = {f"joint{id}" for id in range(10)}

    js_data = [
        JState(name=id, position=n, time=Time(sec=n))
        for id in joint_names
        for n in pos_vals
    ]
    random.seed(0)
    random.shuffle(js_data)

    node = Lvl1Node()
    node.d_required = joint_names
    lvl2_key = node.lvl2_sub.key.removesuffix("/**")
    in_pub = comms.JointStatePub(lvl2_key)
    logging.debug(f"Publishing on: {in_pub.key}")
    lvl0_key = node.lvl0_pub.key
    out_sub = comms.JointStateSub(lvl0_key+"/**")
    logging.debug(f"Listening on: {in_pub.key}")

    async def periodic():
        nonlocal in_pub, js_data
        while len(js_data) > 0:
            js = js_data.pop()
            in_pub.publish(js)
            # logging.debug(f"publishing on {in_pub.key}/{js}")
            await asyncio.sleep(0.0001)
        logging.debug("publishing done")

    pub_task = asyncio.create_task(periodic())

    try:
        print("waiting for pubtask to be done")
        buff = await listen_for(out_sub, timeout=pub_task)
        print("pubtask done")
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
        in_pub.close()
        out_sub.close()
        node.close()
