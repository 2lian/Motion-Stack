import asyncio
import logging
import random
import string
import time
from typing import Awaitable, Tuple

import pytest
import asyncio_for_robotics.zenoh as afor
import zenoh

import motion_stack.zenoh.utils.communication as comms
from motion_stack.core.utils.joint_state import JState, JStateBuffer
from motion_stack.core.utils.time import Time
from motion_stack.zenoh.utils.async_wrapper import ParsedPub, ParsedSub

from .test_serialize import js_combinations, str_combinations

logger = logging.getLogger("motion_stack." + __name__)


@pytest.fixture
async def js_pubsub():
    pub = ParsedPub("ms/test/js_pubsub/hey")
    sub = ParsedSub("ms/test/js_pubsub/**")
    return pub, sub


@pytest.fixture
async def str_pubsub():
    pub = ParsedPub("ms/test/str_pubsub/hey")
    sub = ParsedSub("ms/test/str_pubsub/**")
    return pub, sub


@pytest.mark.parametrize("data", js_combinations)
async def test_js_pubsub(
    js_pubsub: Tuple[ParsedPub[JState], ParsedSub[JState]], data: JState
):
    pub, sub = js_pubsub
    gen = sub.listen_reliable(fresh=True)
    pub.pub(data)
    msg = await afor.soft_wait_for(sub.wait_for_new(), 1)
    assert msg == data
