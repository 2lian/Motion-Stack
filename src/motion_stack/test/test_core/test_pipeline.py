import asyncio
import time
from dataclasses import replace

import asyncio_for_robotics as afor
import pytest
from asyncio_for_robotics.core.sub import BaseSub

from motion_stack.core.lvl1_rework import JointPipeline, JStateBatch
from motion_stack.core.utils.joint_state import JState
from motion_stack.core.utils.time import Time


@pytest.fixture
def delta():
    return JState(name="", time=Time(sec=1), position=0.1)


@pytest.fixture
def input_sub():
    return BaseSub()


@pytest.fixture
async def pipeline(input_sub, delta):
    p = JointPipeline(
        input_sub=input_sub,
        buffer_delta=delta,
        batch_time=0.003,
    )
    task = asyncio.create_task(p.execute_pipeline())
    yield p
    task.cancel()


async def test_input_reaches_internal_sub(
    pipeline: JointPipeline, input_sub: BaseSub[JStateBatch]
):
    jsb = {
        "a": JState("a", time=Time(sec=1), position=1.0),
        "b": JState("b", time=Time(sec=1), position=2.0),
        "c": JState("c", time=Time(sec=1), position=3.0),
    }
    input_sub._input_data_asyncio(jsb)

    start = time.time()
    out = await pipeline.internal_sub.wait_for_value()
    end = time.time()

    assert out == jsb
    assert (end - start) <= pipeline.slow_rate

    out = await afor.soft_wait_for(pipeline.internal_sub.wait_for_new(), 1)
    assert isinstance(out, TimeoutError), "Should not update when nothing"


async def test_multiple_small_batches_aggregate_fast(
    pipeline: JointPipeline, input_sub: BaseSub[JStateBatch]
):
    jsb1 = {"a": JState("a", time=Time(sec=1), position=1.0)}
    jsb2 = {"b": JState("b", time=Time(sec=1), position=2.0)}
    jsb3 = {"c": JState("c", time=Time(sec=1), position=3.0)}

    start = time.time()
    input_sub._input_data_asyncio(jsb1)
    input_sub._input_data_asyncio(jsb2)
    input_sub._input_data_asyncio(jsb3)

    out = await pipeline.output_sub.wait_for_value()
    end = time.time()

    assert out == {**jsb1, **jsb2, **jsb3}
    assert (end - start) <= pipeline.slow_rate


async def test_small_changes_go_through_slow_path(
    pipeline: JointPipeline, input_sub: BaseSub[JStateBatch]
):
    # First batch establishes baseline (urgent)
    jsb1 = {
        "a": JState("a", time=Time(sec=1), position=1.0),
        "b": JState("b", time=Time(sec=1), position=2.0),
        "c": JState("c", time=Time(sec=1), position=3.0),
    }
    input_sub._input_data_asyncio(jsb1)
    await pipeline.output_sub.wait_for_value()

    # Below delta → not urgent
    jsb3 = {"a": JState("a", time=Time(sec=1.2), position=1.02)}
    jsb2 = {"a": JState("a", time=Time(sec=1.1), position=1.01)}

    start = time.time()
    input_sub._input_data_asyncio(jsb2)
    input_sub._input_data_asyncio(jsb3)

    out = await pipeline.output_sub.wait_for_new()
    end = time.time()

    assert out == jsb3
    delta_t = end - start
    assert delta_t >= 1 / pipeline.slow_rate * 0.8


async def test_pre_process_is_applied_to_internal_sub(
    pipeline: JointPipeline, input_sub: BaseSub[JStateBatch]
):
    called = False
    post_called = False

    async def pre(jsb: JStateBatch) -> JStateBatch:
        nonlocal called
        called = True
        return {k + "_pre": replace(v, name=v.name + "_pre") for k, v in jsb.items()}

    async def post(jsb: JStateBatch) -> JStateBatch:
        nonlocal post_called
        post_called = True
        return {k + "_post": replace(v, name=v.name + "_post") for k, v in jsb.items()}

    pipeline.pre_process = pre
    pipeline.post_process = post

    jsb = {
        "a": JState("a", time=Time(sec=1), position=1.0),
    }
    input_sub._input_data_asyncio(jsb)

    inter = await pipeline.buffered_sub.wait_for_value()
    out = await pipeline.output_sub.wait_for_value()

    assert called is True
    assert post_called is True
    assert inter == {
        "a_pre": JState("a_pre", time=Time(sec=1), position=1.0),
    }
    assert out == {
        "a_pre_post": JState("a_pre_post", time=Time(sec=1), position=1.0),
    }
