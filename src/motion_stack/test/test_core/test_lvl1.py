import asyncio
import time
from dataclasses import replace
from pathlib import Path

import asyncio_for_robotics as afor
import pytest
from asyncio_for_robotics.core._logger import setup_logger
from asyncio_for_robotics.core.sub import BaseSub

from motion_stack.core.lvl1_rework import (
    JointCore,
    JointPipeline,
    JStateBatch,
    Lvl1Param,
)
from motion_stack.core.utils.joint_state import JState
from motion_stack.core.utils.time import Time

HERE = Path(__file__).resolve().parent


@pytest.fixture
async def lvl1():
    sensor_sub = BaseSub()
    command_sub = BaseSub()
    lvl1 = JointCore(sensor_sub, command_sub, Lvl1Param(add_joint=["j1", "j2"]))
    async with asyncio.TaskGroup() as tg:
        task = tg.create_task(lvl1.run())
        yield lvl1
        task.cancel()


@pytest.fixture
def mzero_urdf():
    with open(HERE / "m_zero.urdf") as file:
        return file.read()


@pytest.fixture
async def lvl1_mzero_all(mzero_urdf: str):
    sensor_sub = BaseSub()
    command_sub = BaseSub()
    lvl1 = JointCore(sensor_sub, command_sub, Lvl1Param(urdf=mzero_urdf))
    async with asyncio.TaskGroup() as tg:
        task = tg.create_task(lvl1.run())
        yield lvl1
        task.cancel()


@pytest.fixture
async def lvl1_mzero_end1(mzero_urdf: str):
    sensor_sub = BaseSub()
    command_sub = BaseSub()
    lvl1 = JointCore(
        sensor_sub, command_sub, Lvl1Param(urdf=mzero_urdf, end_effector_name="end1")
    )
    async with asyncio.TaskGroup() as tg:
        task = tg.create_task(lvl1.run())
        yield lvl1
        task.cancel()


@pytest.fixture
async def lvl1_mzero_1(mzero_urdf: str):
    sensor_sub = BaseSub()
    command_sub = BaseSub()
    lvl1 = JointCore(
        sensor_sub, command_sub, Lvl1Param(urdf=mzero_urdf, end_effector_name=1)
    )
    async with asyncio.TaskGroup() as tg:
        task = tg.create_task(lvl1.run())
        yield lvl1
        task.cancel()


async def test_sensor_goes_through(lvl1: JointCore):
    payload = {
        "j1": JState("j1", Time(10), 0.1),
        "j2": JState("j2", Time(20), 0.2),
    }
    out = lvl1.joint_read_output.wait_for_next()
    lvl1.sensor_sub._input_data_asyncio(payload)
    out = await afor.soft_wait_for(out, 1)
    if isinstance(out, TimeoutError):
        pytest.fail("inputting sensor data did not result in joint read output")
    assert out == payload


async def test_command_idle_empty(lvl1: JointCore):
    out = lvl1.motor_output.wait_for_next()
    out = await afor.soft_wait_for(out, 1)
    if isinstance(out, TimeoutError):
        pytest.fail("inputting command data did not result in motor output")
    assert {"j1", "j2"} == set(out.keys())


async def test_command_goes_through(lvl1: JointCore):
    payload = {"j2": JState("j2", Time(20), 0.2)}
    await lvl1.motor_output.wait_for_next()
    out = lvl1.motor_output.wait_for_next()
    lvl1.command_sub._input_data_asyncio(payload)
    out = await afor.soft_wait_for(out, 1)
    if isinstance(out, TimeoutError):
        pytest.fail("inputting command data did not result in motor output")
    assert payload == out


async def test_unknown_joint_is_blocked(lvl1: JointCore):
    payload1 = {
        "j1": JState("j1", Time(10), 0.1),
        "j2": JState("j2", Time(20), 0.2),
    }
    payload2 = {
        "j1": JState("j1", Time(10), 0.1),
        "j2": JState("j2", Time(20), 0.2),
        "j3": JState("j3", Time(30), 0.3),
    }
    out = lvl1.joint_read_output.wait_for_next()
    lvl1.sensor_sub._input_data_asyncio(payload2)
    out = await afor.soft_wait_for(out, 1)
    if isinstance(out, TimeoutError):
        pytest.fail("inputting sensor data did not result in joint read output")
    assert out == payload1


async def test_mzero_all_sensor_go_through(lvl1_mzero_all: JointCore):
    payload = {
        f"joint{a}_{b}": JState(f"joint{a}_{b}", Time(10), 0.1)
        for a in range(1, 5)
        for b in range(1, 4)
    }
    payload_nono = {
        f"joint{a}_{b}": JState(f"joint{a}_{b}", Time(10), 0.1)
        for a in range(1, 5)
        for b in range(1, 5)
    }
    out = lvl1_mzero_all.joint_read_output.wait_for_next()
    lvl1_mzero_all.sensor_sub._input_data_asyncio(payload_nono)
    out = await afor.soft_wait_for(out, 1)
    if isinstance(out, TimeoutError):
        pytest.fail("inputting sensor data did not result in joint read output")
    assert out == payload


async def test_mzero_end1_sensor_go_through(lvl1_mzero_end1: JointCore):
    payload = {
        f"joint{a}_{b}": JState(f"joint{a}_{b}", Time(10), 0.1)
        for a in range(1, 2)
        for b in range(1, 4)
    }
    payload_nono = {
        f"joint{a}_{b}": JState(f"joint{a}_{b}", Time(10), 0.1)
        for a in range(1, 5)
        for b in range(1, 4)
    }
    out = lvl1_mzero_end1.joint_read_output.wait_for_next()
    lvl1_mzero_end1.sensor_sub._input_data_asyncio(payload_nono)
    out = await afor.soft_wait_for(out, 1)
    if isinstance(out, TimeoutError):
        pytest.fail("inputting sensor data did not result in joint read output")
    assert out == payload


async def test_mzero_1_sensor_go_through(lvl1_mzero_1: JointCore):
    payload = {
        f"joint{a}_{b}": JState(f"joint{a}_{b}", Time(10), 0.1)
        for a in range(1, 2)
        for b in range(1, 4)
    }
    payload_nono = {
        f"joint{a}_{b}": JState(f"joint{a}_{b}", Time(10), 0.1)
        for a in range(1, 5)
        for b in range(1, 4)
    }
    out = lvl1_mzero_1.joint_read_output.wait_for_next()
    lvl1_mzero_1.sensor_sub._input_data_asyncio(payload_nono)
    out = await afor.soft_wait_for(out, 1)
    if isinstance(out, TimeoutError):
        pytest.fail("inputting sensor data did not result in joint read output")
    assert out == payload
