import random

import pytest

from motion_stack.core.utils.joint_state import JState, JStateBuffer
from motion_stack.core.utils.time import Time


@pytest.fixture
def delta():
    # Thresholds: 0.1 rad, 0.1 rad/s, 0.1 Nm, 1ns
    return JState(name="", time=Time(sec=1), position=0.1, velocity=0.1, effort=0.1)


@pytest.fixture
def buf(delta):
    return JStateBuffer(delta)


def test_empty_buffer(buf):
    assert buf.pull_new() == {}
    assert buf.pull_urgent() == {}


def test_first_push_is_urgent(buf):
    s = JState("j1", position=1.0)
    buf.push({"j1": s})
    u = buf.pull_urgent()
    assert "j1" in u.keys()
    assert u["j1"].position == 1.0
    s = JState("j1", time=Time(sec=1))
    buf.push({"j1": s})
    u = buf.pull_urgent()
    assert "j1" in u.keys()
    s = JState("j1", velocity=1.0)
    buf.push({"j1": s})
    u = buf.pull_urgent()
    assert "j1" in u.keys()
    s = JState("j1", effort=1.0)
    buf.push({"j1": s})
    u = buf.pull_urgent()
    assert "j1" in u.keys()


def test_no_change_not_urgent(buf):
    s1 = JState("j1", position=1.0, time=Time(sec=1))
    buf.push({"j1": s1})
    buf.pull_urgent()
    # push same state again
    s2 = JState("j1", position=1.0, time=Time(sec=1.1))
    buf.push({"j1": s2})
    u = buf.pull_urgent()
    assert u == {}


def test_change_above_delta_triggers_urgent(buf):
    s1 = JState("j1", position=1.0, time=Time(sec=1))
    buf.push({"j1": s1})
    u = buf.pull_urgent()
    assert "j1" in u
    s2 = JState("j1", position=1.3, time=Time(sec=1.1))
    buf.push({"j1": s2})
    u = buf.pull_urgent()
    assert "j1" in u
    s2 = JState("j1", position=1.33, time=Time(sec=1.2))
    buf.push({"j1": s2})
    u = buf.pull_urgent()
    assert u == {}
    s2 = JState("j1", position=1.33, time=Time(sec=2.11))
    buf.push({"j1": s2})
    u = buf.pull_urgent()
    assert "j1" in u


def test_pull_new(buf):
    s1 = JState("j1", position=1.0)
    buf.push({"j1": s1})
    n = buf.pull_new()
    assert "j1" in n
    assert n["j1"].position == 1.0
    # after flush, buffer is empty
    assert buf.pull_new() == {}
    s1 = JState("j1", position=1.0)
    buf.push({"j1": s1})
    n = buf.pull_new()
    assert "j1" in n
    assert n["j1"].position == 1.0


def test_accumulation_keeps_values(buf):
    s1 = JState(name="j1", position=1.0, time=Time(sec=1))
    s2 = JState(name="j1", velocity=2.0, time=Time(sec=2))
    buf.push({"j1": s1})
    buf.pull_new()
    buf.push({"j1": s2})
    buf.pull_new()
    # accumulated should merge position and velocity
    acc = buf.accumulated["j1"]
    assert acc.position == 1.0
    assert acc.velocity == 2.0


def test_past_data_doesnt_do_anything(buf):
    normal_data = JState(name="j1", position=1.0, time=Time(sec=2))
    data_in_past = JState(name="j1", position=20000.0, time=Time(sec=1))
    buf.push({"j1": normal_data})
    buf.pull_new()
    buf.push({"j1": data_in_past})
    n = buf.pull_new()
    assert "j1" not in n.keys()
    buf.push({"j1": normal_data})
    buf.push({"j1": data_in_past})
    n = buf.pull_new()
    assert "j1" in n.keys()
    assert n["j1"] == 1.0


def test_past_data_none_overide(buf):
    normal_data = JState(name="j1", position=1.0, time=Time(sec=2))
    data_in_past = JState(name="j1", position=20000.0, time=None)
    buf.push({"j1": normal_data})
    buf.pull_new()
    buf.push({"j1": data_in_past})
    n = buf.pull_new()
    assert "j1" in n.keys()
    buf.push({"j1": normal_data})
    buf.push({"j1": data_in_past})
    n = buf.pull_new()
    assert "j1" in n.keys()
    assert n["j1"] == data_in_past.position


@pytest.mark.asyncio
async def test_quickfuz_buffer():
    pos_vals = set(range(1, 40))
    joint_names = {f"joint{id}" for id in range(10)}

    js_send_queue = [
        JState(name=id, position=n, time=Time(sec=n))
        for id in joint_names
        for n in pos_vals
    ]
    random.shuffle(js_send_queue)

    buff = JStateBuffer(JState("", position=1, time=Time(sec=1)))
    for k in js_send_queue:
        buff.push(k)

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
