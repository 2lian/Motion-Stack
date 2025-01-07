from typing import Dict, List
import pytest
from motion_stack.core.utils.joint_state import JState, js_from_dict_list, Time, impose_state, js_diff, js_changed

def test_jstate_creation():
    state = JState(name="joint_1", time=Time(1_000_000_000), position=1.0, velocity=0.5, effort=0.2)
    assert state.name == "joint_1"
    assert state.time.nano() == 1_000_000_000
    assert state.position == 1.0
    assert state.velocity == 0.5
    assert state.effort == 0.2

def test_getattr():
    state = JState(name="joint_1", time=Time(1_000_000_000), position=1.0)
    assert state.getattr("name") == "joint_1"
    assert state.getattr("time").nano() == 1_000_000_000
    assert state.getattr("position") == 1.0
    assert state.getattr("velocity") is None
    assert state.getattr("effort") is None

def test_copy():
    state = JState(name="joint_1", time=Time(1_000_000_000), position=1.0)
    copied_state = state.copy()
    assert copied_state == state
    assert copied_state is not state  # Ensure it's a different object

def test_is_initialized():
    state_uninitialized = JState(name="joint_1")
    state_partial = JState(name="joint_2", position=1.0)
    state_full = JState(name="joint_3", position=1.0, velocity=0.5, effort=0.2)

    assert not state_uninitialized.is_initialized
    assert state_partial.is_initialized
    assert state_full.is_initialized

def test_js_from_dict_list():
    dil: Dict[str, List] = {
        "name": ["joint_1", "joint_2"],
        "time": [Time(1_000_000_000), Time(2_000_000_000)],
        "position": [1.0, 2.0],
        "velocity": [0.5, 1.0],
        "effort": [0.2, 0.4],
        "something_else": [1, 4],
        "oh noe": [4],
        "crap": "hey, hello",
    }
    states = js_from_dict_list(dil)
    assert len(states) == 2

    assert states[0].name == "joint_1"
    assert states[0].time.nano() == 1_000_000_000
    assert states[0].position == 1.0
    assert states[0].velocity == 0.5
    assert states[0].effort == 0.2

    assert states[1].name == "joint_2"
    assert states[1].time.nano() == 2_000_000_000
    assert states[1].position == 2.0
    assert states[1].velocity == 1.0
    assert states[1].effort == 0.4

def test_js_from_dict_list_missing_values():
    dil = {
        "name": ["joint_1", "joint_2"],
        "time": [Time(1_000_000_000), None],
        "position": [1.0, None],
        "velocity": [],
    }
    states = js_from_dict_list(dil)
    assert len(states) == 2

    assert states[0].name == "joint_1"
    assert states[0].time.nano() == 1_000_000_000
    assert states[0].position == 1.0
    assert states[0].velocity is None
    assert states[0].effort is None

    assert states[1].name == "joint_2"
    assert states[1].time is None
    assert states[1].position is None
    assert states[1].velocity is None
    assert states[1].effort is None

def test_js_from_dict_list_empty():
    dil = {"name": []}
    states = js_from_dict_list(dil)
    assert states == []

def test_js_from_dict_list_mismatched_lengths():
    dil = {
        "name": ["joint_1", "joint_2"],
        "position": [1.0],  # Mismatched length
    }
    with pytest.raises(AssertionError):
        js_from_dict_list(dil)

def test_impose_state():
    onto = JState(name="joint_1", position=1.0, velocity=None, effort=0.2)
    fromm = JState(name="joint_2", position=None, velocity=0.5, effort=None)
    
    imposed = impose_state(onto, fromm)
    assert imposed.name == "joint_2"  # Name is overridden
    assert imposed.position == 1.0    # Taken from `onto`
    assert imposed.velocity == 0.5   # Taken from `fromm`
    assert imposed.effort == 0.2     # Taken from `onto`

def test_js_diff():
    j1 = JState(name="joint_1", position=2.0, velocity=1.0, effort=0.5)
    j2 = JState(name="joint_1", position=1.0, velocity=1.5, effort=0.2)

    diff = js_diff(j1, j2)
    assert diff.name == "joint_1"
    assert diff.position == 1.0  # 2.0 - 1.0
    assert diff.velocity == -0.5  # 1.0 - 1.5
    assert diff.effort == 0.3  # 0.5 - 0.2

def test_js_diff_time():
    t1 = Time(5_000_000_000)
    t2 = Time(3_000_000_000)
    j1 = JState(name="joint_1", time=t1)
    j2 = JState(name="joint_1", time=t2)

    diff = js_diff(j1, j2)
    assert diff.time == t1-t2  # 5e9 - 3e9

def test_js_diff_unset_attributes():
    j1 = JState(name="joint_1", position=1.0)
    j2 = JState(name="joint_1", velocity=0.5)

    diff = js_diff(j1, j2)
    assert diff.position is None
    assert diff.velocity is None
    assert diff.effort == 0

def test_js_changed_no_change():
    j1 = JState(name="joint_1", position=2.0, velocity=1.0)
    j2 = JState(name="joint_1", position=2.0, velocity=1.0)
    delta = JState(name="joint_1", position=1.0, velocity=1.0)

    assert not js_changed(j1, j2, delta)

def test_js_changed_beyond_delta():
    j1 = JState(name="joint_1", position=3.0)
    j2 = JState(name="joint_1", position=1.0)
    delta = JState(name="joint_1", position=1.0)

    assert js_changed(j1, j2, delta)  # Change of 2 exceeds delta of 1

def test_js_changed_within_delta():
    j1 = JState(name="joint_1", position=2.5)
    j2 = JState(name="joint_1", position=2.0)
    delta = JState(name="joint_1", position=1.0)

    assert not js_changed(j1, j2, delta)  # Change of 0.5 is within delta of 1.0

def test_js_changed_with_none_values():
    j1 = JState(name="joint_1", position=2.0)
    j2 = JState(name="joint_1", position=None)
    delta = JState(name="joint_1", position=1.0)

    assert js_changed(j1, j2, delta)  # Change from 2.0 to None is considered a change

