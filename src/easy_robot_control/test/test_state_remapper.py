import itertools
import string
from copy import deepcopy
from typing import List

import numpy as np
import pytest

from easy_robot_control.utils.joint_state_util import JState, Time
from easy_robot_control.utils.state_remaper import (
    NameMap,
    Shaper,
    StateMap,
    StateRemapper,
    insert_angle_offset,
    reverse_dict,
)

np.random.seed(0)

empty = StateRemapper()
characters = string.ascii_uppercase + string.ascii_lowercase + string.digits + "_-"
length = 2
NAMES: List[str] = ["".join(n) for n in itertools.product(characters, repeat=length)]
prob = lambda x: x if x > 0.05 else None
STATES = [
    JState(
        name=n,
        time=Time(seconds=np.random.rand()),
        position=prob(np.random.rand()),
        velocity=prob(np.random.rand()),
        effort=prob(np.random.rand()),
    )
    for n in NAMES
]

fromm = [s.name for s in STATES]
to = fromm[::-1]
mdic: NameMap = dict(zip(fromm, to))
unmdic: NameMap = dict(zip(to, fromm))

STATES += [
    JState(
        name=f"joint-1-{k}",
        time=Time(seconds=np.random.rand()),
        position=prob(np.random.rand()),
        velocity=prob(np.random.rand()),
        effort=prob(np.random.rand()),
    )
    for k in range(10)
]

NAMES = [s.name for s in STATES]


def test_nothing_does_nothing():
    states = deepcopy(STATES)
    mapped = deepcopy(states)
    empty.map(mapped)
    unmapped = deepcopy(states)
    empty.unmap(unmapped)
    assert mapped == states
    assert unmapped == states
    mapped[0].name = "prout"
    assert mapped != states


def test_reversedic():
    assert reverse_dict(mdic) == unmdic
    assert reverse_dict(unmdic) == mdic


def test_remap_name():
    states = deepcopy(STATES)

    my_mapper = StateRemapper(name_map=mdic, unname_map=unmdic)
    mapped = deepcopy(states)
    my_mapper.map(mapped)
    unmapped = deepcopy(mapped)
    my_mapper.unmap(unmapped)
    assert states == unmapped
    assert mapped != states
    assert [s.name for s in mapped if s.name in to] == to


def test_remap_nameauto():
    states = deepcopy(STATES)

    my_mapper = StateRemapper(name_map=mdic)
    mapped = deepcopy(states)
    my_mapper.map(mapped)
    unmapped = deepcopy(mapped)
    my_mapper.unmap(unmapped)
    assert states == unmapped
    assert mapped != states
    assert [s.name for s in mapped if s.name in to] == to


@pytest.mark.parametrize(
    "a, b",
    [
        (None, None),
        (1, 0),
        (2, 1),
        (-24.2, 0.01),
    ],
)
def test_op(a, b):
    states = deepcopy(STATES)
    if a is not None:
        f = lambda x: a * x + b if x is not None else None
        unf = lambda x: (x - b) / a if x is not None else None
    else:
        f = None
        unf = None
    shaper = Shaper(position=f, velocity=f, effort=f)
    sdic: StateMap = dict(zip(NAMES, [shaper] * len(NAMES)))
    unshaper = Shaper(position=unf, velocity=unf, effort=unf)
    unsdic: StateMap = dict(zip(NAMES, [unshaper] * len(NAMES)))

    my_mapper = StateRemapper(
        state_map=sdic,
        unstate_map=unsdic,
    )
    mapped = deepcopy(states)
    my_mapper.map(mapped)
    unmapped = deepcopy(mapped)
    my_mapper.unmap(unmapped)

    # pytest.approx([f(s.position) for s in states], [s.position for s in mapped])
    if f is None:
        f = lambda x: x
        unf = lambda x: x
    assert [s.position for s in mapped] == pytest.approx([f(s.position) for s in states])
    assert [s.velocity for s in mapped] == pytest.approx([f(s.velocity) for s in states])
    assert [s.effort for s in mapped] == pytest.approx([f(s.effort) for s in states])
    assert [s.position for s in states] == pytest.approx([s.position for s in unmapped])
    assert [s.velocity for s in states] == pytest.approx([s.velocity for s in unmapped])
    assert [s.effort for s in states] == pytest.approx([s.effort for s in unmapped])


@pytest.mark.parametrize(
    "a, b",
    [
        (None, None),
        (2, 1),
    ],
)
def test_all(a, b):
    states = deepcopy(STATES)
    if a is not None:
        f = lambda x: a * x + b if x is not None else None
        unf = lambda x: (x - b) / a if x is not None else None
    else:
        f = None
        unf = None
    shaper = Shaper(position=f, velocity=f, effort=f)
    sdic: StateMap = dict(zip(NAMES, [shaper] * len(NAMES)))
    unshaper = Shaper(position=unf, velocity=unf, effort=unf)
    unsdic: StateMap = dict(zip(NAMES, [unshaper] * len(NAMES)))

    my_mapper = StateRemapper(
        name_map=mdic,
        state_map=sdic,
        unstate_map=unsdic,
    )
    mapped = deepcopy(states)
    my_mapper.map(mapped)
    unmapped = deepcopy(mapped)
    my_mapper.unmap(unmapped)

    # pytest.approx([f(s.position) for s in states], [s.position for s in mapped])
    if f is None:
        f = lambda x: x
        unf = lambda x: x
    assert [s.position for s in mapped] == pytest.approx([f(s.position) for s in states])
    assert [s.velocity for s in mapped] == pytest.approx([f(s.velocity) for s in states])
    assert [s.effort for s in mapped] == pytest.approx([f(s.effort) for s in states])
    assert [s.position for s in states] == pytest.approx([s.position for s in unmapped])
    assert [s.velocity for s in states] == pytest.approx([s.velocity for s in unmapped])
    assert [s.effort for s in states] == pytest.approx([s.effort for s in unmapped])
    assert [s.name for s in states] == ([s.name for s in unmapped])
    assert [s.time for s in states] == ([s.time for s in unmapped])


@pytest.mark.parametrize(
    "a, b",
    [
        (None, None),
        (2, 1),
    ],
)
def test_offset_adder(a, b):
    states = deepcopy(STATES)
    if a is not None:
        f = lambda x: a * x + b if x is not None else None
        unf = lambda x: (x - b) / a if x is not None else None
    else:
        f = None
        unf = None
    shaper = Shaper(position=f, velocity=f, effort=f)
    sdic: StateMap = dict(zip(fromm, [shaper] * len(fromm)))
    unshaper = Shaper(position=unf, velocity=unf, effort=unf)
    unsdic: StateMap = dict(zip(fromm, [unshaper] * len(fromm)))

    my_old_mapper = StateRemapper(
        name_map=mdic,
        state_map=sdic,
        unstate_map=unsdic,
    )
    my_mapper = StateRemapper(
        name_map=mdic,
        state_map=sdic,
        unstate_map=unsdic,
    )
    insert_angle_offset(my_mapper, my_mapper, offsets={"AA": 2})
    j1 = JState(name="AA", position=0)
    j2 = JState(name="AA", position=0)
    my_mapper.map([j1])
    my_old_mapper.unmap([j1])
    assert j1.position == pytest.approx(j2.position + 2)
    mapped = deepcopy(states)
    my_mapper.map(mapped)
    unmapped = deepcopy(mapped)
    my_mapper.unmap(unmapped)

    # pytest.approx([f(s.position) for s in states], [s.position for s in mapped])
    if f is None:
        f = lambda x: x
        unf = lambda x: x
    assert [s.position for s in states] == pytest.approx([s.position for s in unmapped])
    assert [s.velocity for s in states] == pytest.approx([s.velocity for s in unmapped])
    assert [s.effort for s in states] == pytest.approx([s.effort for s in unmapped])
    assert [s.name for s in states] == ([s.name for s in unmapped])
    assert [s.time for s in states] == ([s.time for s in unmapped])
