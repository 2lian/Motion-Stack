from dataclasses import dataclass
from os import environ
from typing import Any, Callable, Dict, Iterable, List, Optional

import numpy as np
import pytest
from python_package_include.joint_state_util import JState

SubShaper = Optional[Callable[[float], float]]


@dataclass
class Shaper:
    position: SubShaper = None
    velocity: SubShaper = None
    effort: SubShaper = None


URDFJointName = str
NameMap = Dict[URDFJointName, URDFJointName]
StateMap = Dict[URDFJointName, Shaper]


def reverse_dict(d: Dict) -> Dict:
    return dict(zip(d.values(), d.keys()))


def remap_names(states: List[JState], mapping: NameMap):
    names_in: List[Optional[str]] = list(map(lambda s: s.name, states))
    shared = set(names_in) & set(mapping.keys())
    for name in shared:
        if name is None:
            continue
        ind = names_in.index(name)
        new_name = mapping.get(name)
        if new_name is not None:
            states[ind].name = new_name


def apply_shaper(state: JState, shaper: Shaper):
    for attr in shaper.__annotations__.keys():
        sub_shaper: SubShaper = getattr(shaper, attr, None)
        if sub_shaper is None:
            continue
        sub_state: Optional[float] = getattr(state, attr, None)
        if sub_state is None:
            continue
        setattr(state, attr, sub_shaper(sub_state))


def shape_states(states: List[JState], mapping: StateMap):
    names_in: List[Optional[str]] = list(map(lambda s: s.name, states))
    shared = set(names_in) & set(mapping.keys())
    for name in shared:
        if name is None:
            continue
        ind = names_in.index(name)
        shaper = mapping.get(name)
        if shaper is not None:
            apply_shaper(states[ind], shaper)


class StateRemapper:
    def __init__(
        self,
        name_map: NameMap = {},
        unname_map: Optional[NameMap] = {},
        state_map: StateMap = {},
        unstate_map: StateMap = {},
    ) -> None:
        self.name_map: NameMap = name_map.copy()
        self.unname_map: NameMap = (
            reverse_dict(name_map) if unname_map is None else unname_map
        ).copy()
        self.state_map: StateMap = state_map.copy()
        self.unstate_map: StateMap = unstate_map.copy()

    def namify(self, states: List[JState]):
        remap_names(states, self.name_map)

    def unnamify(self, states: List[JState]):
        remap_names(states, self.unname_map)

    def shapify(self, states: List[JState]):
        shape_states(states, self.state_map)

    def unshapify(self, states: List[JState]):
        shape_states(states, self.unstate_map)

    def map(self, states):
        """mapping used before sending"""
        self.shapify(states)
        self.namify(states)

    def unmap(self, states):
        """mapping used before receiving"""
        self.unnamify(states)
        self.unshapify(states)

    def simplify(self, names_to_keep: Iterable[str]) -> "StateRemapper":
        """Eliminates (not in place) all entries whose keys are not in names_to_keep.
        Returns:
            new StateRemapper
        """
        # return self
        maps: List[Dict[str, Any]] = [
            self.name_map,
            self.unname_map,
            self.state_map,
            self.unstate_map,
        ]
        new: List[Dict[str, Any]] = [{}, {}, {}, {}]
        for m, mnew in zip(maps, new):
            for k, v in m.items():
                if k in names_to_keep:
                    mnew[k] = v
        return StateRemapper(*new)


empty_remapper = StateRemapper({}, {}, {}, {})
