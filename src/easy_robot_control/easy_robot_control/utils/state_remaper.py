import dataclasses
import operator
from dataclasses import dataclass
from typing import Any, Callable, Dict, Iterable, List, Optional, Union, overload

from easy_robot_control.utils.joint_state_util import JState

SubShaper = Optional[Callable[[float], float]]


def operate_sub_shapers(
    shaper1: SubShaper, shaper2: SubShaper, op: Callable[[float, float], float]
) -> SubShaper:
    if shaper1 and shaper2:
        return lambda x: op(shaper1(x), shaper2(x))
    return shaper1 or shaper2


def eggify_shapers(inner: SubShaper, outer: SubShaper) -> SubShaper:
    if inner and outer:
        return lambda x: outer(inner(x))
    return inner or outer


@dataclass
class Shaper:
    position: SubShaper = None
    velocity: SubShaper = None
    effort: SubShaper = None

    def _combine(self, other: "Shaper", op: Callable) -> "Shaper":
        return Shaper(
            position=operate_sub_shapers(self.position, other.position, op),
            velocity=operate_sub_shapers(self.velocity, other.velocity, op),
            effort=operate_sub_shapers(self.effort, other.effort, op),
        )

    # Arithmetic operations
    def __add__(self, other: "Shaper") -> "Shaper":
        return self._combine(other, operator.add)

    def __sub__(self, other: "Shaper") -> "Shaper":
        return self._combine(other, operator.sub)

    def __mul__(self, other: "Shaper") -> "Shaper":
        return self._combine(other, operator.mul)

    def __truediv__(self, other: "Shaper") -> "Shaper":
        return NotImplemented
        # return self._combine(other, operator.truediv)

    @overload
    def __call__(self, other: "Shaper") -> "Shaper": ...

    @overload
    def __call__(self, other: JState) -> JState: ...

    def __call__(self, other: Union["Shaper", JState]) -> Union["Shaper", JState]:
        if isinstance(other, Shaper):
            return Shaper(
                position=eggify_shapers(other.position, self.position),
                velocity=eggify_shapers(other.velocity, self.velocity),
                effort=eggify_shapers(other.effort, self.effort),
            )
        elif isinstance(other, JState):
            out = dataclasses.replace(other)
            apply_shaper(out, self)
            return out
        else:
            return NotImplemented

    def __repr__(self):
        return (
            f"Shaper(position={self.position}, "
            f"velocity={self.velocity}, "
            f"effort={self.effort})"
        )


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
        # print(f"{attr}: {sub_state} -> {sub_shaper(sub_state)}")


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

    def __call__(self, inner: "StateRemapper") -> "StateRemapper":
        """ok maybe not today"""
        return NotImplemented


def insert_angle_offset(
    mapper_in: StateRemapper, mapper_out: StateRemapper, offsets: Dict[str, float]
) -> None:
    """Applies an position offsets to a StateRemapper.
    the state_map adds the offset, then the unstate_map substracts it.

    changing sub_shapers from mapper_in will overwrite sub_shapers in mapper_out.
        mapper_out = mapper_in, may lead to undefined behavior.
        Any function shared between in/out may lead to undefined behavior. please don't.
        Use deepcopy() to avoid issues.

    this is a very rough function. feel free to improve

    Args:
        mapper_in: original function map to which offset should be added
        mapper_out: changes will be stored here
    """
    # add the offset before executing the original mapping
    # for k, orig_func in mapper_in.state_map.items():
    for k, off_val in offsets.items():
        orig_func = mapper_in.state_map.get(k)
        if orig_func is None:
            orig_func = Shaper()
        # self.parent.pwarn(off_val)
        off_func: SubShaper
        if off_val is not None:
            off_func = lambda x, off=off_val: x + off
        else:
            off_func = None
        off_shaper = Shaper(position=off_func)
        mapper_out.state_map[k] = orig_func(off_shaper)

    # substracts the offset after executing the original unmapping
    # for k, orig_func in mapper_in.unstate_map.items():
    for k, off_val in offsets.items():
        orig_func = mapper_in.unstate_map.get(k)
        if orig_func is None:
            orig_func = Shaper()
        # self.parent.pwarn(off_val)
        off_func: SubShaper
        if off_val is not None:
            off_func = lambda x, off=off_val: x - off
        else:
            off_func = None
        off_shaper = Shaper(position=off_func)
        mapper_out.unstate_map[k] = off_shaper(orig_func)


empty_remapper = StateRemapper({}, {}, {}, {})
