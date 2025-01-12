import operator
from dataclasses import dataclass
from typing import Any, Callable, Dict, Iterable, List, Optional, Union, overload

from .joint_state import JState

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
    """Holds and applies functions to position, velocity and effort fields.

    If None, the indentity is used.
    """
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
    def __call__(self, other: JState) -> None: ...

    def __call__(self, other: Union["Shaper", JState]) -> Union["Shaper", None]:
        if isinstance(other, Shaper):
            return Shaper(
                position=eggify_shapers(other.position, self.position),
                velocity=eggify_shapers(other.velocity, self.velocity),
                effort=eggify_shapers(other.effort, self.effort),
            )
        elif isinstance(other, JState):
            apply_shaper(other, self)
            return 
        else:
            return NotImplemented

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
            shaper(states[ind])
