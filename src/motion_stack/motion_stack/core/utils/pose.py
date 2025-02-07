import dataclasses
from dataclasses import dataclass
from typing import Generic, NamedTuple, Tuple, TypeVar

import numpy as np

from .math import (
    Barr,
    Farr,
    Flo3,
    Flo4,
    Quaternion,
    angle_with_unit_quaternion,
    qt,
    qt_repr,
)
from .time import Time

# @dataclass(frozen=True)
# class PoseUndefined:
#     time: Optional[Time] = None
#     xyz: Optional[Flo3] = None
#     quat: Optional[Quaternion] = None
#

T1 = TypeVar("T1")
T2 = TypeVar("T2")


class XyzQuat(NamedTuple, Generic[T1, T2]):
    """Tuple containing spatial and rotation data"""

    xyz: T1
    quat: T2


@dataclass
class Pose:
    time: Time
    xyz: Flo3
    quat: Quaternion

    def __sub__(self, other: "Pose") -> "Pose":
        return Pose(
            time=self.time - other.time,
            xyz=self.xyz - other.xyz,
            quat=self.quat / other.quat,
        )

    def __str__(self) -> str:
        return f"Pose(time={self.time:_}, xyz={self.xyz}, quat={qt.as_float_array(self.quat)})"

    def close2zero(self, atol: Tuple[float, float] = (1, np.deg2rad(1))) -> bool:
        a = np.linalg.norm(self.xyz) < atol[0]
        b = angle_with_unit_quaternion(self.quat) < atol[1]
        return bool(a and b)

    def copy(self):
        return dataclasses.replace(self)
