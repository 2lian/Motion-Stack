from dataclasses import dataclass
from typing import Optional
from warnings import WarningMessage

import numpy as np

from .math import Barr, Farr, Flo3, Flo4, Quaternion, qt, qt_repr
from .time import Time


# @dataclass(frozen=True)
# class PoseUndefined:
#     time: Optional[Time] = None
#     xyz: Optional[Flo3] = None
#     quat: Optional[Quaternion] = None
#
@dataclass()
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

    def close2zero(self, atol=(1, 0.01)) -> bool:
        a = np.allclose(self.xyz, 0, atol=atol[0])
        b = qt.allclose(self.quat, qt.one, atol=atol[1])
        return bool(a and b)
