from typing import Any

import nptyping as nt
import numpy as np
import quaternion as qt
from nptyping import NDArray, Shape

Flo3 = NDArray[Shape["3"], nt.Floating]
Flo4 = NDArray[Shape["4"], nt.Floating]
Farr = NDArray[Any, nt.Float]
Barr = NDArray[Any, nt.Bool]


class Quaternion(qt.quaternion): ...


Quaternion = qt.quaternion


def qt_normalize(q: Quaternion):
    if q.w < 0:
        q *= -1
    return q / np.linalg.norm(qt.as_float_array(q))


def qt_repr(q: Quaternion) -> str:
    return str(qt.as_float_array(q))


assert qt.one.w == 1


def angle_with_unit_quaternion(q):
    qt_normalize(q)
    return 2 * np.arccos(q.w)


def patch_numpy_display_light(floating_points: int = 2):
    float_formatter = ("{:." + str(floating_points) + "f}").format
    np.set_printoptions(formatter={"float_kind": float_formatter})
