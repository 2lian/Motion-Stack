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
