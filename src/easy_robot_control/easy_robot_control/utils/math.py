import numpy as np
import quaternion as qt


class Quaternion(qt.quaternion): ...


Quaternion = qt.quaternion


def qt_normalize(q: Quaternion):
    if q.w < 0:
        q *= -1
    return q / np.linalg.norm(qt.as_float_array(q))


def qt_repr(q: Quaternion) -> str:
    return str(qt.as_float_array(q))
