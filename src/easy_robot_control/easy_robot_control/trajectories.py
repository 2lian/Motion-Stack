from dataclasses import dataclass
import traceback
from typing import Callable, Dict, Optional, TypeVar, Union
import numpy as np
from numpy.typing import NDArray
import quaternion as qt
from rclpy.time import Time
from scipy.spatial import geometric_slerp
from tf2_ros import Duration


@dataclass
class Point:
    time: Time
    coord: Optional[NDArray] = None
    rot: Optional[qt.quaternion] = None


@dataclass
class PointDefined:
    time: Time
    coord: NDArray
    rot: qt.quaternion


def assessPointDefine(point: Point) -> PointDefined:
    assert point.coord is not None
    assert point.rot is not None
    return PointDefined(time=point.time, coord=point.coord, rot=point.rot)


TypeP = TypeVar("TypeP", float, np.ndarray, qt.quaternion)
# Am I too used to C++ ??? no that can't be
IFunGlob = Callable[[TypeP, float], TypeP]
IFunTime = Callable[[float, float], float]
# Ok maybe yes


def smooth(relative_end: float, t: float) -> float:
    """smoothes the interval [0, 1] to have a soft start and end
    (derivative is zero)
    """
    x = relative_end * (1 - np.cos(t * np.pi)) / 2
    return x


def linear(relative_end: TypeP, t: float) -> TypeP:
    # assert isinstance(relative_end, TypeP)
    if isinstance(relative_end, float) or isinstance(relative_end, np.ndarray):
        return relative_end * t
    elif isinstance(relative_end, qt.quaternion):
        intPart: int = np.floor(t)
        floatPart: float = t - intPart

        float_slerp: NDArray = geometric_slerp(
            start=qt.one,
            end=qt.as_float_array(relative_end),
            t=floatPart,  # only works with 0<=t<=1
        )

        int_slerp = relative_end**intPart
        combined = qt.as_quat_array(float_slerp) * int_slerp
        return combined
    else:
        return relative_end * t  # type: ignore


def triangle(relative_end: float, t: float) -> float:
    return float((-abs(t - 1 / 2) * 2 + 1) * relative_end)


def boomrang(relative_end: float, t: float) -> float:
    # x = (1 - np.cos(t * np.pi)) / 2
    z = np.sin(t * np.pi)
    return z * relative_end


def singo(relative_end: float, t: float) -> float:
    z = np.sin(1 / 2 * t * np.pi)
    return z * relative_end


def sincom(relative_end: float, t: float) -> float:
    return reverseFun(singo)(relative_end, t)


def prolongate(func: IFunTime) -> IFunTime:
    inv = reverseFun(func)
    def newfunc(r:float, t:float) -> float:
        intPart: int = np.floor(t)
        floatPart: float = t - intPart

        if intPart % 2 == 1:
            float_slerp = inv(
                r,
                floatPart,
            )
        else:
            float_slerp = func(
                r,
                floatPart,
            )

        int_slerp = intPart
        combined = (float_slerp) + int_slerp
        return combined * r
    return newfunc

def reverseFun(func: IFunTime) -> IFunTime:
    return lambda relative_end, t: -func(relative_end, 1 - t) + 1


temporal: Dict[str, IFunTime] = {
    "linear": linear,
    "triangle": triangle,
    "boomrang": lambda r,t: prolongate(singo)(r, triangle(r, t)),
    "cos": prolongate(sincom),
    "sin": prolongate(singo),
    "smooth": prolongate(smooth),
    "smooth2": prolongate(lambda x, y: smooth(x, smooth(x, y))),
}
spatial: Dict[str, IFunGlob] = {
    "linear": linear,
}


def get_interp(
    interp_str: str, start: Point, end: Point
) -> Callable[[Time], PointDefined]:
    assert interp_str in list(spatial.keys())
    assert interp_str in list(temporal.keys())
    if start.coord is None:
        start.coord = np.zeros(3, dtype=float)
    if start.rot is None:
        start.rot = qt.one.copy()

    definedStart: PointDefined = assessPointDefine(start)
    func = lambda t: globalInterpolator(
        definedStart,
        end,
        spatial[interp_str],
        temporal[interp_str],
        t,
    )
    return func


def globalInterpolator(
    start: PointDefined,
    end: Point,
    spatial_interp: IFunGlob,
    temporal_interp: IFunGlob,
    now: Time,
) -> PointDefined:

    if now < start.time:
        return start
    if now > end.time:
        saturated = end
        if saturated.coord is None:
            saturated.coord = start.coord
        if saturated.rot is None:
            saturated.rot = start.rot
        return assessPointDefine(saturated)

    elapsed: Duration = now - start.time
    total: Duration = end.time - start.time
    progress: float = elapsed.nanoseconds / total.nanoseconds
    assert 0 <= progress <= 1

    t = temporal_interp(1, progress)
    coord = coord_interp(start.coord, end.coord, spatial_interp, t)
    rot = quat_interp(start.rot, end.rot, spatial_interp, t)

    interPoint = PointDefined(
        time=now,
        coord=coord,
        rot=rot,
    )

    return interPoint


def time_interp(ifunc: IFunGlob, t: float):
    interp = ifunc(1, t)
    return interp


def coord_interp(start: NDArray, end: Optional[NDArray], ifunc: IFunGlob, t: float):
    if end is None:
        return start

    relative_end = end - start
    interp = ifunc(relative_end, t)
    return interp


def quat_interp(start: qt.quaternion, end: qt.quaternion, ifunc: IFunGlob, t: float):
    if end is None:
        return start

    relative_end = end / start
    interp = ifunc(relative_end, t)
    return interp


class Interpolator:
    def __init__(
        self,
        start: PointDefined,
        end: Point,
        spatial_interp: Union[None, IFunGlob, str] = None,
        temporal_interp: Union[None, IFunGlob, str] = None,
    ) -> None:
        self.start: PointDefined = start
        self.end: Point = end
        self.temporal_interp: IFunGlob
        self.spatial_interp: IFunGlob
        self.last: PointDefined

        if temporal_interp is None:
            self.temporal_interp = linear
        elif isinstance(temporal_interp, str):
            assert temporal_interp in spatial.keys()
            self.temporal_interp = temporal[temporal_interp]
        else:
            self.temporal_interp = temporal_interp

        if spatial_interp is None:
            self.spatial_interp = linear
        elif isinstance(spatial_interp, str):
            assert spatial_interp in spatial.keys()
            self.spatial_interp = spatial[spatial_interp]
        else:
            self.spatial_interp = spatial_interp

        self.last: PointDefined = self.compute(now=end.time)

    def expired(self, now: Time) -> bool:
        return self.end.time < now

    def early(self, now: Time) -> bool:
        return self.start.time > now

    def compute(self, now: Time) -> PointDefined:
        return globalInterpolator(
            self.start,
            self.end,
            self.spatial_interp,
            self.temporal_interp,
            now,
        )


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    col = 3
    row = (len(temporal.keys()) + col - 1) // col
    print(len(temporal.keys()), row, col)
    fig, axes = plt.subplots(
        row,
        col,
        figsize=(6 * col, 3 * row),
    )
    for idx, k in enumerate(temporal.keys()):
        f = lambda t: temporal[k](1, t)
        x = np.linspace(-1, 2, 1000, dtype=float)
        try:
            y = np.array([f(n) for n in x])
        except Exception as exception:
            print(f"FAILED {k}")
            m = f"Exception intercepted {traceback.format_exc()}"
            print(m)
            continue
        ax = axes[idx // col, idx % col]
        ax.plot(x, y)
        ax.grid()
        ax.set_title(f"{k}")
        rectangle = plt.Rectangle((0, 0), 1, 1, fill=None, edgecolor='black', linewidth=2)
        ax.add_patch(rectangle)
        print(f"done {k}.png")
    plt.savefig(f"shapers.png")
    plt.cla
    plt.clf
    plt.close
