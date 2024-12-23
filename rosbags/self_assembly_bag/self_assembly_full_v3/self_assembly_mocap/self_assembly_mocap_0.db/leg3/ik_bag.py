from dataclasses import dataclass
from os import posix_spawn
from typing import Any, Final, List, Literal
import quaternion as qt

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from numpy._typing import NDArray

import util
from util import parse_date

float_formatter = "{:_.2f}".format
np.set_printoptions(formatter={"float_kind": float_formatter})

OUTPUT_FILE = "ik_op.pdf"
START_TIME = 15
END_TIME = 40
FILE_FK = "./bag/manuik/manuik_0/leg4/tip_pos.csv"
FILE_IK = "./bag/manuik/manuik_0/leg4/set_ik_target.csv"
start_epoch: int = parse_date("2024/12/16 17:59:42.181511184")
offset_fk: int = -int(0.3 * 1e9)

IArr = NDArray[np.unsignedinteger]
FArr = NDArray[np.float64]


@dataclass
class Poses:
    time: IArr
    xyz: FArr
    rot: FArr


def bag_to_str_array(file) -> NDArray[np.string_]:
    data = np.genfromtxt(
        file,
        delimiter=",",
        skip_header=1,
        dtype=str,
    )
    return data


def parse_transform_bag(file: str) -> Poses:
    data = bag_to_str_array(file)
    rows = data.shape[0]
    time: IArr = np.empty(rows, dtype=int)
    invalid: List[int] = []
    for i in range(rows):
        try:
            time[i] = util.parse_date(data[i, 0])
        except ValueError:
            invalid.append(i)
    mask = np.ones(data.shape[0], dtype=bool)  # Start with all True
    mask[invalid] = False         # Set unwanted indices to False
    time = time[mask]
    data = data[mask]

    time -= start_epoch
    eliminatiation = np.ones(shape=(data.shape[0]), dtype=bool)
    eliminatiation[:-1] = time[:-1] < time[1:]

    time = time[eliminatiation]
    data = data[eliminatiation]

    xyz = data[:, [1, 2, 3]].astype(float)
    rot = data[:, [4, 5, 6, 7]]
    return Poses(time=time, xyz=xyz.astype(float), rot=rot.astype(float))


def load_ik_targets() -> Poses:
    return parse_transform_bag(FILE_IK)


def load_fk() -> Poses:
    p = parse_transform_bag(FILE_FK)
    p.time += offset_fk
    return p


def make_arr_plot(t: IArr, data: FArr, ax: Axes, **kwargs):
    assert len(t.shape) == 1
    assert len(data.shape) == 1
    ax.plot(t / 1e9, data, **kwargs)


def make_xyz_plot(pos_fk: Poses, pos_ik: Poses, axes: List[Axes]):
    assert len(axes) == 3
    for n, ax, ind in zip(["x", "y", "z"], axes, range(len(axes))):
        make_arr_plot(pos_fk.time, pos_fk.xyz[:, ind], ax, color=util.default_colors[0])
        make_arr_plot(
            pos_ik.time,
            pos_ik.xyz[:, ind],
            ax,
            color="black",
            # markevery=2,
            linestyle=" ",
            marker=".",
            markersize=1,
        )
        ax.set_ylabel(f"{n} (mm)")


def make_euler_plot(pos_fk, pos_ik, axes):
    print(f"{len(axes)=}")
    assert len(axes) == 3
    eulerized_fk = np.rad2deg(qt.as_euler_angles(qt.from_float_array(pos_fk.rot)))
    eulerized_ik = np.rad2deg(qt.as_euler_angles(qt.from_float_array(pos_ik.rot)))
    for n, ax, ind in zip(["roll", "pitch", "yaw"], axes, range(len(axes))):
        make_arr_plot(pos_fk.time, eulerized_fk[:, ind], ax, color=util.default_colors[0])
        make_arr_plot(
            pos_ik.time,
            eulerized_ik[:, ind],
            ax,
            color="black",
            # markevery=2,
            linestyle=" ",
            marker=".",
            markersize=1,
        )
        ax.set_ylabel(f"{n} (deg)")

def make_quat_plot(pos_fk, pos_ik, axes):
    assert len(axes) == 4
    for n, ax, ind in zip(["w", "x", "y", "z"], axes, range(len(axes))):
        make_arr_plot(pos_fk.time, pos_fk.rot[:, ind], ax, color=util.default_colors[0])
        make_arr_plot(
            pos_ik.time,
            pos_ik.rot[:, ind],
            ax,
            color="black",
            # markevery=2,
            linestyle=" ",
            marker=".",
            markersize=1,
        )
        ax.set_ylabel(f"q.{n} (rad)")


def main():
    ratios = [1, 1, 1, 1, 1, 1, 1]
    axes: List[Axes]
    fig, axes = plt.subplots(  # type: ignore
        len(ratios),
        1,
        figsize=(5.5 * 1, 2 * len(ratios)),
        # sharex=True,
        gridspec_kw={
            "height_ratios": ratios,
            # "hspace": 0.15,
        },
    )
    pos_fk: Poses = load_fk()
    pos_ik: Poses = load_ik_targets()
    make_xyz_plot(pos_fk, pos_ik, axes[:3])
    make_quat_plot(pos_fk, pos_ik, axes[3:8])

    for i, a in enumerate(axes):
        a.spines["right"].set_visible(False)
        a.spines["top"].set_visible(False)
        # a.spines["bottom"].set_visible(False)
        a.set_xlim([START_TIME, END_TIME])
    plt.tight_layout()
    print(OUTPUT_FILE)
    plt.savefig(OUTPUT_FILE)


if __name__ == "__main__":
    main()
