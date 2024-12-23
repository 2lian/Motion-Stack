from copy import deepcopy
from typing import List

import matplotlib.pyplot as plt
import numpy as np
import quaternion as qt

import ik_bag as m

m.START_TIME = 240
m.END_TIME = 550
m.FILE_FK = "mocap_pose.csv"
m.FILE_IK = "tip_pos.csv"
m.OUTPUT_FILE = "shami.pdf"
m.start_epoch = m.parse_date("2024/12/19 18:45:26.858841848")

offset = np.array([0, -25, 50])


class Quaternion(qt.quaternion): ...


Quaternion = qt.quaternion


def qt_normalize(q: Quaternion):
    if len(q.shape) == 1:
        q = qt.as_float_array(q)
        assert q.shape[1] == 4
        to_reverse = q[:, 0] < 0
        q[to_reverse] = -1 * q[to_reverse]
        nor = np.linalg.norm(q, axis=1).reshape(-1, 1)
        res = q / nor
        assert res.shape[1] == 4
        return qt.from_float_array(res)

    else:
        if q.w < 0:
            q *= -1
        return q / np.linalg.norm(qt.as_float_array(q))


def qt_repr(q: Quaternion) -> str:
    if isinstance(q, qt.quaternion):
        return str(qt.as_float_array(q))
    else:
        return str(q)


def align_with(pos: m.Poses, ref: m.Poses) -> m.Poses:
    pos = deepcopy(pos)
    ref = deepcopy(ref)
    pos.xyz = (qt.rotate_vectors(1 / pos.rot[0], pos.xyz - pos.xyz[0])) + ref.xyz[0]
    pos.rot = pos.rot / pos.rot[0] * ref.rot[0]
    return pos


def main():
    ratios = [1, 1, 1, 1, 1, 1, 1]
    axes: List[plt.Axes]
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
    pos_fk: m.Poses = m.load_fk()
    pos_ik: m.Poses = m.load_ik_targets()
    pos_fk.xyz = pos_fk.xyz[:, :] * 1000
    # pos_fk.time = pos_fk.time[::10]
    # pos_fk.rot = pos_fk.rot[::10, :]

    pos_ik.rot = qt_normalize(qt.from_float_array(pos_ik.rot))
    pos_fk.rot = qt_normalize(qt.from_float_array(pos_fk.rot))
    pos_fk = align_with(pos_fk, pos_ik)
    pos_ik = align_with(pos_ik, pos_ik)
    pos_fk.xyz = pos_fk.xyz + offset

    print(qt_repr(pos_ik.rot))

    pos_ik.rot = qt_normalize(pos_ik.rot)
    pos_fk.rot = qt_normalize(pos_fk.rot)

    pos_ik.rot = qt.as_float_array(pos_ik.rot)
    pos_fk.rot = qt.as_float_array(pos_fk.rot)
    print(pos_fk.rot.shape)
    print(pos_ik.rot.shape)
    m.make_xyz_plot(pos_fk, pos_ik, axes[:3])
    m.make_euler_plot(pos_fk, pos_ik, axes[3:6])
    # m.make_quat_plot(pos_fk, pos_ik, axes[3:7])

    for i, a in enumerate(axes):
        a.spines["right"].set_visible(False)
        a.spines["top"].set_visible(False)
        # a.spines["bottom"].set_visible(False)
        a.set_xlim([m.START_TIME, m.END_TIME])
    plt.tight_layout()
    print(m.OUTPUT_FILE)
    plt.savefig(m.OUTPUT_FILE)


if __name__ == "__main__":
    print("hey")
    main()
