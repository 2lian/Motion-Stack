"""
Vectorized functions to clamp onto an hypershpere

Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

from typing import Any, Tuple

import nptyping as nt
import numpy as np
import quaternion as qt
from nptyping import NDArray, Shape

from .math import Quaternion, qt_normalize

Farr = NDArray[Any, nt.Float]
Barr = NDArray[Any, nt.Bool]
Flo3 = NDArray[Shape["3"], nt.Floating]

SAMPLING_STEP = 0.01  # will sample every 0.01 for a unit hypersphere
# if you use the radii, it is equivalent sampling every 0.01 * radii
ORD = 2  # Order of the norm for clamping


def clamp_to_unit_hs(
    start: NDArray[Shape["N"], nt.Float],
    end: NDArray[Shape["N"], nt.Float],
    sampling_step: float = SAMPLING_STEP,
) -> NDArray[Shape["N"], nt.Float]:
    """Finds the farthest point on the segment that is inside the unit hypersphere."""
    assert start.shape == end.shape
    assert len(start.shape) == 1
    assert start.shape[0] > 0
    dimensionality: int = start.shape[0]
    sample_count: int = int(np.linalg.norm(end - start) / sampling_step) + 1

    t = np.linspace(0, 1, sample_count, endpoint=True).reshape(-1, 1)
    interp = end * t + start * (1 - t)
    inside_hyper = np.linalg.norm(interp, ord=ORD, axis=1) < 1
    assert inside_hyper.shape[0] == sample_count
    selection = interp[inside_hyper]
    if selection.shape[0] == 0:
        return start
    elif selection.shape[0] == sample_count:
        return end
    furthest = selection[-1, :]
    assert furthest.shape[0] == dimensionality
    return furthest


def clamp_to_sqewed_hs(
    center: NDArray[Shape["N"], nt.Floating],
    start: NDArray[Shape["N"], nt.Floating],
    end: NDArray[Shape["N"], nt.Floating],
    radii: NDArray[Shape["N"], nt.Floating],
) -> NDArray[Shape["N"], nt.Floating]:
    """Finds the farthest point on the segment that is inside the sqewed hypersphere.

    radii of the hypersphere in each dimensions is computed by streching the space in each dimension, then computing relative to the unit hypersphere, then unstreching.
    """
    assert len(center.shape) == 1
    assert center.shape == start.shape == end.shape == radii.shape
    start_n = (start - center) / radii
    end_n = (end - center) / radii
    clamped_n = clamp_to_unit_hs(start=start_n, end=end_n)
    clamped = clamped_n * radii + center
    assert end.shape == clamped.shape
    return clamped


xyz_slots = [0, 1, 2]
quat_slots = [3, 4, 5, 6]
dims = len(xyz_slots) + len(quat_slots)


def fuse_xyz_quat(xyz: Flo3, quat: Quaternion) -> NDArray[Shape["7"], nt.Floating]:
    quat = qt_normalize(quat)
    fused = np.empty(dims, dtype=float)
    fused[xyz_slots] = xyz
    fused[quat_slots] = qt.as_float_array(quat)
    assert fused.shape == (dims,)
    return fused


def unfuse_xyz_quat(arr: NDArray[Shape["7"], nt.Floating]) -> Tuple[Flo3, Quaternion]:
    q = qt.from_float_array(arr[quat_slots])
    q = qt_normalize(q)
    return arr[xyz_slots], q


def clamp_xyz_quat(
    center: Tuple[Flo3, Quaternion],
    start: Tuple[Flo3, Quaternion],
    end: Tuple[Flo3, Quaternion],
    radii: Tuple[float, float],
) -> Tuple[Flo3, Quaternion]:
    """wrapper for clamp_to_sqewed_hs specialized in one 3D coordinate + one quaternion.

    The math for the quaternion is wrong (lerp instead of slerp). So:
    Center and start quat should not be opposite from each-other.
    Precision goes down if they are far appart.

    Args:
        center:  center from which not to diverge
        start: start point of the interpolation
        end: end point of the interpolation
        radii: allowed divergence for coord and quat

    Returns:

    """
    xyz_radius, quat_radius = radii
    center_xyz, center_quat = center
    start_xyz, start_quat = start
    end_xyz, end_quat = end

    # quat_oneify = 1 / start_quat
    # quat_oneify = 1
    # for q in [center_quat, start_quat, end_quat]:
    # q = q * quat_oneify

    radii_arr = np.array(
        [xyz_radius] * len(xyz_slots) + [quat_radius] * len(quat_slots), dtype=float
    )
    fused_center = fuse_xyz_quat(center_xyz, center_quat)
    fused_end = fuse_xyz_quat(end_xyz, end_quat)
    fused_start = fuse_xyz_quat(start_xyz, start_quat)

    fused_clamp = clamp_to_sqewed_hs(fused_center, fused_start, fused_end, radii_arr)
    fuse_xyz, fuse_quat = unfuse_xyz_quat(arr=fused_clamp)

    # fuse_quat = fuse_quat / quat_oneify

    assert fuse_xyz.shape == (3,)
    assert qt.as_float_array(fuse_quat).shape == (4,)

    return fuse_xyz, fuse_quat
