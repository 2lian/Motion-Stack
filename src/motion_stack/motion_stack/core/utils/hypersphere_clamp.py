"""
Vectorized functions to clamp onto an hypershpere

Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

from typing import Any, Iterable, List, Tuple, Union, overload

import nptyping as nt
import numpy as np
import quaternion as qt
from nptyping import NDArray, Shape

from motion_stack.core.utils.pose import XyzQuat

from .math import Flo3, Flo4, Quaternion, qt_normalize

#: will sample every 0.01 for a unit hypersphere
#: if you use the radii, it is equivalent sampling every 0.01 * radii
SAMPLING_STEP = 0.01
ORD = np.inf  #: Order of the norm for clamping


def clamp_to_unit_hs(
    start: NDArray[Shape["N"], nt.Float],
    end: NDArray[Shape["N"], nt.Float],
    sampling_step: float = SAMPLING_STEP,
    norm_ord=ORD,
) -> NDArray[Shape["N"], nt.Float]:
    """Finds the farthest point on the segment that is inside the unit hypersphere.

    Args:
        start: start of the segment
        end: end of the segment
        sampling_step: distance between each sample.
        norm_ord (int or numpy inf): order of the distance/norm used to create the hypersphere

    Returns:
        Farthest point on the segment that is inside the unit hypersphere

    """
    assert start.shape == end.shape
    assert len(start.shape) == 1
    assert start.shape[0] > 0
    dimensionality: int = start.shape[0]
    sample_count: int = int(np.linalg.norm(end - start) / sampling_step) + 1

    t = np.linspace(0, 1, sample_count, endpoint=True).reshape(-1, 1)
    interp = end * t + start * (1 - t)
    dist = np.linalg.norm(interp, ord=norm_ord, axis=1)
    inside_hyper = dist <= 1
    assert inside_hyper.shape[0] == sample_count
    selection = interp[inside_hyper]
    if selection.shape[0] == 0:  # no solution
        return np.full_like(start, fill_value=np.nan)
        print(f"no sol: {closest_ind=}")
        closest_ind = np.argmin(dist)
        closest = interp[closest_ind]
        # restarts from the origin, towards the closest point on the segment
        # return start
        # return closest
        res = clamp_to_unit_hs(
            start=np.zeros_like(start),
            end=closest,
            norm_ord=norm_ord,
        )
        return res
    elif selection.shape[0] == sample_count:  # segment in the sphere
        return end
    furthest = selection[-1, :]
    assert furthest.shape[0] == dimensionality
    return furthest


def clamp_to_sqewed_hs(
    center: NDArray[Shape["N"], nt.Floating],
    start: NDArray[Shape["N"], nt.Floating],
    end: NDArray[Shape["N"], nt.Floating],
    radii: NDArray[Shape["N"], nt.Floating],
    norm_ord=ORD,
) -> NDArray[Shape["N"], nt.Floating]:
    """Finds the farthest point on the segment that is inside the sqewed hypersphere.

    radii of the hypersphere in each dimensions is computed by streching the space in each dimension, then computing relative to the unit hypersphere, then unstreching.
    """
    assert len(center.shape) == 1
    assert center.shape == start.shape == end.shape == radii.shape
    start_n = (start - center) / radii
    end_n = (end - center) / radii
    clamped_n = clamp_to_unit_hs(start=start_n, end=end_n, norm_ord=norm_ord)
    clamped = clamped_n * radii + center
    assert end.shape == clamped.shape
    return clamped


XYZ_SLOT = np.array([0, 1, 2], dtype=int)
QUAT_SLOT = np.array([3, 4, 5, 6], dtype=int)
DIMS = len(XYZ_SLOT) + len(QUAT_SLOT)


@overload
def fuse_xyz_quat(
    pose: XyzQuat[Flo3, Quaternion]
) -> NDArray[Shape["7"], nt.Floating]: ...


@overload
def fuse_xyz_quat(
    pose: List[XyzQuat[Flo3, Quaternion]]
) -> NDArray[Shape["7 n"], nt.Floating]: ...


def fuse_xyz_quat(
    pose: Union[XyzQuat[Flo3, Quaternion], List[XyzQuat[Flo3, Quaternion]]]
) -> NDArray[Shape["7 n"], nt.Floating]:
    """
    Fuses `XyzQuat` objects into a flat numpy array.

    - If input is a single `XyzQuat`, returns a (7,) fused array.
    - If input is a list, returns a flat (n*7,) fused array.

    Args:
        pose: A single or list of `XyzQuat` objects containing position and orientation.

    Returns:
        The fused representation(s) as (7,) for a single input or (n*7,) for batched input.
    """
    if not isinstance(pose, List):
        xyz, quat = pose
        quat = qt_normalize(quat)
        fused = np.empty(DIMS, dtype=float)
        fused[XYZ_SLOT] = xyz
        fused[QUAT_SLOT] = qt.as_float_array(quat)
        assert fused.shape == (DIMS,)
        return fused

    assert isinstance(pose, list), "Input must be a single `XyzQuat` or a list of them."

    liar = [fuse_xyz_quat(p) for p in pose]
    fused = np.concatenate(liar, axis=0)
    assert fused.shape == (len(pose) * DIMS,)
    return fused


def unfuse_xyz_quat(
    arr: NDArray[Shape["7 n"], nt.Floating]
) -> List[XyzQuat[Flo3, Quaternion]]:
    """
    Unpacks a fused 7D array back into XYZ and Quaternion components.

    Args:
        arr: Flattened fused representation(s).

    Returns:
        The unfused position(s) and orientation(s).
    """
    assert (
        arr.shape[0] % DIMS == 0
    ), f"Array length {arr.shape[0]} is not a multiple of {DIMS}."

    if arr.shape == (DIMS,):  # Single (7,) input
        q = qt.from_float_array(arr[QUAT_SLOT])
        q = qt_normalize(q)
        return [
            XyzQuat(arr[XYZ_SLOT], q),
        ]

    n = arr.shape[0] // DIMS  # Number of poses

    return [unfuse_xyz_quat(arr[i * DIMS : (i + 1) * DIMS])[0] for i in range(n)]


def clamp_multi_xyz_quat(
    center: List[XyzQuat[Flo3, Quaternion]],
    start: List[XyzQuat[Flo3, Quaternion]],
    end: List[XyzQuat[Flo3, Quaternion]],
    radii: Union[List[XyzQuat[float, float]], XyzQuat[float, float]],
    norm_ord=np.inf,
) -> List[XyzQuat[Flo3, Quaternion]]:
    """wrapper for clamp_to_sqewed_hs specialized in several 3D coordinate + one quaternion.

    The math for the quaternion is wrong (lerp instead of slerp). So:
    Center and start quat should not be opposite from each-other.
    Precision goes down if they are far appart. But it's not so bad.

    Args:
        center:  center from which not to diverge
        start: start point of the interpolation
        end: end point of the interpolation
        radii: allowed divergence for coord and quat
        norm_ord (int or numpy inf): order of the distance/norm used to create the hypersphere.

    Returns:
        Futhest point on the `start`-`end` segment that is inside the hypersphere of center `center` and radii `radii`.
    """
    if not isinstance(radii, list):
        radii = [radii for _ in range(len(center))]

    assert len(center) == len(start) == len(end) == len(radii)

    radii_arr = np.empty(len(radii) * DIMS, float)
    for i, value in enumerate(radii):
        index = i * DIMS
        radii_arr[XYZ_SLOT + index] = value.xyz
        radii_arr[QUAT_SLOT + index] = value.xyz

    fused_center = fuse_xyz_quat(center)
    fused_end = fuse_xyz_quat(end)
    fused_start = fuse_xyz_quat(start)

    fused_clamp = clamp_to_sqewed_hs(
        fused_center, fused_start, fused_end, radii_arr, norm_ord=norm_ord
    )
    unfused = unfuse_xyz_quat(arr=fused_clamp)

    assert len(center) == len(unfused)
    return unfused


def clamp_xyz_quat(
    center: XyzQuat[Flo3, Quaternion],
    start: XyzQuat[Flo3, Quaternion],
    end: XyzQuat[Flo3, Quaternion],
    radii: XyzQuat[float, float],
    norm_ord=2,
) -> XyzQuat[Flo3, Quaternion]:
    """wrapper for clamp_to_sqewed_hs specialized in one 3D coordinate + one quaternion.

    The math for the quaternion is wrong (lerp instead of slerp). So:
    Center and start quat should not be opposite from each-other.
    Precision goes down if they are far appart. But it's not so bad.

    Args:
        center:  center from which not to diverge
        start: start point of the interpolation
        end: end point of the interpolation
        radii: allowed divergence for coord and quat
        norm_ord (int or numpy inf): order of the distance/norm used to create the hypersphere.

    Returns:
        Futhest point on the `start`-`end` segment that is inside the hypersphere of center `center` and radii `radii`.
    """
    return clamp_multi_xyz_quat(
        center=[center],
        start=[start],
        end=[end],
        radii=[radii],
        norm_ord=norm_ord,
    )[0]
