"""
@author: Melina SUNDELL
@laboratory: AMC team, Space Robotic Lab, Tohoku University
"""

from typing import Tuple, Union
import numpy as np
import numba
import python_package_include.utilities as ut


@numba.njit("complex64[:](complex64[:], int32)")
def shift_array(array: np.ndarray, direction: np.int32) -> np.ndarray:
    array_shifted = array.copy()
    array_shifted = array_shifted[
        (np.arange(array_shifted.shape[0], dtype=np.int32) + direction)
        % len(array_shifted)
    ]
    return array_shifted


@numba.njit("complex64[:](complex64[:])")
def holdset_to_vectors(holdset: np.ndarray) -> np.ndarray:
    # Vectors from point to next point
    holdset_shifted = shift_array(holdset, 1)
    vectors = holdset_shifted - holdset
    return vectors


@numba.njit("float32[:](complex64[:])")
def holdset_angles(holdset: np.ndarray) -> np.ndarray:
    # Calculate angles between adjacent edges

    vectors = holdset_to_vectors(holdset)
    vectors_shifted = shift_array(vectors, -1)

    # Angle between vector and x-axis. Convert negative angles to positive.
    angle1 = np.arctan2(vectors.imag, vectors.real)
    angle2 = np.arctan2(-vectors_shifted.imag, -vectors_shifted.real)

    # Angles between adjacent edges
    angles = (angle2 - angle1) % (2 * np.pi)

    return angles.astype(np.float32)


@numba.njit("complex64[:](complex64[:], float32[:])")
def remove_concave_points(holdset: np.ndarray, angles: np.ndarray) -> np.ndarray:
    # Remove angles that are >pi
    valid = (angles <= np.pi)
    points = holdset[valid]
    return points


@numba.njit("complex64[:](complex64[:], complex64)")
def vectors_point_to_holdset(holdset: np.ndarray, point: np.ndarray) -> np.ndarray:
    # Calculate vector from a point to each point in holdset
    vectors = point - holdset
    return vectors


@numba.njit("Tuple([complex64[:], complex64])(float32[:,::2], float32[::2])")
def stability_arrays(
    holdset: np.ndarray, point: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    # Remove nan values
    holdset, _ = ut.holdset_to_point_angle_numba(holdset)

    # Convert holdset of points [x,y,x] into complex points [x+jy]
    holdset = ut.complexify(holdset)
    point = point[0] + ut.j[0] * point[1]

    # Calculate angles between adjacent edges
    angles = holdset_angles(holdset)

    # Remove points that are >180deg
    # (They lie inside the support polygon, so we don't care. Also, cross product method will not work)
    convex_holdset = remove_concave_points(holdset, angles)

    return (convex_holdset, point)


@numba.njit("boolean(float32[:,::2], float32[::2])")
def is_point_stable(holdset: np.ndarray, point: np.ndarray) -> bool:
    # Find if point inside support polygon
    # Method: cross products of vectors from point to adjacent vertices

    # Convert holdset of points [x,y,x] into complex points [x+jy]
    holdset_stab, point_stab = stability_arrays(holdset, point)

    # Vectors from point to adjacent vertices
    vectors_to_convex_vertex1 = vectors_point_to_holdset(holdset_stab, point_stab)
    vectors_to_convex_vertex2 = shift_array(vectors_to_convex_vertex1, 1)

    cross_products = np.imag(
        np.conj(vectors_to_convex_vertex1) * vectors_to_convex_vertex2
    )

    # If all cross products are either pos or neg, the point is inside the
    # polygon
    if np.all(cross_products >= 0) or np.all(cross_products < 0):
        return True
    # otherwise outside polygon
    else:
        return False


@numba.njit("float32[::2](float32[:,::2], float32[::2], boolean)")
def stability_vector(
    holdset: np.ndarray, point: np.ndarray, inside: bool
) -> np.ndarray:
    # Find the shortest distance from a point to a polygon
    # Static Stability Margin: minimum distance from C.o.M to polygon edges

    # Convert holdset of points [x,y,x] into complex points [x+jy]
    holdset_stab, point_stab = stability_arrays(holdset, point)

    p = point_stab
    a = holdset_stab
    b = shift_array(holdset_stab, 1)
    ab = b - a
    ap = p - a
    pa = a - p

    dot_abap = ab.real * ap.real + ab.imag * ap.imag
    proj_ab = (
        a.real
        + (dot_abap / (ab.real**2 + ab.imag**2)) * ab.real
        + 1j * (a.imag + (dot_abap / (ab.real**2 + ab.imag**2)) * ab.imag)
    )
    vector_ap_proj = proj_ab - p
    distances = np.sqrt(vector_ap_proj.real**2 + vector_ap_proj.imag**2)

    if inside:
        # If point inside polygon
        ssm_distance = min(distances)
        ssm_index = distances == ssm_distance
        ssm_vector = vector_ap_proj[ssm_index]

    else:
        # If point is outside polygon, shortest distance is
        # either normal to point or distance to corner

        # Remove edges that are not normal to point (cannot be closest)
        t = dot_abap / (ab.real**2 + ab.imag**2)
        edge_index = (t >= 0) & (t <= 1)
        new_vector_ap_proj = vector_ap_proj[edge_index]

        # Combine vectors and distances to normal edges and corners
        all_distances = np.append(
            np.sqrt(new_vector_ap_proj.real**2 + new_vector_ap_proj.imag**2),
            np.abs(pa),
        )
        all_vectors = np.append(new_vector_ap_proj, pa)

        ssm_distance = min(all_distances)
        ssm_index = all_distances == ssm_distance
        ssm_vector = all_vectors[ssm_index]

    # Convert SSM_vector from imag to real
    ssm_vector_float32 = np.concatenate(
        (ssm_vector.real.astype(np.float32), ssm_vector.imag.astype(np.float32))
    )

    return np.float32(1.00) * ssm_vector_float32
