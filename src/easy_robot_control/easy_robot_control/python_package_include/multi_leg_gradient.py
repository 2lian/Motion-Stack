# -*- coding: utf-8 -*-
"""
Provides the gradient of several legs

numba.njit is used to accelerate computation.
You should pass dtype=float32 numpy arrays to the numba functions or it will throw an error
(please look what it is before modification,
comment the @njit decorators  if you are struggling with modifications)

All robot parameters are loaded from basic_utilities

@author: Elian NEPPEL
@laboratory: AMC team, Space Robotic Lab, Tohoku University
"""

import numpy as np
from numba import njit, prange
from typing import Union, List, Dict, Any, Tuple, Callable

from python_package_include.distance_and_reachable_function import *
import python_package_include.utilities as ut

j = ut.j
float32 = np.float32

# functions transfered to utilities
holdset_to_point_angle = ut.holdset_to_point_angle
holdset_to_point_angle_numba = ut.holdset_to_point_angle_numba


@njit("float32[:,::3](float32[:,::3], float32[:])")
def rotate_points(points: np.ndarray, z_angles: np.ndarray) -> np.ndarray:
    """
    rotates every 'points' by 'z_angles'
    :param points:
    :param z_angles:
    :return: new np.ndarray
    """
    rotated_points = points.copy()
    points_xy_rotated = ut.complexify(rotated_points[:, :2]) * np.exp(z_angles * j)
    rotated_points[:, 0], rotated_points[:, 1] = np.real(points_xy_rotated), np.imag(points_xy_rotated)
    return rotated_points


@njit("boolean[:](float32[:,::3], float32[:])")
def multi_leg_reachable(points: np.ndarray, leg_z_angles: np.ndarray) -> np.ndarray:
    """
    computes is_reachable for every points given the offset of their leg.

    Rotates every points by leg_z_angles over the z axis before calculation
    leg_z_angles should be the offset angle of the leg trying to reach the point.
    So 0 if the point is to be reached by leg0, X*pi/3 if the point is to be reached by legX
    :param points: np.ndarray shape(N,3)
    :param leg_z_angles: np.ndarray shape(N,)
    :return: np.ndarray shape(N,) dtype=bool
    """
    points_as_leg0 = rotate_points(points, -leg_z_angles)
    reachability_array = np.empty((points_as_leg0.shape[0],), dtype='bool')
    for row in prange(points_as_leg0.shape[0]):
        reachability_array[row] = is_reachable_no_under_coxa(points_as_leg0[row, :])
    return reachability_array


@njit("float32[:,::3](float32[:,::3], float32[:])")
def multi_leg_vect(points: np.ndarray, leg_z_angles: np.ndarray) -> np.ndarray:
    """
    computes vect_to_avg_surf_weighted for every points given the offset of their leg.

    Rotates every points by leg_z_angles over the z axis before calculation
    then reverse the rotation on the result.
    leg_z_angles should be the offset angle of the leg trying to reach the point.
    So 0 if the point is to be reached by leg0, X*pi/3 if the point is to be reached by legX
    :param points: np.ndarray shape(N,3)
    :param leg_z_angles: np.ndarray shape(N,)
    :return: np.ndarray shape(N,3)
    """
    points_as_leg0 = rotate_points(points, -leg_z_angles)
    results_vect_array = np.empty(points_as_leg0.shape, dtype='float32')
    for row in prange(points_as_leg0.shape[0]):
        results_vect_array[row, :] = vect_to_avg_surf(points_as_leg0[row, :])
    result_rotated_back = rotate_points(results_vect_array, leg_z_angles)
    return result_rotated_back


@njit("Tuple((float32[:], boolean))(float32[:,::3], float32[:], float32)")
def multi_leg_avg_vect_weighted(points: np.ndarray, leg_z_angles: np.ndarray, reachable_divided_by: float = 2) \
        -> Tuple[np.ndarray, bool]:
    """ 4.83 µs for 6p
    computes the average vect_to_avg_surf for every point given the offset of their leg.
    Then divides by reachable_divided_by if the point is reachable
    If all points are reachable, return True on the second value returned
    Basicaly the gradient vector

    leg_z_angles should be the offset angle of the leg trying to reach the point.
    So 0 if the point is to be reached by leg0, X*pi/3 if the point is to be reached by legX

    :param points: np.ndarray shape(N,3)
    :param leg_z_angles: np.ndarray shape(N,)
    :param reachable_divided_by: divides vect of each leg by that if reachable
    :return: np.ndarray shape(3,), bool
    """
    distance_vect_array = np.empty(points.shape, dtype=float32)
    reachability_array = np.empty((points.shape[0], 1), dtype='bool')
    points_as_leg0 = rotate_points(points, -leg_z_angles)
    for row in prange(points.shape[0]):
        distance_vect_array[row, :] = vect_to_avg_surf(points_as_leg0[row, :])
        reachability_array[row, 0] = is_reachable_no_under_coxa(points_as_leg0[row, :])
    distance_vect_array = rotate_points(distance_vect_array,
                                        leg_z_angles)  # rotates the vector back to the correct angle
    # the sign of reachability_weight changes the behavior of the weight
    if reachable_divided_by > 0:  # faster
        # reachable have a weight of reachable_divided_by and non-reachable have 1
        # then done so the sum of the weight are equal to one
        reachability_weight = float32(1) / (reachability_array * float32(reachable_divided_by - 1) + float32(1))
        reachability_weight_normalized = reachability_weight / np.sum(reachability_weight)  # , axis=0)
    else:  # more precise for small movable solution
        # reachable have a weight of reachable_divided_by and non-reachable have 1
        # then simply divided by the number of legs
        reachability_weight = float32(1) / (reachability_array * float32(-reachable_divided_by - 1) + float32(1))
        reachability_weight_normalized = reachability_weight / np.shape(points)[0]
    # print(reachability_weight_normalized)
    weighted_vect = distance_vect_array * reachability_weight_normalized

    return np.sum(weighted_vect, axis=0), reachability_array.all()


@njit("float32(float32[:,::3], float32[:])")
def multi_leg_dist_avg(points: np.ndarray, leg_z_angles: np.ndarray) -> float32:
    """ 4.24 µs for 6p
    computes the average dist_to_avg_surf_weighted for every points given the offset of their leg.
    Basicaly the gradient

    leg_z_angles should be the offset angle of the leg trying to reach the point.
    So 0 if the point is to be reached by leg0, X*pi/3 if the point is to be reached by legX
    :param points: np.ndarray shape(N,3)
    :param leg_z_angles: np.ndarray shape(N,)
    :return: float32
    """
    points_as_leg0 = points.copy()
    points_xy_rotated = ut.complexify(points_as_leg0[:, :2]) * np.exp(-leg_z_angles * j)
    points_as_leg0[:, 0], points_as_leg0[:, 1] = np.real(points_xy_rotated), np.imag(points_xy_rotated)
    result = np.empty(leg_z_angles.shape, dtype='float32')
    for row in prange(points_as_leg0.shape[0]):
        result[row] = dist_to_avg_surf_weighted(points_as_leg0[row, :])
    return np.sum(result) / len(result)


def get_body_gradient_vect_function_weighted(points: np.ndarray, leg_z_angles: np.ndarray,
                                             default_weight: float = float32(2.0)) -> Callable:
    """ 641 ns
    returns the function computing the gradient vector of the center body for the given 'points' on the ground
    and their corresponding offset angles.
    The minimum is the position where the body should be.

    :param points: np.ndarray shape(N,3)
        points on the ground
    :param leg_z_angles: np.ndarray shape(N,)
        corresponding offset angles
    :return:
    """

    def gradient_vect(body_coord: np.ndarray(shape=(3,), dtype=float32), reachable_weight: float = default_weight) \
            -> Tuple[np.ndarray, bool]:
        """ 6.1 µs for 6p
        Gradient vector of the body position, with the footholds set by the parent function

        :param body_coord: position of the body
        :param reachable_weight: the grad OF THE LEG is divided by this value if the foothold is reachable
        :return: value of the gradient
        """
        gradient_function_result = multi_leg_avg_vect_weighted(points - body_coord, leg_z_angles, reachable_weight)
        return -gradient_function_result[0], gradient_function_result[1]

    return gradient_vect


def get_body_gradient_dist_function(points: np.ndarray, leg_z_angles: np.ndarray) -> Callable:
    """ 430 ns
    returns the function computing the gradient of the center body for the given 'points' on the ground
    and their corresponding offset angles.
    The minimum is the position where the body should be.

    :param points: np.ndarray shape(N,3)
        points on the ground
    :param leg_z_angles: np.ndarray shape(N,)
        corresponding offset angles
    :return:
    """

    def gradient(body_coord: np.ndarray(shape=(3,), dtype=float32)) -> float32:
        """ 6.1 µs for 6p
        Gradient of the body position, with the footholds set by the parent function

        :param body_coord: position of the body
        :return: value of the gradient
        """
        return multi_leg_dist_avg(points - body_coord, leg_z_angles)

    return gradient
