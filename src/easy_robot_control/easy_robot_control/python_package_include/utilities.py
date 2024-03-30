# -*- coding: utf-8 -*-
"""
Provides values and functions used all throughout the code

numba.njit is used to accelerate computation.
You should pass dtype=float32 numpy arrays to the numba functions or it will throw an error
(please look what it is before modification,
comment the @njit decorators  if you are struggling with modifications)

@author: Elian NEPPEL
@laboratory: AMC team, Space Robotic Lab, Tohoku University
"""

import numpy as np
import quaternion
import quaternion as qt
from numba import njit, prange, guvectorize
import numba
from typing import Union, List, Dict, Any, Tuple, Callable

j = np.array([1j]).astype(np.complex64)
# the imaginary unit as complex 64
pi_over_3_zquat = qt.from_rotation_vector(np.array([0, 0, np.pi / 3], dtype='float32'))
# precomputed quaternion for pi/3 rotation
float32 = np.float32

leg_dim = {'body': 181,  # dimensions of the robot
           'coxa': 64,
           'femur': 129,
           'tibia': 160,
           'a_coxa': 60,
           'a_femur': 97,
           'a_tibia': 114,
           }

default_leg0_tip = np.array([461, 0, 0], dtype='float32')  # coord of the tip of leg0
default_holdset = np.empty((6, 3), dtype='float32')  # default holdset of the robot

for legnum in range(6):  # p0 rotated 5 times for the other legs
    q = pi_over_3_zquat ** legnum
    rotated = qt.rotate_vectors(q, default_leg0_tip)
    default_holdset[legnum, :] = rotated

@njit("complex64[:](float32[:,::2])")
def complexify(data: np.ndarray) -> np.ndarray:
    """
    :param data: np.ndarray shape(N,2) float32 - 2 columns float array
    :return: np.ndarray shape(N,1) complex 64 - 1 column complex array
    """
    return data[:, 0] + data[:, 1] * j


@njit(["float32[:,::6,::3](float32[:,::6,::3])"])
def replace_nan_by_0_numba(array: np.ndarray) -> np.ndarray:
    """
    replace nans by 0 in the provided array
    :param array: any shape
    :return: same shape as input
    """
    shape = array.shape
    array = array.flatten()
    array[np.isnan(array)] = 0
    array = array.reshape(shape)
    return array


@njit(["float32[:,::3](float32[:,::6,::3])"])
def average_of_holdset(holdset_array: np.ndarray) -> np.ndarray:
    """
    Computes np.average(input_array, axis=1), computing the avg of the 6 points of every holdset
    np.nan values are ignored in the calculation
    :param holdset_array: float32 shape(N, 6, 3) - array of holdset
    :return: float32  shape(N, 3) - average of each holdset
    """
    hold_3D_nan_as_0 = replace_nan_by_0_numba(holdset_array)
    count_of_non_nan_values = np.sum(~np.isnan(holdset_array[:, :, 0]), axis=1).reshape((hold_3D_nan_as_0.shape[0], 1))
    average = np.sum(hold_3D_nan_as_0, axis=1) / count_of_non_nan_values
    return average


@njit(["float32[:,::6,::3](float32[:,::6,::3])"])
def avg2zero_holdset(holdset_array: np.ndarray) -> np.ndarray:
    """
    Center the holdset around the origin by subtracting the center of mass of the 6 holdset
    :param holdset_array: float32 shape(N, 6, 3) - array of holdset
    :return: float32 shape(N, 6, 3) - array of holdset with average at the origin
    """
    # idk why the array is not contiguous
    return holdset_array - np.reshape(np.ascontiguousarray(average_of_holdset(holdset_array)),
                                      (holdset_array.shape[0], 1, 3))


leg_offset_angle = (np.arange(0, 6, 1) * np.pi / 3).astype(np.float32)


def holdset_to_point_angle(holdset: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    fixed_leg_number = ~np.isnan(holdset[:, 0])
    angles = leg_offset_angle[fixed_leg_number]
    points = holdset[fixed_leg_number]
    return points, angles


@njit("Tuple((float32[:,::3],float32[:]))(float32[::6,::3])")
def holdset_to_point_angle_numba(holdset: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    fixed_leg_number = ~np.isnan(holdset[:, 0])
    angles = leg_offset_angle[fixed_leg_number]
    points = holdset[fixed_leg_number]
    return points, angles


rotation_matrices_list = np.array(
    [quaternion.as_rotation_matrix(pi_over_3_zquat ** (-1 * i)) for i in range(6)]
).astype('float32')


# @njit("float32[:,::6,::3](float32[:,::6,::3], int8)")
# @numba.njit()
def rotate_by_pithree(holdset_array: np.ndarray, nb_of_pi3: int) -> np.ndarray:
    """
    Rotates every hold in the holdset array by pi/3*'nb_of_pi3'
    :param holdset_array: np.ndarray float32 shape(N, 6, 3) - holdset array
    :param nb_of_pi3: int - number of pi/3 rotations to apply
    :return: np.ndarray float32 shape(N, 6, 3) - holdset
    """
    # everthing_in_3_columns = np.reshape(np.ascontiguousarray(holdset_array), (-1, 3))
    everthing_in_3_columns = np.reshape(holdset_array, (-1, 3))
    # ev3col_rotated = np.matmul(everthing_in_3_columns, rotation_matrices_list[rot % 6])
    ev3col_rotated = everthing_in_3_columns @ rotation_matrices_list[nb_of_pi3 % 6]
    return ev3col_rotated.reshape(holdset_array.shape)


@njit("float32[:,::6,::3](float32[:,::6,::3], int8)")
def rotate_leg_order(holdset_array: np.ndarray, nb_of_rotation: int) -> np.ndarray:
    """
    rotate the order of the legs. So leg 0 will become leg 'nb_of_rotation'
    :param holdset_array:
    :param nb_of_rotation:
    :return:
    """
    order = (np.arange(6) - nb_of_rotation) % 6
    return holdset_array[:, order, :]


@njit("float32[:,::6,::3](float32[:,::6,::3])")
def replace_nan_with_inf(holdset_array):
    for i in range(holdset_array.shape[0]):
        for j in range(holdset_array.shape[1]):
            if np.isnan(holdset_array[i, j, 0]):
                holdset_array[i, j, :] = np.inf
    return holdset_array

@njit("boolean[:](float32[:,::6,::3], float32)")
def dist2adjacent_is_enough(holdset_array: np.ndarray, min_dist_allowed: float) -> np.ndarray:
    """
    checks that the distance between each foothold and the neighboring foothold is more than 'min_dist_allowed'
    for every holdset in the array
    :param holdset_array: np.ndarray float32 shape(N, 6, 3) - holdset array
    :param min_dist_allowed: minimum distance allowed between two consecutive footholds
    :return: np.ndarray bool shape(N,) - True if the minimum distance is more than min_dist_allowed
    """
    # return np.ones(shape=holdset_array.shape[0], dtype="bool")
    holdset_array = replace_nan_with_inf(holdset_array)
    # Compute the differences between adjacent points, including the last and first points
    differences = holdset_array[:, np.arange(1, holdset_array.shape[1] + 1) % holdset_array.shape[1], :] - holdset_array

    # Calculate the squared Euclidean distances for all holdset_array
    squared_distances = np.sum(differences ** 2, axis=2)

    # Find the smallest squared distance for each holdset
    # stores it in the first row of previous array
    for row in prange(squared_distances.shape[0]):
        squared_distances[row, 0] = np.min(squared_distances[row])

    # Take the square root to get the smallest distances
    smallest_distances = np.sqrt(squared_distances[:, 0])

    return smallest_distances > min_dist_allowed

# @njit("boolean[:](float32[:,::6,::3])")
# def basic_not_crossing(holdset_array: np.ndarray) -> np.ndarray:
#
#     differences = holdset_array[:, np.arange(1, holdset_array.shape[1] + 1) % holdset_array.shape[1], :] - holdset_array
#
