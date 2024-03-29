# -*- coding: utf-8 -*-
"""
Provides functions for the default leg0.
Computes reachable space, distances to middle of reachable space, score for the gradient descent.

numba.njit is used to accelerate computation
(please look what it is before modification, comment the @njit decorators  if you are struggling with modifications)

All robot parameters are loaded from basic_utilities

@author: Elian NEPPEL
@laboratory: AMC team, Space Robotic Lab, Tohoku University
"""

import numpy as np
import python_package_include.utilities as ut
from numba import njit

float32 = np.float32

leg_info = ut.leg_dim
body = float32(leg_info['body'])
coxa_angle_deg = float32(leg_info['a_coxa'])
coxa_length = float32(leg_info['coxa'])
tibia_angle_deg = float32(leg_info['a_femur'])  # sorry femur and tibia are mixed up
tibia_length = float32(leg_info['femur'])
femur_angle_deg = float32(leg_info['a_tibia'])  # sorry femur and tibia are mixed up
femur_length = float32(leg_info['tibia'])

max_angle_coxa = float32(np.deg2rad(coxa_angle_deg))
min_angle_coxa = float32(-np.deg2rad(coxa_angle_deg))

max_angle_coxa_w_margin = float32(np.deg2rad(coxa_angle_deg - 10))
min_angle_coxa_w_margin = float32(-np.deg2rad(coxa_angle_deg - 10))

max_angle_tibia = float32(np.deg2rad(tibia_angle_deg))
min_angle_tibia = float32(-np.deg2rad(tibia_angle_deg))

max_angle_tibia_wider = float32(np.deg2rad(tibia_angle_deg + 20))
min_angle_tibia_wider = float32(-np.deg2rad(tibia_angle_deg + 20))

max_tibia_to_gripper_dist = tibia_length + femur_length
min_tibia_to_gripper_dist = np.abs(tibia_length + femur_length * np.exp(np.deg2rad(femur_angle_deg) * 1j))
middle_TG = (max_tibia_to_gripper_dist + min_tibia_to_gripper_dist) / 2

f_math = True

# middle tibia_to_gripper distance, the surface we want the distance to is here

@njit('float32[::3](float32[::3])', fastmath=f_math)
def vect_to_avg_surf(point: np.ndarray) -> np.ndarray:
    """
    The function computes the shortest vector from the 'point' to the surface at the middle of the reachable space of
    leg 0. The 'point' array must have the shape (3,). The returned array also has the shape (3,) and represents the
    shortest vector from the 'point' to the surface at the middle of the reachable space of leg 0.

    :param point: np.ndarray shape: (3,) - The point from which the vector is to be calculated.
    :return: np.ndarray shape: (3,) - The shortest vector from the given point to the surface at the middle of the reachable space of leg0.
    """
    solution = np.empty((3,), dtype='float32')
    point_relative_to_coxa_origin = point - np.array([body, 0, 0], dtype='float32')
    p_in_horizontal_complex_plane = point_relative_to_coxa_origin[0] + point_relative_to_coxa_origin[1] * 1j
    required_angle_coxa = np.angle(p_in_horizontal_complex_plane)

    if required_angle_coxa > max_angle_coxa_w_margin:
        real_coxa_angle = max_angle_coxa_w_margin
    elif required_angle_coxa < min_angle_coxa_w_margin:
        real_coxa_angle = min_angle_coxa_w_margin
    else:
        real_coxa_angle = required_angle_coxa
    horizontal_complex_plane_with_x_following_coxa = p_in_horizontal_complex_plane * np.exp(-real_coxa_angle * 1j)

    solution[1] = np.imag(horizontal_complex_plane_with_x_following_coxa)

    vertical_complex_tibia_plane = np.real(horizontal_complex_plane_with_x_following_coxa - coxa_length) + point[2] * 1j
    required_tibia_angle = np.angle(vertical_complex_tibia_plane)

    if required_tibia_angle > max_angle_tibia_wider:
        real_tibia_angle = max_angle_tibia_wider
    elif required_tibia_angle < min_angle_tibia_wider:
        real_tibia_angle = min_angle_tibia_wider
    else:
        real_tibia_angle = required_tibia_angle
    tibia_plane_with_origin_on_the_avg_line = vertical_complex_tibia_plane - middle_TG * np.exp(real_tibia_angle * 1j)

    solution[0] = np.real(tibia_plane_with_origin_on_the_avg_line)
    solution[2] = np.imag(tibia_plane_with_origin_on_the_avg_line)
    align_solution_back_to_xy_plane = (solution[0] + solution[1] * 1j) * np.exp(real_coxa_angle * 1j)
    solution[0] = np.real(align_solution_back_to_xy_plane)
    solution[1] = np.imag(align_solution_back_to_xy_plane)

    return -solution


@njit('float32(float32[::3])', fastmath=f_math)
def dist_to_avg_surf(point: np.ndarray) -> float:
    """
    Computes the distance from the given point to the surface at the middle of the reachable space of leg 0.

    :param point: np.ndarray shape: (3,) - representing the coordinates of the point.
    :return: float - representing the distance from the point to the surface at the middle of the reachable space of leg 0.
    """
    return np.linalg.norm(vect_to_avg_surf(point))


@njit('boolean(float32[::3])', fastmath=f_math)
def is_reachable(point: np.ndarray) -> bool:
    """
    Computes if 'point' is in the reachable space of leg0

    :param point: np.ndarray, shape (3,) - The 3D point to be checked.
    :return: bool - True if the given point is within the reachable space of leg 0, otherwise returns False.
    """
    solution = np.empty((3,), dtype='float32')

    # Shifting the point so that the coxa is at the origin.
    point_with_coxa_as_origin = point - np.array([body, 0, 0], dtype='float32')

    # Converting the point to the horizontal complex plane.
    p_in_horizontal_complex_plane = point_with_coxa_as_origin[0] + point_with_coxa_as_origin[1] * 1j

    # Calculating the required angle of the coxa to reach the point.
    required_angle_coxa = np.angle(p_in_horizontal_complex_plane)

    # Checking if the required angle is within the valid range.
    if min_angle_coxa < required_angle_coxa < max_angle_coxa:
        real_coxa_angle = required_angle_coxa
    elif np.pi - min_angle_coxa > required_angle_coxa > np.pi - max_angle_coxa:
        real_coxa_angle = np.pi + required_angle_coxa
    else:
        return False

    # Rotating the horizontal plane so that the x-axis is following the coxa.
    horizontal_complex_plane_with_x_following_coxa = p_in_horizontal_complex_plane * np.exp(-real_coxa_angle * 1j)

    # Extracting the y coordinate.
    solution[1] = np.imag(horizontal_complex_plane_with_x_following_coxa)

    # Computing the distance from the point to the tibia in the vertical complex plane.
    vertical_complex_tibia_plane = np.real(horizontal_complex_plane_with_x_following_coxa - coxa_length) + point[2] * 1j
    distance = np.abs(vertical_complex_tibia_plane)

    # Checking if the distance is within the valid range.
    if distance < min_tibia_to_gripper_dist:
        return False
    elif distance > max_tibia_to_gripper_dist:
        return False

    # Calculating the required angle of the tibia to reach the point.
    # this is not really the real angle
    required_tibia_angle = np.angle(vertical_complex_tibia_plane)

    # Checking if the required angle is within the valid range.
    if required_tibia_angle > max_angle_tibia:
        real_tibia_angle = max_angle_tibia
    elif required_tibia_angle < min_angle_tibia:
        real_tibia_angle = min_angle_tibia
    else:
        return True

    # Transforming the vertical_complex_tibia_plane so that the origin is at the femur joint.
    tibia_plane_with_origin_on_femur = vertical_complex_tibia_plane - tibia_length * np.exp(real_tibia_angle * 1j)

    # Checking if the point is within the valid range of the femur
    if np.abs(tibia_plane_with_origin_on_femur) < femur_length:
        return True
    else:
        return False


@njit('boolean(float32[::3])', fastmath=f_math)
def is_reachable_no_under_coxa(point: np.ndarray) -> bool:
    """
    Computes if 'point' is in the reachable space of leg0, without anything below the coxa

    :param point: np.ndarray, shape (3,) - The 3D point to be checked.
    :return: bool - True if the given point is within the reachable space of leg 0, otherwise returns False.
    """
    solution = np.empty((3,), dtype='float32')
    point_relative_to_coxa_origin = point - np.array([body, 0, 0], dtype='float32')
    p_in_horizontal_complex_plane = point_relative_to_coxa_origin[0] + point_relative_to_coxa_origin[1] * 1j
    required_angle_coxa = np.angle(p_in_horizontal_complex_plane)

    if min_angle_coxa < required_angle_coxa < max_angle_coxa:
        real_coxa_angle = required_angle_coxa
    # elif np.pi - min_angle_coxa > required_angle_coxa > np.pi - max_angle_coxa:
    #     real_coxa_angle = np.pi + required_angle_coxa
    else:
        return False
    horizontal_complex_plane_with_x_following_coxa = p_in_horizontal_complex_plane * np.exp(-real_coxa_angle * 1j)

    solution[1] = np.imag(horizontal_complex_plane_with_x_following_coxa)

    vertical_complex_tibia_plane = np.real(horizontal_complex_plane_with_x_following_coxa - coxa_length) + point[2] * 1j
    distance = np.abs(vertical_complex_tibia_plane)
    if distance < min_tibia_to_gripper_dist:
        return False
    elif distance > max_tibia_to_gripper_dist:
        return False

    required_tibia_angle = np.angle(vertical_complex_tibia_plane)
    if required_tibia_angle > max_angle_tibia:
        real_tibia_angle = max_angle_tibia
    elif required_tibia_angle < min_angle_tibia:
        real_tibia_angle = min_angle_tibia
    else:
        return True
    tibia_plane_with_origin_on_femur = vertical_complex_tibia_plane - tibia_length * np.exp(real_tibia_angle * 1j)

    if np.abs(tibia_plane_with_origin_on_femur) < femur_length:
        return True
    else:
        return False


@njit('float32[::3](float32[::3])', fastmath=f_math)
def vect_to_avg_surf_weighted(point: np.ndarray) -> np.ndarray:
    """
    Computes the shortest vector from 'point' to the surface at the middle of the reachable space of leg0
    and divides by 10 if 'point' is reachable

    :param point: np.ndarray shape: (3,) - The point from which the vector is to be calculated.
    :return: np.ndarray shape: (3,) - The shortest vector from the given point to the surface at the middle of the reachable space of leg0.
            divided by 10 if the point is reachable.
    """
    if is_reachable_no_under_coxa(point):
        return vect_to_avg_surf(point) / 10
    else:
        return vect_to_avg_surf(point)


@njit('float32(float32[::3])', fastmath=f_math)
def dist_to_avg_surf_weighted(point: np.ndarray) -> float:
    """
    Computes the distance from 'point' to the surface at the middle of the reachable space of leg0
    and divides by 10 if 'point' is reachable

    :param point: np.ndarray shape: (3,) - The point to calculate the distance from.
    :return: float32 - The distance from the given point to the surface at the middle of the reachable space of leg0,
             divided by 10 if the point is reachable.
    """
    return np.linalg.norm(vect_to_avg_surf_weighted(point))


if __name__ == '__main__':
    p = np.array([400 + 321, 1200, 0], dtype='float32')
    dist_to_avg_surf(p)
