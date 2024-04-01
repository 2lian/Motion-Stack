# -*- coding: utf-8 -*-
"""
Provides the gradient descent to find movable configurations

numba.njit is used to accelerate computation.
You should pass dtype=float32 numpy arrays to the numba functions or it will throw an error
(please look what it is before modification,
comment the @njit decorators  if you are struggling with modifications)

All robot parameters are loaded from basic_utilities

@author: Elian NEPPEL
@laboratory: AMC team, Space Robotic Lab, Tohoku University
"""

from python_package_include.multi_leg_gradient import *
import python_package_include.utilities as ut

USE_CACHE = ut.USE_CACHE
FAST_MATH = True

# functions transfered to utilities
average_of_holdset = ut.average_of_holdset


def find_body_position(
    points: np.ndarray,
    angles: np.ndarray,
    start: np.ndarray = None,
    max_iteration: int = 20,
    lower_z_limit=-np.inf,
    upper_z_limit=np.inf,
) -> Tuple[np.ndarray, bool]:
    """
    Computes the body position of the hexapod given the foothold position and
    corresponding angle offset of the leg reaching the point.
    If the gradient descent converges also returns True

    :param points: np.ndarray shape: (N, 3) - the position of the end of each leg on the ground.
    :param angles: np.ndarray shape: (N,) - the angle offset of each leg's mount in radians.
    :param start: np.ndarray shape: (3,) - the starting point for gradient descent. Default: None.
    :param max_iteration: int - the maximum number of iterations for gradient descent. Default: 20.
    :param lower_z_limit: float - the lower limit for the z-coordinate of the center of the hexapod. Default: -inf.
    :param upper_z_limit: float - the upper limit for the z-coordinate of the center of the hexapod. Default: inf.

    :return: Tuple[np.ndarray, bool] - the center body position of the hexapod and whether the gradient descent converges.
    """
    lower_z_limit = float32(lower_z_limit)
    upper_z_limit = float32(upper_z_limit)

    # Get the gradient function for the body position
    gradient_function = get_body_gradient_vect_function_weighted(points, angles)

    # Set the starting point for the gradient descent algorithm
    if start is None:
        # The starting point is set to the center of the feet positions plus an offset of 250 units along the z axis
        starting_point = np.sum(points, axis=0) / points.shape[0]
        starting_point[2] += float32(250)
    else:
        starting_point = start

    # Initialize the current point for the gradient descent algorithm
    current_point = starting_point
    z = current_point[2]
    if z < lower_z_limit:
        current_point[2] = lower_z_limit
    elif z > upper_z_limit:
        current_point[2] = upper_z_limit

    # Initialize the reachability flag for the gradient descent algorithm
    reachability = False

    # Run the gradient descent algorithm for a maximum of max_search_step steps
    for current_step in range(max_iteration):

        # Update the weight for the weighted gradient function
        if current_step < 2:
            weight = 5
        elif current_step < 6:
            weight = 2.9
        else:
            weight = -6

        # Compute the gradient vector and reachability flag for the current point using the gradient function
        gradient_vector, reachability = gradient_function(
            current_point, reachable_weight=weight
        )

        # If the gradient descent algorithm reached convergence, exit the loop
        if reachability:
            break

        # Update the current point for the gradient descent algorithm
        current_point = current_point + gradient_vector
        z = current_point[2]
        if z < lower_z_limit:
            current_point[2] = lower_z_limit
        elif z > upper_z_limit:
            current_point[2] = upper_z_limit

    # Return the estimated body position and the reachability flag
    return current_point, reachability


def apply_gradient_on_holdset(
    holdset: np.ndarray,
    number_of_legs_above_body_allowed: int = 0,
    start: np.ndarray = None,
    max_iteration: int = 20,
) -> Tuple[np.ndarray, bool]:
    """
    applies find_body_position to the provided holdset. It converts to points angles, and computes the z_lower_limit
    based on the number of legs allowed above the body

    :param holdset: array float shape(6,3) - points to grab
    :param number_of_legs_above_body_allowed: z_lower_limit
    based on the number of legs allowed above the body if negative, no z limit
    :param start: start position of the descent
    :param max_iteration: number of iter of the gradent descent
    :return:
    """
    points, angles = holdset_to_point_angle(holdset)

    if number_of_legs_above_body_allowed >= 0:
        z_heights = points[:, 2]
        k = number_of_legs_above_body_allowed
        partition_indices = np.argpartition(-z_heights, k)
        kth_largest_height = z_heights[partition_indices[k]]
        z_lower_limit = kth_largest_height
    else:
        z_lower_limit = -np.inf

    return find_body_position(
        points, angles, start, max_iteration, lower_z_limit=z_lower_limit
    )


@njit(
    "Tuple((float32[:], boolean))(float32[:,::3], float32[:], Optional(float32[::3]), Optional(int64), "
    "Optional(float32), Optional(float32))",
    cache=USE_CACHE,
)
def find_body_position_optimized(
    points: np.ndarray,
    angles: np.ndarray,
    start=None,
    max_iteration=None,
    lower_z_limit=None,
    upper_z_limit=None,
) -> Tuple[np.ndarray, bool]:
    """
    This version bypasses the intermediate function creation used in find_body_position,
    alowing numba compatibility and faster speed
    Computes the body position of the hexapod given the foothold position and
    corresponding angle offset of the leg reaching the point.
    If the gradient descent converges also returns True.

    :param points: np.ndarray shape: (N, 3) - the position of the end of each leg on the ground.
    :param angles: np.ndarray shape: (N,) - the angle offset of each leg's mount in radians.
    :param start: np.ndarray shape: (3,) - the starting point for gradient descent. Default: None.
    :param max_iteration: int - the maximum number of iterations for gradient descent. Default: 20.
    :param lower_z_limit: float - the lower limit for the z-coordinate of the center of the hexapod. Default: -inf.
    :param upper_z_limit: float - the upper limit for the z-coordinate of the center of the hexapod. Default: inf.

    :return: Tuple[np.ndarray, bool] - the center body position of the hexapod and whether the gradient descent converges.
    """
    if max_iteration is None:
        max_iteration = int(20)
    if lower_z_limit is None:
        lower_z_limit = float32(-np.inf)
    if upper_z_limit is None:
        upper_z_limit = float32(np.inf)

    # This is part we optimise
    # gradient_function = get_body_gradient_vect_function_weighted(points, angles)

    # Set the starting point for the gradient descent algorithm
    if start is None:
        # The starting point is set to the center of the feet positions plus an offset of 250 units along the z axis
        starting_point = np.sum(points, axis=0) / points.shape[0]
        starting_point[2] += float32(250)
    else:
        starting_point = start

    # Initialize the current point for the gradient descent algorithm
    current_point = starting_point
    z = current_point[2]
    if z < lower_z_limit:
        current_point[2] = lower_z_limit
    elif z > upper_z_limit:
        current_point[2] = upper_z_limit

    # Initialize the reachability flag for the gradient descent algorithm
    reachability = False

    # Run the gradient descent algorithm for a maximum of max_search_step steps
    for current_step in range(max_iteration):

        # Update the weight for the weighted gradient function
        if current_step < 2:
            weight = 5
        elif current_step < 6:
            weight = 2.9
        else:
            weight = -6

        # Compute the gradient vector and reachability flag for the current point using the gradient function
        gradient_function_result = multi_leg_avg_vect_weighted(
            points=points - current_point,
            leg_z_angles=angles,
            reachable_divided_by=weight,
        )
        gradient_vector, reachability = (
            -gradient_function_result[0],
            gradient_function_result[1],
        )

        # If the gradient descent algorithm reached convergence, exit the loop
        if reachability:
            break

        # Update the current point for the gradient descent algorithm
        current_point = current_point + gradient_vector
        z = current_point[2]
        if z < lower_z_limit:
            current_point[2] = lower_z_limit
        elif z > upper_z_limit:
            current_point[2] = upper_z_limit

    # Return the estimated body position and the reachability flag
    return current_point, reachability


@njit(
    "Tuple((float32[:], boolean))(float32[:,::3], float32[:], int32, optional(float32[::3]), optional(int32))",
    nogil=True,
    cache=USE_CACHE,
)
def apply_gradient_on_pointsangles_optimized(
    points,
    angles,
    number_of_legs_above_body_allowed: int = 0,
    start: np.ndarray = None,
    max_iteration: int = None,
):
    """
    apply_gradient_on_holdset but numba version
    :param holdset:
    :param number_of_legs_above_body_allowed:
    :param start:
    :param max_iteration:
    :return:
    """

    if number_of_legs_above_body_allowed >= 0:
        z_heights = points[:, 2]
        # lower_z_limit = np.sort(z_heights)[-number_of_legs_above_body_allowed-1]
        n_lowest_index = np.argsort(z_heights)[
            -number_of_legs_above_body_allowed - 1
        ]
        lower_z_limit = z_heights[n_lowest_index]
        # k = number_of_legs_above_body_allowed
        # partition_indices = np.argpartition(-z_heights, k)
        # kth_largest_height = z_heights[partition_indices[k]]
        # lower_z_limit = kth_largest_height
    else:
        lower_z_limit = -np.inf

    return find_body_position_optimized(
        points,
        angles,
        start=start,
        max_iteration=max_iteration,
        lower_z_limit=lower_z_limit,
        upper_z_limit=None,
    )


@njit(
    "Tuple((float32[:], boolean))(float32[::6,::3], int32, optional(float32[::3]), optional(int32))",
    nogil=True,
    cache=USE_CACHE,
)
def apply_gradient_on_holdset_optimized(
    holdset,
    number_of_legs_above_body_allowed: int = 0,
    start: np.ndarray = None,
    max_iteration: int = None,
):
    """
    apply_gradient_on_holdset but numba version
    :param holdset:
    :param number_of_legs_above_body_allowed:
    :param start:
    :param max_iteration:
    :return:
    """
    points, angles = holdset_to_point_angle_numba(holdset)

    if number_of_legs_above_body_allowed >= 0:
        z_heights = points[:, 2]
        # lower_z_limit = np.sort(z_heights)[-number_of_legs_above_body_allowed-1]
        n_lowest_index = np.argsort(z_heights)[
            -number_of_legs_above_body_allowed - 1
        ]
        lower_z_limit = z_heights[n_lowest_index]
        # k = number_of_legs_above_body_allowed
        # partition_indices = np.argpartition(-z_heights, k)
        # kth_largest_height = z_heights[partition_indices[k]]
        # lower_z_limit = kth_largest_height
    else:
        lower_z_limit = -np.inf

    return find_body_position_optimized(
        points,
        angles,
        start=start,
        max_iteration=max_iteration,
        lower_z_limit=lower_z_limit,
        upper_z_limit=None,
    )


@njit(
    ["Tuple((float32[:,::3], boolean[:]))(float32[:,:,::3], float32[:,:], int32)"],
    cache=USE_CACHE,
)
def gradient_points_angle(
    points: np.ndarray, angles: np.ndarray, max_iteration: int = 20
):
    """
    For an array of points and angles, computes the movability of each hold and a possible body position.
    This does the gradient descent assuming number_of_legs_above_body_allowed=0 start=None
    :param holdset_array:
    :param max_iteration:
    :return:
    """
    movability_result = np.empty((points.shape[0],), "bool")
    position_result = np.empty((points.shape[0], 3), "float32")
    # position_shittylist = [None]*holdset_array.shape[0]

    # finding start position with vectorized operation
    average = np.sum(points, axis=1) / points.shape[1]
    average_plus250 = average_of_holdset(points) + np.array(
        [0, 0, 250], dtype=float32
    )
    # average_plus250[:, 2] = average_plus250[:, 2] + 250  # shifting

    for row in prange(points.shape[0]):
        result_of_the_row = apply_gradient_on_pointsangles_optimized(
            points[row, :, :],
            angles[row, :],
            number_of_legs_above_body_allowed=0,
            start=average_plus250[row, :],
            max_iteration=max_iteration,
        )

        movability_result[row] = result_of_the_row[1]
        position_result[row, :] = result_of_the_row[0]

    return position_result, movability_result


@njit(
    ["Tuple((float32[:,::3], boolean[:]))(float32[:,::6,::3], int32)"],
    cache=USE_CACHE,
)
def easy_gradient_3Darray(holdset_array: np.ndarray, max_iteration: int = 20):
    """
    For an array of holdset, computes the movability of each hold and a possible body position.
    This does the gradient descent assuming number_of_legs_above_body_allowed=0 start=None
    :param holdset_array:
    :param max_iteration:
    :return:
    """
    movability_result = np.empty((holdset_array.shape[0],), "bool")
    position_result = np.empty((holdset_array.shape[0], 3), "float32")
    # position_shittylist = [None]*holdset_array.shape[0]

    # finding start position with vectorized operation
    average_plus250 = average_of_holdset(holdset_array) + np.array(
        [0, 0, 250], dtype=float32
    )
    # average_plus250[:, 2] = average_plus250[:, 2] + 250  # shifting

    for row in prange(holdset_array.shape[0]):
        result_of_the_row = apply_gradient_on_holdset_optimized(
            holdset_array[row, :, :],
            number_of_legs_above_body_allowed=0,
            start=average_plus250[row, :],
            max_iteration=max_iteration,
        )

        movability_result[row] = result_of_the_row[1]
        position_result[row, :] = result_of_the_row[0]

    return position_result, movability_result


if __name__ == "__main__":
    pass
