"""
Creator: Elian NEPPEL
Copyright: Elian NEPPEL
"""

import numpy as np
import quaternion as qt
from typing import Any
import copy

POSITIVE_FEMUR_PENALTY = 30  # deg
ACCEPTABLE_ERROR = 50  # mm
PI_OVER_2_Z_QUAT = qt.from_vector_part(np.array([0, 0, np.pi / 2]))

D1 = 0.181  # Distance between Origin of base and origin of the joint1
L1 = 0.0645  # Length between joint1 (Near the base joint) and joint2
L2 = 0.129  # Length between joint2 and joint3 (Near the Tip Joint)
L3 = 0.16  # Length between Joint3 and Tip

ArrayOfFloat = np.ndarray[Any, np.dtype[np.float_]]
# type for array of float


class LegParameters:
    def __init__(
        self,
        mounting_point: ArrayOfFloat,
        mounting_quaternion: ArrayOfFloat,
        coxa_length: float,
        femur_length: float,
        tibia_length: float,
        coxaMax_degree: float,
        coxaMin_degree: float,
        femurMax_degree: float,
        femurMin_degree: float,
        tibiaMax_degree: float,
        tibiaMin_degree: float,
    ):
        """Creates an object parameterizing the leg

        Args:
            mounting_point - array (float) (3,): position of coxa this is basically the origin of the leg
            mounting_quaternion - quaternion : orientation of coxa
            coxa_length:
            femur_length:
            tibia_length:
            coxaMax_degree: will be converted to radians
            coxaMin_degree:
            femurMax_degree:
            femurMin_degree:
            tibiaMax_degree:
            tibiaMin_degree:
        """
        self.mounting_point = mounting_point.astype(float).copy()
        self.mounting_quaternion = mounting_quaternion.copy()
        self.coxaLength = coxa_length
        self.femurLength = femur_length
        self.tibiaLength = tibia_length

        self.coxaMax = np.deg2rad(coxaMax_degree)
        self.coxaMin = np.deg2rad(coxaMin_degree)
        self.femurMax = np.deg2rad(femurMax_degree)
        self.femurMin = np.deg2rad(femurMin_degree)
        self.tibiaMax = np.deg2rad(tibiaMax_degree)
        self.tibiaMin = np.deg2rad(tibiaMin_degree)

        self.minFemurTargetDist = 0.0
        self.update_minFemurTargetDist()

    def update_minFemurTargetDist(self):
        self.minFemurTargetDist = np.abs(
            self.femurLength + self.tibiaLength * np.exp(1j * self.tibiaMax)
        )


moonbot0_leg_default = LegParameters(
    mounting_point=np.array([D1, 0, 0], dtype=float),
    mounting_quaternion=qt.from_rotation_vector(np.zeros(3)),
    coxa_length=float(L1 * 1000),
    femur_length=float(L2 * 1000),
    tibia_length=float(L3 * 1000),
    coxaMax_degree=90,
    coxaMin_degree=-90,
    femurMax_degree=120,
    femurMin_degree=-120,
    tibiaMax_degree=120,
    tibiaMin_degree=-12,
)


def rotate_leg_by(
    leg: LegParameters, quat: qt.quaternion, inplace: bool = False
) -> LegParameters:
    if not inplace:
        leg = copy.deepcopy(moonbot0_leg_default)
    leg.mounting_point = qt.rotate_vectors(quat, leg.mounting_point)
    leg.mounting_quaternion = leg.mounting_quaternion * quat
    return leg


def law_of_cosines(a: float, b: float, c: float):
    """return the angle between the length sides a, b. Of the abc triangle

    Args:
        a: length of side A
        b: length of side B
        c: length of side C

    Returns:
        angle between sides A, B
    """
    return np.arccos((c**2 - b**2 - a**2) / (-2.0 * a * b))


def choice_ik_coxa_zero(
    target: ArrayOfFloat,
    leg_param: LegParameters = moonbot0_leg_default,
    reverse_coxa: bool = False,
    femur_down: bool = False,
):
    """computes the raw IK given a target in leg reference frame. 4 solutions exists, they are toggled by the two input booleans. The best solution should then be picked.

    Args:
        target - array (float) (3,): target to reach in the leg reference frame (not robot ref)
        leg_param: parameters of the leg
        reverse_coxa: if True, solution with coxa pointing away from the target will be returned
        femur_down: if True, solution with the femure pointing down will be returned

    Returns:
        joints_angle - array (float) (3,): [coxa, femur, coxa] angles in this order
    """
    coxa_angle = max(
        min(
            np.arctan2(target[1], target[0] * (-1 if reverse_coxa else 1)),
            leg_param.coxaMax,
        ),
        leg_param.coxaMin,
    )
    # print("coxa_angle :", np.rad2deg(coxa_angle))
    rot_mat_coxa = np.array(
        [
            [np.cos(coxa_angle), -np.sin(coxa_angle), 0],
            [np.sin(coxa_angle), np.cos(coxa_angle), 0],
            [0, 0, 1],
        ]
    )

    femur_ref_frame = target @ rot_mat_coxa
    femur_ref_frame[0] -= leg_param.coxaLength
    # print("femur_ref_frame :", femur_ref_frame)

    intermediate_distance = min(
        max(np.linalg.norm(femur_ref_frame), leg_param.minFemurTargetDist),
        leg_param.femurLength + leg_param.tibiaLength,
    )
    intermediate_angle = np.arctan2(femur_ref_frame[2], femur_ref_frame[0])

    # print("intermediate_angle :", np.rad2deg(intermediate_angle))

    femur_switch = -1 if femur_down else 1

    # femur_angle = intermediate_angle + femur_switch * law_of_cosines(femurLength,
    # intermediate_distance, tibiaLength)
    femur_angle = intermediate_angle + femur_switch * law_of_cosines(
        leg_param.femurLength, intermediate_distance, leg_param.tibiaLength
    )
    femur_angle = max(min(femur_angle, leg_param.femurMax), leg_param.femurMin)
    # print("femur_angle :", np.rad2deg(femur_angle))

    rot_mat_coxa = np.array(
        [
            [np.cos(-femur_angle), 0, np.sin(-femur_angle)],
            [0, 1, 0],
            [-np.sin(-femur_angle), 0, np.cos(-femur_angle)],
        ]
    )

    tibia_ref_frame = femur_ref_frame @ rot_mat_coxa
    tibia_ref_frame[0] -= leg_param.femurLength
    # print("tibia_ref_frame :", tibia_ref_frame)

    tibia_angle = max(
        min(np.arctan2(tibia_ref_frame[2], tibia_ref_frame[0]), leg_param.tibiaMax),
        leg_param.tibiaMin,
    )
    # print("tibia_angle :", np.rad2deg(tibia_angle))

    return np.array([coxa_angle, femur_angle, tibia_angle], dtype=float)


def forward_kine_coxa_zero(
    joint_angles: ArrayOfFloat, leg_param: LegParameters = moonbot0_leg_default
):
    """Returns end tip position given coxa, femur, tibia angles in leg frame of reference

    Args:
        joint_angles - array (float) (3,):coxa, femur, tibia angles
        leg_param: parameters of the leg

    Returns:
        tip_position - array (float) (3,)
    """
    joints = np.array(
        [leg_param.coxaLength, leg_param.femurLength, leg_param.tibiaLength], dtype=float
    )

    points = np.zeros((3, 3), dtype=float)
    points[:, 0] = joints

    for i in range(1, 3):
        angle = joint_angles[i]
        y_rot_mat = np.array(
            [
                [np.cos(angle), 0, np.sin(angle)],
                [0, 1, 0],
                [-np.sin(angle), 0, np.cos(angle)],
            ]
        )
        points[i:] = points[i:] @ y_rot_mat

    z_rot_mat = np.array(
        [
            [np.cos(-joint_angles[0]), -np.sin(-joint_angles[0]), 0],
            [np.sin(-joint_angles[0]), np.cos(-joint_angles[0]), 0],
            [0, 0, 1],
        ]
    )
    points = points @ z_rot_mat

    for i in range(2):
        points[i + 1] += points[i]

    return points


def forward_kine_body_zero(
    joint_angles: ArrayOfFloat, leg_param: LegParameters = moonbot0_leg_default
):
    """Returns end tip position given coxa, femur, tibia angles in body frame of ref

    Args:
        joint_angles - array (float) (3,): joint angles
        leg_param: leg parameters

    Returns:
        end_tip_position
    """
    points_coxa_origin = forward_kine_coxa_zero(joint_angles, leg_param)
    points_body_origin = np.zeros((4, 3), dtype=float)
    points_body_origin[0, :] = leg_param.mounting_point
    points_body_origin[1:, :] = points_body_origin[0, :] + qt.rotate_vectors(
        leg_param.mounting_quaternion, points_coxa_origin
    )

    return points_body_origin


def auto_ik_coxa(
    target: ArrayOfFloat,
    previous_angles: ArrayOfFloat,
    leg_param: LegParameters = moonbot0_leg_default,
):
    """finds joint angles to reach the target, It uses previous_angles to pick the solution that is close to the previous one.
    It favoritizes the femur being down and
    might ignore a perfect solution if the movement is not worth it compared to the error.

    Args:
        target - array (float) (3,): target to reach
        previous_angles - array (float) (3,): angles from which the new target will be reached
        leg_param: leg parameters

    Returns:
        joint angles
    """
    reverse_coxa_priority_order = [False, True]
    femur_down_priority_order = [True, False]

    angles = np.empty(shape=(4, 3), dtype=float)
    errors = np.empty(shape=(4,), dtype=float)
    joints_travel = np.empty(shape=(4,), dtype=float)

    for fd in femur_down_priority_order:
        bias = fd * np.deg2rad(POSITIVE_FEMUR_PENALTY)  # if femur up, Xdeg penality
        for rc in reverse_coxa_priority_order:
            index = fd * 2 + rc
            angles[index] = choice_ik_coxa_zero(
                target, leg_param=leg_param, reverse_coxa=rc, femur_down=fd
            )
            tip_position = forward_kine_coxa_zero(angles[index], leg_param=leg_param)[-1]
            joints_travel[index] = np.sum(abs(previous_angles - angles[index])) + bias
            errors[index] = np.linalg.norm(tip_position - target)

    # closest solution and those less than Xcm away are selected
    low_error = errors - np.min(errors) < ACCEPTABLE_ERROR
    joints_travel += ~low_error * 500  # high error is penalized into oblivion
    lesser_travel = np.argmin(joints_travel)  # closest solution to previous is selected
    return angles[lesser_travel]


def leg_ik(
    target: ArrayOfFloat,
    previous_angles: ArrayOfFloat,
    leg_param: LegParameters = moonbot0_leg_default,
):
    """computes IK given target in body reference frame
    It favoritizes the femur being down and
    might ignore a perfect solution if the movement is not worth it compared to the error.

    Args:
        target - array (float) (3,): target to reach
        previous_angles - array (float) (3,): angles from which the new target will be reached
        leg_param: leg parameters

    Returns:
        joint angles
    """
    corrected_target = target.copy()
    corrected_target -= leg_param.mounting_point
    corrected_target = qt.rotate_vectors(
        1 / leg_param.mounting_quaternion, corrected_target
    )

    return auto_ik_coxa(corrected_target, previous_angles, leg_param=leg_param)


def simple_auto_ik_coxa_zero(target, leg_param: LegParameters = moonbot0_leg_default):
    """finds joint angles to reach the target.
    It favoritizes the femur being down

    Args:
        target - array (float) (3,): target to reach
        leg_param: leg parameters

    Returns:
        joint angles
    """
    reverse_coxa_priority_order = [False, True]
    femur_down_priority_order = [False, True]

    minimal_error = np.inf
    best_angles = np.empty(3, dtype=float)

    for fd in femur_down_priority_order:
        for rc in reverse_coxa_priority_order:
            angles = choice_ik_coxa_zero(
                target, leg_param=leg_param, reverse_coxa=rc, femur_down=fd
            )
            tip_position = forward_kine_coxa_zero(angles, leg_param=leg_param)[-1]
            error = np.linalg.norm(tip_position - target)

            if error < minimal_error - 5:
                minimal_error = error
                best_angles = angles

    return best_angles


def simple_leg_ik(target: ArrayOfFloat, leg_param: LegParameters = moonbot0_leg_default):
    """computes IK given target in body reference frame, nothing special, just favoritize femur down

    Args:
        target - array (float) (3,): target to reach
        leg_param: leg parameters

    Returns:
        joint angles
    """
    corrected_target = target.copy()
    corrected_target -= leg_param.mounting_point
    corrected_target = qt.rotate_vectors(
        1 / leg_param.mounting_quaternion, corrected_target
    )

    return simple_auto_ik_coxa_zero(corrected_target, leg_param=leg_param)
