"""
Creator: Elian NEPPEL
Copyright: Elian NEPPEL
"""

import numpy as np

D1 = 0.181  # Distance between Origin of base and origin of the joint1
L1 = 0.283 - D1  # Length between joint1 (Near the base joint) and joint2
L2 = 0.396 - L1 - D1  # Length between joint2 and joint3 (Near the Tip Joint)
L3 = 0.490 - (L2 + L1 + D1)  # Length between Joint3 and Tip


class LegParameters:
    def __init__(self,
                 body_to_coxa,
                 coxa_length,
                 femur_length,
                 tibia_length,
                 coxaMax_degree,
                 coxaMin_degree,
                 femurMax_degree,
                 femurMin_degree,
                 tibiaMax_degree,
                 tibiaMin_degree,
                 ):
        self.bodyToCoxa = body_to_coxa
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
        self.minFemurTargetDist = np.abs(self.femurLength + self.tibiaLength * np.exp(1j * self.tibiaMax))


moonbot_leg = LegParameters(
    body_to_coxa=float(D1 * 1000),
    coxa_length=float(L1 * 1000),
    femur_length=float(L2 * 1000),
    tibia_length=float(L3 * 1000),
    coxaMax_degree=90,
    coxaMin_degree=-90,
    femurMax_degree=120,
    femurMin_degree=-120,
    tibiaMax_degree=120,
    tibiaMin_degree=-12
)


def law_of_cosines(a, b, c):
    """return the angle between the length sides a, b. Of the abc triangle"""
    return np.arccos((c ** 2 - b ** 2 - a ** 2) / (-2.0 * a * b))


def choice_ik_coxa_zero(target: np.ndarray, leg_param: LegParameters = moonbot_leg, reverse_coxa: bool = False,
                        femur_down: bool = False):
    """4 possible angle solutions exists, the basic one will be good enough for now I think"""
    coxa_angle = max(min(
        np.arctan2(target[1], target[0] * (-1 if reverse_coxa else 1)),
        leg_param.coxaMax), leg_param.coxaMin)
    # print("coxa_angle :", np.rad2deg(coxa_angle))
    rot_mat_coxa = np.array([[np.cos(coxa_angle), -np.sin(coxa_angle), 0],
                             [np.sin(coxa_angle), np.cos(coxa_angle), 0],
                             [0, 0, 1],
                             ])

    femur_ref_frame = (target @ rot_mat_coxa)
    femur_ref_frame[0] -= leg_param.coxaLength
    # print("femur_ref_frame :", femur_ref_frame)

    intermediate_distance = min(max(
        np.linalg.norm(femur_ref_frame),
        leg_param.minFemurTargetDist), leg_param.femurLength + leg_param.tibiaLength)
    intermediate_angle = np.arctan2(femur_ref_frame[2], femur_ref_frame[0])

    # print("intermediate_angle :", np.rad2deg(intermediate_angle))

    femur_switch = -1 if femur_down else 1

    # femur_angle = intermediate_angle + femur_switch * law_of_cosines(femurLength,
    #                                                                  intermediate_distance, tibiaLength)
    femur_angle = intermediate_angle + femur_switch * law_of_cosines(leg_param.femurLength,
                                                                     intermediate_distance, leg_param.tibiaLength)
    femur_angle = max(min(femur_angle, leg_param.femurMax), leg_param.femurMin)
    # print("femur_angle :", np.rad2deg(femur_angle))

    rot_mat_coxa = np.array([[np.cos(-femur_angle), 0, np.sin(-femur_angle)],
                             [0, 1, 0],
                             [-np.sin(-femur_angle), 0, np.cos(-femur_angle)]
                             ])

    tibia_ref_frame = femur_ref_frame @ rot_mat_coxa
    tibia_ref_frame[0] -= leg_param.femurLength
    # print("tibia_ref_frame :", tibia_ref_frame)

    tibia_angle = max(min(
        np.arctan2(tibia_ref_frame[2], tibia_ref_frame[0]),
        leg_param.tibiaMax), leg_param.tibiaMin)
    # print("tibia_angle :", np.rad2deg(tibia_angle))

    return np.array([coxa_angle, femur_angle, tibia_angle], dtype=float)


def forward_kine_coxa_zero(joint_angles, leg_param: LegParameters = moonbot_leg):
    joints = np.array([leg_param.coxaLength, leg_param.femurLength, leg_param.tibiaLength], dtype=float)

    points = np.zeros((3, 3), dtype=float)
    points[:, 0] = joints

    for i in range(1, 3):
        angle = joint_angles[i]
        y_rot_mat = np.array([[np.cos(angle), 0, np.sin(angle)],
                              [0, 1, 0],
                              [-np.sin(angle), 0, np.cos(angle)]
                              ])
        points[i:] = points[i:] @ y_rot_mat

    z_rot_mat = np.array([[np.cos(-joint_angles[0]), -np.sin(-joint_angles[0]), 0],
                          [np.sin(-joint_angles[0]), np.cos(-joint_angles[0]), 0],
                          [0, 0, 1],
                          ])
    points = points @ z_rot_mat

    for i in range(2):
        points[i + 1] += points[i]

    return points


def forward_kine_body_zero(joint_angles, leg_number, leg_param: LegParameters = moonbot_leg):
    points_coxa_origin = forward_kine_coxa_zero(joint_angles, leg_param)
    points_body_origin = np.zeros((4, 3), dtype=float)
    points_body_origin[:, 0] = leg_param.bodyToCoxa
    points_body_origin[1:, :] += points_coxa_origin

    rot_angle = np.pi / 2 * leg_number
    z_rot_mat = np.array([[np.cos(rot_angle), np.sin(rot_angle), 0],
                          [-np.sin(rot_angle), np.cos(rot_angle), 0],
                          [0, 0, 1],
                          ])
    points_rotated_back = points_body_origin @ z_rot_mat

    return points_rotated_back


def auto_ik_coxa(target: np.ndarray, previous_angles: np.ndarray, leg_param: LegParameters = moonbot_leg):
    reverse_coxa_priority_order = [False, True]
    femur_down_priority_order = [True, False]

    minimal_error = np.inf
    minimal_travel = np.inf
    best_angles = np.empty(3, dtype=float)

    angles_arr = np.empty(shape=(4, 3), dtype=float)
    error_arr = np.empty(shape=(4,), dtype=float)
    travel_arr = np.empty(shape=(4,), dtype=float)

    for fd in femur_down_priority_order:
        bias = fd * np.deg2rad(30)  # if femur up, 30deg penality
        for rc in reverse_coxa_priority_order:
            angles_arr[fd * 2 + rc] = choice_ik_coxa_zero(target, leg_param=leg_param, reverse_coxa=rc, femur_down=fd)
            tip_position = forward_kine_coxa_zero(angles_arr[fd * 2 + rc], leg_param=leg_param)[-1]
            travel_arr[fd * 2 + rc] = np.sum(abs(previous_angles - angles_arr[fd * 2 + rc])) + bias
            error_arr[fd * 2 + rc] = np.linalg.norm(tip_position - target)

    closest_b_arr = error_arr - np.min(error_arr) < 50  # closest solution and those less than 5cm away
    travel_arr += ~closest_b_arr * 500  # far away is penalized into oblivion
    lesser_travel = np.argmin(travel_arr)
    return angles_arr[lesser_travel]


def simple_auto_ik_coxa_zero(target, leg_param: LegParameters = moonbot_leg):
    reverse_coxa_priority_order = [False, True]
    femur_down_priority_order = [False, True]  # no False because I don't want to flip the robot

    minimal_error = np.inf
    best_angles = np.empty(3, dtype=float)

    for fd in femur_down_priority_order:
        for rc in reverse_coxa_priority_order:
            angles = choice_ik_coxa_zero(target, leg_param=leg_param, reverse_coxa=rc, femur_down=fd)
            tip_position = forward_kine_coxa_zero(angles, leg_param=leg_param)[-1]
            error = np.linalg.norm(tip_position - target)

            if error < minimal_error - 5:
                minimal_error = error
                best_angles = angles

    return best_angles


def simple_leg_ik(leg_number: int, target: np.ndarray, leg_param: LegParameters = moonbot_leg):
    angle = leg_number * np.pi / 2

    z_rot_mat = np.array([[np.cos(angle), -np.sin(angle), 0],
                          [np.sin(angle), np.cos(angle), 0],
                          [0, 0, 1],
                          ])

    rotated_target = target @ z_rot_mat
    rotated_target[0] -= leg_param.bodyToCoxa

    return simple_auto_ik_coxa_zero(rotated_target, leg_param=leg_param)


def leg_ik(leg_number: int, target: np.ndarray, previous_angles: np.ndarray, leg_param: LegParameters = moonbot_leg):
    angle = leg_number * np.pi / 2

    z_rot_mat = np.array([[np.cos(angle), -np.sin(angle), 0],
                          [np.sin(angle), np.cos(angle), 0],
                          [0, 0, 1],
                          ])

    rotated_target = target @ z_rot_mat
    rotated_target[0] -= leg_param.bodyToCoxa

    return auto_ik_coxa(rotated_target, previous_angles, leg_param=leg_param)
