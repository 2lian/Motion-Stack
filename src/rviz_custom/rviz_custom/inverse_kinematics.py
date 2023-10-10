"""
Creator: Elian NEPPEL
Copyright: Elian NEPPEL
"""

import numpy as np

L1 = 0.10 # Length between joint1 (Near the base joint) and joint2
L2 = 0.10 # Length between joint2 and joint3 (Near the Tip Joint)
L3 = 0.10 # Length between Joint3 and Tip
D1 = 0.18 # Distance between Origin of base and origin of the joint1

coxaMax = np.deg2rad(90)
coxaMin = np.deg2rad(-90)

femurMax = np.deg2rad(120)
femurMin = np.deg2rad(-120)

tibiaMax = np.deg2rad(120)
tibiaMin = np.deg2rad(-120)

coxaLength = float(L1*1000)
femurLength = float(L2*1000)
tibiaLength = float(L3*1000)

minFemurTargetDist = np.abs(femurLength + tibiaLength * np.exp(1j * tibiaMax))

bodyToCoxa = float(D1*1000)


def law_of_cosines(a, b, c):
    """return the angle between the length sides a, b. Of the abc triangle"""
    return np.arccos((c ** 2 - b ** 2 - a ** 2) / (-2.0 * a * b))


def choice_ik_coxa_zero(target: np.ndarray, reverse_coxa: bool = False, femur_down: bool = False):
    """4 possible angle solutions exists, the basic one will be good enough for now I think"""
    coxa_angle = max(min(
        np.arctan2(target[1], target[0]) + (np.pi if reverse_coxa else 0),
        coxaMax), coxaMin)
    # print("coxa_angle :", np.rad2deg(coxa_angle))
    rot_mat_coxa = np.array([[np.cos(coxa_angle), -np.sin(coxa_angle), 0],
                             [np.sin(coxa_angle), np.cos(coxa_angle), 0],
                             [0, 0, 1],
                             ])

    femur_ref_frame = (target @ rot_mat_coxa)
    femur_ref_frame[0] -= femurLength
    # print("femur_ref_frame :", femur_ref_frame)

    intermediate_distance = min(max(
        np.linalg.norm(femur_ref_frame),
        minFemurTargetDist), femurLength + tibiaLength)
    intermediate_angle = np.arctan2(femur_ref_frame[2], femur_ref_frame[0])

    femur_switch = -1 if femur_down else 1

    femur_angle = intermediate_angle + femur_switch * law_of_cosines(femurLength,
                                                                     intermediate_distance, tibiaLength)

    femur_angle = max(min(femur_angle, femurMax), femurMin)
    # print("femur_angle :", np.rad2deg(femur_angle))

    rot_mat_coxa = np.array([[np.cos(-femur_angle), 0, np.sin(-femur_angle)],
                             [0, 1, 0],
                             [-np.sin(-femur_angle), 0, np.cos(-femur_angle)]
                             ])

    tibia_ref_frame = femur_ref_frame @ rot_mat_coxa
    tibia_ref_frame[0] -= tibiaLength
    # print("tibia_ref_frame :", tibia_ref_frame)

    tibia_angle = max(min(
        np.arctan2(tibia_ref_frame[2], tibia_ref_frame[0]),
        tibiaMax), tibiaMin)
    # print("tibia_angle :", np.rad2deg(tibia_angle))

    return np.array([coxa_angle, femur_angle, tibia_angle], dtype=float)

def forward_kine_coxa_zero(joint_angles):
    joints = np.array([coxaLength, femurLength, tibiaLength], dtype=float)

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

def simple_auto_ik_coxa_zero(target):
    reverse_coxa_priority_order = [False, True]
    femur_down_priority_order = [False, True] # no False because I don't want to flip the robot

    minimal_error = np.inf
    best_angles = np.empty(3, dtype=float)

    for fd in femur_down_priority_order:
        for rc in reverse_coxa_priority_order:
            angles = choice_ik_coxa_zero(target, reverse_coxa=rc, femur_down=fd)
            tip_position = forward_kine_coxa_zero(angles)[-1]
            error = np.linalg.norm(tip_position - target)
            print(tip_position)

            if error < minimal_error - 5:
                minimal_error = error
                best_angles = angles

    return best_angles



def leg_ik(leg_number: int, target: np.ndarray):
    angle = leg_number * np.pi / 2

    z_rot_mat = np.array([[np.cos(angle), -np.sin(angle), 0],
                          [np.sin(angle), np.cos(angle), 0],
                          [0, 0, 1],
                          ])

    rotated_target = target @ z_rot_mat
    rotated_target[0] -= bodyToCoxa

    return simple_auto_ik_coxa_zero(rotated_target)
