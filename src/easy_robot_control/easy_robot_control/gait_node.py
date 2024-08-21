"""
This node is responsible for chosing targets and positions.

Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

import numpy as np
from numpy.typing import NDArray
import quaternion as qt
import time
import matplotlib.pyplot as plt
import rclpy
from rclpy.task import Future
from rclpy.node import Node, Union, List
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.publisher import Publisher
from EliaNode import Client, EliaNode, error_catcher, np2TargetSet, np2tf

import pkg_resources
from rclpy.time import Duration
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TransformStamped, Transform

from custom_messages.srv import (
    ReturnTargetBody,
    ReturnVect3,
    Vect3,
    TFService,
    ReturnTargetSet,
    SendTargetSet,
    SendTargetBody,
)
from custom_messages.msg import TargetBody, TargetSet
import python_package_include.distance_and_reachable_function as reach_pkg
import python_package_include.multi_leg_gradient as multi_pkg
import python_package_include.stability as stab_pkg
import python_package_include.inverse_kinematics as ik_pkg


class GaitNode(EliaNode):
    def __init__(self):
        # rclpy.init()
        super().__init__("gait_node")  # type: ignore
        self.setAndBlockForNecessaryClients("mover_alive")

        # V Params V
        #   \  /   #
        #    \/    #
        self.declare_parameter("number_of_legs", 4)
        self.NUMBER_OF_LEG = (
            self.get_parameter("number_of_legs").get_parameter_value().integer_value
        )
        #    /\    #
        #   /  \   #
        # ^ Params ^

        # V Publishers V
        #   \  /   #
        #    \/    #

        #    /\    #
        #   /  \   #
        # ^ Publishers ^

        # V Service client V
        #   \  /   #
        #    \/    #

        self.sendTargetBody: Client = self.get_and_wait_Client(
            "get_targetset", ReturnTargetSet
        )
        self.sendTargetBody: Client = self.get_and_wait_Client(
            "go2_targetbody", SendTargetBody
        )

        #    /\    #
        #   /  \   #
        # ^ Service client ^

        # V Service server V
        #   \  /   #
        #    \/    #

        #    /\    #
        #   /  \   #
        # ^ Service server ^

        self.firstSpin = self.create_timer(
            1, self.firstSpinCBK, callback_group=MutuallyExclusiveCallbackGroup()
        )

    def firstSpinCBK(self):
        self.destroy_timer(self.firstSpin)
        height = 220
        width = 250
        ts = np.array(
            [
                [width, 0, -height],
                [0, width, -height],
                [-width, 0, -height],
                [0, -width, -height],
            ],
            dtype=float,
        )

        self.pwarn("call")
        self.sendTargetBody.call(
            SendTargetBody.Request(
                target_body=TargetBody(
                    target_set=np2TargetSet(ts),
                    body=np2tf(),
                )
            )
        )
        self.pwarn("call done")
        self.pwarn("call")
        self.sendTargetBody.call(
            SendTargetBody.Request(
                target_body=TargetBody(
                    # target_set=np2TargetSet(ts),
                    body=np2tf(np.array([-50.0, 0.0, 0.0], None)),
                )
            )
        )
        self.pwarn("call done")

    def getTargetSet(self) -> NDArray: ...

    def goToTargetSet(self, NDArray) -> None: ...

    def crawlToTargetSet(self, NDArray) -> None: ...

    def goToDefault(self): ...

    def stand(self): ...


def main(args=None):
    rclpy.init()
    node = GaitNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt as e:
        node.get_logger().debug(
            "KeyboardInterrupt caught, node shutting down cleanly\nbye bye <3"
        )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


def compute_targetset_pressure_precise(
    last_targetset, body_shift, potential_next, leg_angles, leg_dimemsions
):
    body_shift = body_shift.astype(np.float32)
    legCount = leg_angles.shape[0]
    approximate_next_target = potential_next - body_shift

    dist_after = multi_pkg.multi_leg_vect(last_targetset - 1 * body_shift, leg_angles)

    dist_approx = multi_pkg.multi_leg_vect(approximate_next_target, leg_angles)

    angle_after = np.empty_like(last_targetset)
    angle_appromimate = np.empty_like(last_targetset)

    for leg in range(legCount):
        leg_param = ik_pkg.rotate_legparam_by(
            ik_pkg.moonbot0_leg_default, ik_pkg.PI_OVER_2_Z_QUAT**legCount, False
        )
        angle_after[leg, :] = ik_pkg.simple_leg_ik(
            last_targetset[leg, :] - body_shift,
            leg_param,
        )
        angle_appromimate[leg, :] = ik_pkg.simple_leg_ik(
            approximate_next_target[leg, :],
            leg_param,
        )

    pressure = np.empty((legCount,), np.float32)

    for leg in range(legCount):
        next_pressure = dist_after.copy()
        next_pressure[leg, :] = dist_approx[leg, :]
        next_pressure = np.linalg.norm(next_pressure, axis=1) / (
            np.linalg.norm(body_shift) * legCount
        )
        coxa_press = angle_after[:, 0].copy()
        coxa_press[leg] = angle_appromimate[leg, 0]
        coxa_press = abs(coxa_press) / abs(leg_dimemsions.coxaMax) * 3

        stabbool_before = stab_pkg.is_point_stable(
            np.delete(last_targetset, leg, axis=0), np.zeros_like(body_shift)
        )
        stabbool_after = stab_pkg.is_point_stable(
            np.delete(last_targetset, leg, axis=0), body_shift
        )
        if not (stabbool_before and stabbool_after):
            stab_before = stab_pkg.stability_vector(
                np.delete(last_targetset, leg, axis=0),
                np.zeros_like(body_shift),
                stabbool_before,
            )
            stab_after = stab_pkg.stability_vector(
                np.delete(last_targetset, leg, axis=0), body_shift, stabbool_after
            )
            stab_antipress = (
                (
                    np.linalg.norm(stab_after * ~stabbool_after)
                    - np.linalg.norm(stab_before * ~stabbool_before)
                )
                / np.linalg.norm(body_shift)
                / 3
            )
        else:
            stab_antipress = 0
        # pressure[leg] = sum(coxa_press + next_pressure)
        pressure[leg] = sum(coxa_press + next_pressure + stab_antipress)

    return pressure


def compute_targetset_pressure(last_targetset, body_shift, leg_angles, leg_dimemsions):
    legCount = leg_angles.shape[0]
    approximate_next_target = last_targetset + body_shift * (legCount - 1)

    dist_after = multi_pkg.multi_leg_vect(
        last_targetset - body_shift.astype(np.float32), leg_angles
    )

    dist_approx = multi_pkg.multi_leg_vect(approximate_next_target, leg_angles)

    angle_after = np.empty_like(last_targetset)
    angle_appromimate = np.empty_like(last_targetset)

    for leg in range(legCount):
        this_leg_dim = ik_pkg.rotate_leg_by(leg_dimemsions, ik_pkg.PI_OVER_2_Z_QUAT**leg)
        angle_after[leg, :] = ik_pkg.simple_leg_ik(
            last_targetset[leg, :] - body_shift, this_leg_dim
        )
        angle_appromimate[leg, :] = ik_pkg.simple_leg_ik(
            approximate_next_target[leg, :], this_leg_dim
        )

    pressure = np.empty((legCount,), np.float32)

    for leg in range(legCount):
        next_pressure = dist_after.copy()
        next_pressure[leg, :] = dist_approx[leg, :]
        next_pressure = np.linalg.norm(next_pressure, axis=1) / (
            np.linalg.norm(body_shift) * legCount
        )
        coxa_press = angle_after[:, 0].copy()
        coxa_press[leg] = angle_appromimate[leg, 0]
        coxa_press = abs(coxa_press) / abs(leg_dimemsions.coxaMax) * 1

        # stabbool_before = stab_pkg.is_point_stable(
        #     np.delete(last_targetset, leg, axis=0), np.zeros_like(body_shift)
        # )
        # stabbool_after = stab_pkg.is_point_stable(
        #     np.delete(last_targetset, leg, axis=0), body_shift
        # )
        # if not (stabbool_before and stabbool_after):
        #     stab_before = stab_pkg.stability_vector(
        #         np.delete(last_targetset, leg, axis=0),
        #         np.zeros_like(body_shift),
        #         stabbool_before,
        #     )
        #     stab_after = stab_pkg.stability_vector(
        #         np.delete(last_targetset, leg, axis=0), body_shift, stabbool_after
        #     )
        #     stab_antipress = (
        #         np.linalg.norm(stab_after * ~stabbool_after) - np.linalg.norm(stab_before * ~stabbool_before)
        #     ) / np.linalg.norm(body_shift)
        # else:
        #     stab_antipress = 0
        pressure[leg] = sum(coxa_press + next_pressure)
        # pressure[leg] = sum(coxa_press + next_pressure + stab_antipress)

    return pressure
