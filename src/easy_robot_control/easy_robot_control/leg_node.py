"""
This node is responsible for recieving targets with trajectories of the
corresponding leg. It will performe smooth movements in the body center frame of reference
or relative to the current end effector position.
This node keeps track of the leg end effector to generate trajectories to the target.

Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

import time
import traceback

import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from custom_messages.srv import Vect3, ReturnVect3

SUCCESS = True
TARGET_OVERWRITE_DIST = 50
TARGET_TIMEOUT_BEFORE_OVERWRITE = 1  # seconds
WAIT_AFTER_MOTION = 0.1  # seconds


def error_catcher(func):
    # This is a wrapper to catch and display exceptions
    def wrap(*args, **kwargs):
        try:
            out = func(*args, **kwargs)
        except Exception as exception:
            if exception is KeyboardInterrupt:
                raise KeyboardInterrupt
            else:
                traceback_logger_node = Node("error_node")  # type: ignore
                traceback_logger_node.get_logger().error(traceback.format_exc())
                raise exception
        return out

    return wrap


class LegNode(Node):
    def __init__(self):
        # rclpy.init()
        super().__init__(f"ik_node")  # type: ignore

        # V Parameters V
        #   \  /   #
        #    \/    #
        self.declare_parameter("leg_number", 0)
        self.leg_num = (
            self.get_parameter("leg_number").get_parameter_value().integer_value
        )

        self.declare_parameter("std_movement_time", 3.0)
        self.movementTime = (
            self.get_parameter("std_movement_time").get_parameter_value().double_value
        )

        self.declare_parameter("movement_update_rate", 30.0)
        self.movementUpdateRate = (
            self.get_parameter("movement_update_rate").get_parameter_value().double_value
        )
        #    /\    #
        #   /  \   #
        # ^ Parameters ^

        self.necessary_client = self.create_client(Empty, f"ik_{self.leg_num}_alive")
        while not self.necessary_client.wait_for_service(timeout_sec=2):
            self.get_logger().warning(
                f"""Waiting for node, check that the [ik_{self.leg_num}_alive] service is running"""
            )
        self.get_logger().warning(f"""ik_{self.leg_num} connected :)""")

        self.lastTarget = np.zeros(3, dtype=float)
        self.currentTip = np.zeros(3, dtype=float)

        # V Callback Groups V
        #   \  /   #
        #    \/    #
        movement_cbk_group = MutuallyExclusiveCallbackGroup()
        #    /\    #
        #   /  \   #
        # ^ Callback Groups ^

        # V Publishers V
        #   \  /   #
        #    \/    #
        self.ik_pub = self.create_publisher(Vector3, f"set_ik_target_{self.leg_num}", 10)
        #    /\    #
        #   /  \   #
        # ^ Publishers ^

        # V Subscribers V
        #   \  /   #
        #    \/    #
        self.tip_pos_sub = self.create_subscription(
            Vector3,
            f"tip_pos_{self.leg_num}",
            self.tip_pos_received_cbk,
            10,
            callback_group=movement_cbk_group,
        )
        #    /\    #
        #   /  \   #
        # ^ Subscribers ^

        # V Service server V
        #   \  /   #
        #    \/    #
        self.iAmAlive = self.create_service(
            Empty, f"leg_{self.leg_num}_alive", (lambda req, res: res)
        )
        self.rel_transl_server = self.create_service(
            Vect3,
            f"leg_{self.leg_num}_rel_transl",
            self.rel_transl_srv_cbk,
            callback_group=movement_cbk_group,
        )
        self.rel_hop_server = self.create_service(
            Vect3,
            f"leg_{self.leg_num}_rel_hop",
            self.rel_hop_srv_cbk,
            callback_group=movement_cbk_group,
        )
        self.shift_server = self.create_service(
            Vect3,
            f"leg_{self.leg_num}_shift",
            self.shift_cbk,
            callback_group=movement_cbk_group,
        )
        self.tipos_server = self.create_service(
            ReturnVect3,
            f"leg_{self.leg_num}_tip_pos",
            self.send_most_recent_tip,
        )
        self.rot_server = self.create_service(
            Vect3,
            f"leg_{self.leg_num}_rot",
            self.rot_cbk,
        )
        #    /\    #
        #   /  \   #
        # ^ Service sever ^

        # V Timers V
        #   \  /   #
        #    \/    #
        self.overwriteTargetTimer = self.create_timer(
            TARGET_TIMEOUT_BEFORE_OVERWRITE, self.overwrite_target
        )
        self.overwriteTargetTimer.cancel()
        #    /\    #
        #   /  \   #
        # ^ Timers ^

    @error_catcher
    def send_most_recent_tip(
        self, request: ReturnVect3.Request, response: ReturnVect3.Response
    ) -> ReturnVect3.Response:
        """Publish the last target sent

        Args:
            request: ReturnVect3.Request - Nothing
            response: ReturnVect3.Response - Vector3 (float x3)

        Returns:
            ReturnVect3 - Vector3 (float x3)
        """
        response.vector.x = self.lastTarget[0]
        response.vector.y = self.lastTarget[1]
        response.vector.z = self.lastTarget[2]
        return response

    @error_catcher
    def rel_transl(self, target: np.ndarray):
        """performs translation to the target relative to body

        Args:
            target: np.array float(3,) - target relative to the body center

        Returns:
            target: np.array float(3,) - target relative to the body center
        """
        samples = int(self.movementTime * self.movementUpdateRate)
        rate = self.create_rate(self.movementUpdateRate)
        start_target = self.lastTarget.copy()
        for x in np.linspace(0 + 1 / samples, 1, num=samples):  # x: ]0->1]
            x = (1 - np.cos(x * np.pi)) / 2
            x = (1 - np.cos(x * np.pi)) / 2
            intermediate_target = target * x + start_target * (1 - x)

            msg = Vector3()
            msg.x, msg.y, msg.z = tuple(intermediate_target.tolist())
            rate.sleep()
            self.ik_pub.publish(msg)

        self.lastTarget = target
        return target

    @error_catcher
    def shift(self, shift: np.ndarray):
        """performs translation to the target relative to current position

        Args:
            target: np.array float(3,) - target relative to current position

        Returns:
            target: np.array float(3,) - target relative to current position
        """
        return self.rel_transl(self.lastTarget + shift)

    @error_catcher
    def rel_hop(self, target: np.ndarray):
        """performs jump to the target relative to body

        Args:
            target: np.array float(3,) - target relative to the body center

        Returns:
            target: np.array float(3,) - target relative to the body center
        """
        samples = int(self.movementTime * self.movementUpdateRate)
        rate = self.create_rate(self.movementUpdateRate)
        start = self.lastTarget.copy()
        for x in np.linspace(0 + 1 / samples, 1, num=samples):  # x: ]0->1]
            x = (1 - np.cos(x * np.pi)) / 2
            z_hop = (np.sin(x * np.pi)) * 100
            x = (1 - np.cos(x * np.pi)) / 2
            intermediate_target = target * x + start * (1 - x)
            intermediate_target[2] += z_hop

            msg = Vector3()
            msg.x, msg.y, msg.z = tuple(intermediate_target.tolist())
            rate.sleep()
            self.ik_pub.publish(msg)

        self.lastTarget = target
        return target

    @error_catcher
    def tip_pos_received_cbk(self, msg: Vector3):
        """callback when a new end effector position is received

        Args:
            msg (Vector3): real end effector position
        """
        self.currentTip[0] = msg.x
        self.currentTip[1] = msg.y
        self.currentTip[2] = msg.z
        self.check_divergence()
        return

    def check_divergence(self):
        """If the real end effector is not on the last target that was sent.
        Launches the timer to overwrite the target with the real position.

        If target and end effector correpsond, cancel the timer to overwrite the target.
        """
        if np.linalg.norm(self.currentTip - self.lastTarget) > TARGET_OVERWRITE_DIST:
            if self.overwriteTargetTimer.is_canceled():
                self.overwriteTargetTimer.reset()
        else:
            self.overwriteTargetTimer.cancel()

    @error_catcher
    def overwrite_target(self):
        """
        overwrites the last sent target with the real position of the end effector.

        This will happen on startup, and when there is a problem on the real leg (cannot
        perform the movement)
        Small divergences are expected (in position and time), hence the timer and the
        check_divergence function.

        Setting lastTarget to the currentTip all the time is a bas idea, it create
        overcorrections.
        """
        self.get_logger().info(f"leg[{self.leg_num}] target overwriten")
        self.lastTarget = self.currentTip
        self.overwriteTargetTimer.cancel()

    @error_catcher
    def shift_cbk(
        self, request: ReturnVect3.Request, response: ReturnVect3.Response
    ) -> ReturnVect3.Response:
        """Callback for leg shift motion

        Args:
            request: target relative to current end effector position
            response:

        Returns:
            success = True all the time
        """
        target = np.array(
            [request.vector.x, request.vector.y, request.vector.z], dtype=float
        )
        self.shift(target)
        time.sleep(WAIT_AFTER_MOTION)
        response.success = SUCCESS
        return response

    @error_catcher
    def rel_transl_srv_cbk(
        self, request: ReturnVect3.Request, response: ReturnVect3.Response
    ) -> ReturnVect3.Response:
        """Callback for leg translation motion

        Args:
            request: target relative to body
            response:

        Returns:
            success = True all the time
        """
        target = np.array(
            [request.vector.x, request.vector.y, request.vector.z], dtype=float
        )
        self.rel_transl(target)
        time.sleep(WAIT_AFTER_MOTION)
        response.success = SUCCESS
        return response

    @error_catcher
    def rel_hop_srv_cbk(
        self, request: ReturnVect3.Request, response: ReturnVect3.Response
    ) -> ReturnVect3.Response:
        """Callback for leg hopping motion

        Args:
            request: target relative to body
            response:

        Returns:
            success = True all the time
        """
        target = np.array(
            [request.vector.x, request.vector.y, request.vector.z], dtype=float
        )

        self.rel_hop(target)
        time.sleep(WAIT_AFTER_MOTION)

        response.success = SUCCESS
        return response


def main(args=None):
    rclpy.init()
    node = LegNode()
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
