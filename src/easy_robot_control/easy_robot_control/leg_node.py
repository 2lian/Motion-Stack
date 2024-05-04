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
from types import FunctionType, LambdaType
from typing import Tuple

import numpy as np
import quaternion as qt
from rclpy.guard_condition import GuardCondition
from scipy.spatial import geometric_slerp
from numpy.typing import NDArray
import rclpy
from rclpy.task import Future
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3

from rclpy.callback_groups import (
    CallbackGroup,
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)

from custom_messages.srv import Vect3, ReturnVect3, TFService

SUCCESS = ""
TARGET_OVERWRITE_DIST = 50
TARGET_TIMEOUT_BEFORE_OVERWRITE = 1  # seconds
WAIT_AFTER_MOTION = 0.1  # seconds
STEPSIZE = 100


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
                raise KeyboardInterrupt
        return out

    return wrap


class LegNode(Node):
    def __init__(self):
        # rclpy.init()
        super().__init__(f"ik_node")  # type: ignore

        self.guard_grp = MutuallyExclusiveCallbackGroup()
        # self.guard_test = self.create_guard_condition(self.guar_test_cbk)

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
        self.trajectory_queue = np.zeros((0, 3), dtype=float)

        # V Callback Groups V
        #   \  /   #
        #    \/    #
        movement_cbk_group = ReentrantCallbackGroup()
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
            TFService,
            f"leg_{self.leg_num}_rel_transl",
            self.rel_transl_srv_cbk,
            callback_group=movement_cbk_group,
        )
        self.rel_hop_server = self.create_service(
            TFService,
            f"leg_{self.leg_num}_rel_hop",
            self.rel_hop_srv_cbk,
            callback_group=movement_cbk_group,
        )
        self.shift_server = self.create_service(
            TFService,
            f"leg_{self.leg_num}_shift",
            self.shift_cbk,
            callback_group=movement_cbk_group,
        )
        self.rot_server = self.create_service(
            TFService,
            f"leg_{self.leg_num}_rot",
            self.rot_cbk,
            callback_group=movement_cbk_group,
        )
        self.tipos_server = self.create_service(
            ReturnVect3,
            f"leg_{self.leg_num}_tip_pos",
            self.send_most_recent_tip,
        )
        #    /\    #
        #   /  \   #
        # ^ Service sever ^

        # V Timers V
        #   \  /   #
        #    \/    #
        self.trajectory_timer = self.create_timer(
            1 / self.movementUpdateRate,
            self.trajectory_executor,
            callback_group=self.guard_grp,
        )
        self.trajectory_timer.cancel()
        self.overwriteTargetTimer = self.create_timer(
            TARGET_TIMEOUT_BEFORE_OVERWRITE,
            self.overwrite_target,
            callback_group=self.guard_grp,
        )
        self.overwriteTargetTimer.cancel()
        #    /\    #
        #   /  \   #
        # ^ Timers ^

    @error_catcher
    def print(self, s: str):
        if self.leg_num == 0:
            self.get_logger().warn(s)

    @error_catcher
    def publish_to_ik(self, target: NDArray):
        """publishes target towards ik node

        Args:
            target: np.array float(3,)
        """
        msg = Vector3()
        msg.x, msg.y, msg.z = tuple(target.tolist())
        self.ik_pub.publish(msg)
        self.lastTarget = target

    @error_catcher
    def trajectory_executor(self) -> None:
        """pops target from trajectory queue and publishes it.
        This follows and handle the trajectory_timer"""
        target = self.pop_coord_from_trajectory()
        if target is not None:
            # if self.leg_num is 0:
            # self.get_logger().info(f"{np.round(target)}")
            self.publish_to_ik(target)
        else:
            self.trajectory_finished_cbk()

    @error_catcher
    def trajectory_finished_cbk(self) -> None:
        """executes after the last target in the trajectory is process"""
        self.trajectory_timer.cancel()

    @error_catcher
    def queue_empty(self) -> bool:
        return self.trajectory_queue.shape[0] <= 0

    @error_catcher
    def pop_coord_from_trajectory(self, index: int = 0) -> NDArray | None:
        """deletes and returns the first value of the trajectory_queue.

        Args:
            index: if you wanna pop not the first but somewhere else

        Returns:
            value: np.array float(3,) - popped value
        """
        if self.queue_empty():
            return None
        val = self.trajectory_queue[0]
        self.trajectory_queue = np.delete(self.trajectory_queue, index, axis=0)
        return val

    @error_catcher
    def get_final_target(self) -> NDArray:
        """returns the final position of where the leg is. Or will be at the end of the
        current trajectory_queue.

        Returns:
            end_point: np.array float(3,) - final coordinates
        """
        if self.queue_empty():
            end_point = self.lastTarget
        else:
            end_point = self.trajectory_queue[-1, :]
        return end_point

    @error_catcher
    def add_to_trajectory(self, new_traj: NDArray) -> None:
        """Adds a trajectory RELATIVE TO THE BODY CENTER to the trajectory queue

        Args:
            new_traj: np.array float(:, 3) - trajectory RELATIVE TO THE BODY CENTER
        """
        queue_final_coord = self.get_final_target()
        # self.print(f"{np.round(queue_final_coord)}")
        fused_traj = np.empty(
            (max(new_traj.shape[0], self.trajectory_queue.shape[0]), 3), dtype=float
        )
        fused_traj[:, :] = queue_final_coord

        fused_traj[: self.trajectory_queue.shape[0], :] = self.trajectory_queue

        fused_traj[: new_traj.shape[0], :] += new_traj - queue_final_coord
        fused_traj[new_traj.shape[0] - 1 :, :] = new_traj[-1, :]

        self.trajectory_queue = fused_traj

    @error_catcher
    def start_trajectory(self, new_traj: NDArray) -> None:
        """Adds a trajectory RELATIVE TO THE BODY CENTER to the trajectory queue and
        begins the execution of the queue.

        Args:
            new_traj: np.array float(:, 3) - trajectory RELATIVE TO THE BODY CENTER
        """
        self.add_to_trajectory(new_traj)
        if self.trajectory_timer.is_canceled():
            self.trajectory_timer.reset()

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
    def smoother(self, x: NDArray) -> NDArray:
        """smoothes the interval [0, 1] to have a soft start and end
        (derivative is zero)
        """
        x = (1 - np.cos(x * np.pi)) / 2
        x = (1 - np.cos(x * np.pi)) / 2
        return x

    @error_catcher
    def smoother_complement(self, x: NDArray) -> NDArray:
        """changes the interval [0, 1] to 0->1->0
        and smoothes to have a soft start and end
        """
        x = (1 - np.cos(x * np.pi)) / 2
        z = np.sin(x * np.pi)
        return z

    @error_catcher
    def rel_transl(self, target: np.ndarray):
        """performs translation to the target relative to body

        Args:
            target: np.array float(3,) - target relative to the body center

        Returns:
            target: np.array float(3,) - target relative to the body center
        """
        # self.print(f"rel_transl exe")
        samples = int(self.movementTime * self.movementUpdateRate)
        start_target = self.get_final_target()

        x = np.linspace(0 + 1 / samples, 1, num=samples - 1)  # x: ]0->1]
        x = self.smoother(x)
        x = np.tile(x, (3, 1)).transpose()
        trajectory = target * x + start_target * (1 - x)

        # self.get_logger().warn(f"{np.round(trajectory)}")
        self.start_trajectory(trajectory)
        return trajectory[-1, :]

    @error_catcher
    def rel_rotation(
        self,
        quaternion: qt.quaternion,
        center: NDArray = np.array([0, 0, 0], dtype=float),
    ) -> NDArray:
        """performs rotation to the target relative to body

        Args:
            quaternion: qt.quaternion float(4,) - quaternion to rotate by
            center: np.array float(3,) - center of rotation

        Returns:
            final target: np.array float(3,) - final target relative to the body center
        """
        # self.print(f"rel_rot exe")
        samples = int(self.movementTime * self.movementUpdateRate)
        start_target = self.get_final_target()

        x = np.linspace(0 + 1 / samples, 1, num=samples - 1)  # x: ]0->1]
        x = self.smoother(x)
        quaternion_interpolation = geometric_slerp(
            start=qt.as_float_array(qt.one), end=qt.as_float_array(quaternion), t=x
        )
        quaternion_interpolation = qt.as_quat_array(quaternion_interpolation)

        trajectory = (
            qt.rotate_vectors(quaternion_interpolation, start_target + center) - center
        )

        self.start_trajectory(trajectory)
        return trajectory[-1, :]

    @error_catcher
    def shift(self, shift: np.ndarray) -> NDArray:
        """performs translation to the target relative to current position

        Args:
            target: np.array float(3,) - target relative to current position

        Returns:
            target: np.array float(3,) - target relative to current position
        """
        return self.rel_transl(self.get_final_target() + shift)

    @error_catcher
    def rel_hop(self, target: np.ndarray) -> NDArray:
        """performs jump to the target relative to body

        Args:
            target: np.array float(3,) - target relative to the body center

        Returns:
            target: np.array float(3,) - target relative to the body center
        """
        samples = int(self.movementTime * self.movementUpdateRate)
        start_target = self.get_final_target()

        x = np.linspace(0 + 1 / samples, 1, num=samples - 1)  # x: ]0->1]
        z = self.smoother_complement(x)
        x = self.smoother(x)
        x = np.tile(x, (3, 1)).transpose()
        trajectory = target * x + start_target * (1 - x)
        trajectory[:, 2] += z * STEPSIZE

        self.start_trajectory(trajectory)
        return trajectory[-1, :]

    @error_catcher
    def tip_pos_received_cbk(self, msg: Vector3) -> None:
        """callback when a new end effector position is received

        Args:
            msg (Vector3): real end effector position
        """
        self.currentTip[0] = msg.x
        self.currentTip[1] = msg.y
        self.currentTip[2] = msg.z
        self.check_divergence()
        return

    @error_catcher
    def check_divergence(self) -> None:
        """If the real end effector is not on the last target that was sent.
        Launches the timer to overwrite the target with the real position.

        If target and end effector correpsond, cancel the timer to overwrite the target.
        """
        if (
            np.linalg.norm(self.currentTip - self.lastTarget) > TARGET_OVERWRITE_DIST
            and self.queue_empty()
        ):
            if self.overwriteTargetTimer.is_canceled():
                self.overwriteTargetTimer.reset()

        else:
            self.overwriteTargetTimer.cancel()

    @error_catcher
    def overwrite_target(self) -> None:
        """
        overwrites the last sent target with the real position of the end effector.

        This will happen on startup, and when there is a problem on the real leg (cannot
        perform the movement)
        Small divergences are expected (in position and time), hence the timer and the
        check_divergence function.

        Setting lastTarget to the currentTip all the time is a bas idea, it create
        overcorrections.
        """
        self.get_logger().info(
            f"leg[{self.leg_num}] target overwriten with {np.round(self.currentTip)}"
        )
        self.lastTarget = self.currentTip
        self.overwriteTargetTimer.cancel()

    @error_catcher
    def wait_end_of_motion(self) -> None:
        """waits for the trajectory to end. This function is very bad, but I don't need
        nor have time to do something better.

        We should keep track of which trajectory are beeing queued to improve"""
        rate = self.create_rate(1 / (self.movementTime + 0.0))
        rate.sleep()

    def execute_in_cbk_group(
        self, fun: FunctionType | LambdaType, callback_group: CallbackGroup | None = None
    ) -> Tuple[Future, GuardCondition]:
        """Executes the given function by adding it as a callback to a callback_group.
        This avoids function in multiple threads modifying indentical data simultaneously.

        Not gonna lie, I think that's not how it should be done.

        Args:
            callback_group - CallbackGroup: callback group in which to execute the function
            fun - function: function to execute
        Returns:
            future: holds the future results
            quardian: the guard condition object from the other callback_group
        """
        if callback_group is None:
            callback_group = self.guard_grp

        future = Future()
        def fun_with_future():
            if not future.done():  # guardian triggers several times, idk why.
                # so this condition protects this mess from repeting itselfs randomly
                future.set_result(fun())

        # guardian will execute fun_with_future inside of callback_group
        guardian = self.create_guard_condition(fun_with_future, callback_group)
        guardian.trigger()
        future.add_done_callback(lambda result: guardian.destroy())
        return future, guardian

    @error_catcher
    def shift_cbk(
        self, request: TFService.Request, response: TFService.Response
    ) -> TFService.Response:
        """Callback for leg shift motion

        Args:
            request: target relative to current end effector position
            response:

        Returns:
            success = True all the time
        """
        target = np.array(
            [
                request.tf.translation.x,
                request.tf.translation.y,
                request.tf.translation.z,
            ],
            dtype=float,
        )
        quat = qt.from_float_array(
            [
                request.tf.rotation.w,
                request.tf.rotation.x,
                request.tf.rotation.y,
                request.tf.rotation.z,
            ]
        )

        fun = lambda: self.shift(target)
        self.execute_in_cbk_group(fun)
        # guard_test = self.create_guard_condition(
        # lambda: self.shift(target), self.guard_grp
        # )
        # guard_test.trigger()
        # self.get_logger().warn(f"shift trans exe: {self.guard_grp.can_execute(guard_test)}")
        self.wait_end_of_motion()
        response.success_str.data = SUCCESS
        return response

    @error_catcher
    def rel_transl_srv_cbk(
        self, request: TFService.Request, response: TFService.Response
    ) -> TFService.Response:
        """Callback for leg translation motion

        Args:
            request: target relative to body
            response:

        Returns:
            success = True all the time
        """
        target = np.array(
            [
                request.tf.translation.x,
                request.tf.translation.y,
                request.tf.translation.z,
            ],
            dtype=float,
        )
        quat = qt.from_float_array(
            [
                request.tf.rotation.w,
                request.tf.rotation.x,
                request.tf.rotation.y,
                request.tf.rotation.z,
            ]
        )

        fun = lambda: self.rel_transl(target)
        self.execute_in_cbk_group(fun)

        self.wait_end_of_motion()
        response.success_str.data = SUCCESS
        return response

    @error_catcher
    def rel_hop_srv_cbk(
        self, request: TFService.Request, response: TFService.Response
    ) -> TFService.Response:
        """Callback for leg hopping motion

        Args:
            request: target relative to body
            response:

        Returns:
            success = True all the time
        """
        target = np.array(
            [
                request.tf.translation.x,
                request.tf.translation.y,
                request.tf.translation.z,
            ],
            dtype=float,
        )
        quat = qt.from_float_array(
            [
                request.tf.rotation.w,
                request.tf.rotation.x,
                request.tf.rotation.y,
                request.tf.rotation.z,
            ]
        )

        fun = lambda: self.rel_hop(target)
        self.execute_in_cbk_group(fun)

        self.wait_end_of_motion()
        response.success_str.data = SUCCESS
        return response

    @error_catcher
    def rot_cbk(
        self, request: TFService.Request, response: TFService.Response
    ) -> TFService.Response:
        """Callback for leg translation motion

        Args:
            request: TF for the rotation and center of the rotation
            response:

        Returns:
            success = True all the time
        """
        center = np.array(
            [
                request.tf.translation.x,
                request.tf.translation.y,
                request.tf.translation.z,
            ],
            dtype=float,
        )
        quat = qt.from_float_array(
            [
                request.tf.rotation.w,
                request.tf.rotation.x,
                request.tf.rotation.y,
                request.tf.rotation.z,
            ]
        )

        fun = lambda: self.rel_rotation(quat, center)
        self.execute_in_cbk_group(fun)

        self.wait_end_of_motion()
        response.success_str.data = SUCCESS
        return response


def main(args=None):
    rclpy.init()
    node = LegNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().debug(
            "KeyboardInterrupt caught, node shutting down cleanly\nbye bye <3"
        )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
