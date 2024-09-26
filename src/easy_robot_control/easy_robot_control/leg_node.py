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
from typing import Callable, List, Optional, Tuple
from rclpy.time import Duration
from scipy.spatial.transform import Slerp

from EliaNode import EliaNode, error_catcher

import numpy as np
import quaternion as qt
from rclpy.guard_condition import GuardCondition
from scipy.spatial import geometric_slerp
from numpy.typing import NDArray
import rclpy
from rclpy.task import Future
from rclpy.node import Node, Service
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import Transform, Vector3

from rclpy.callback_groups import (
    CallbackGroup,
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)

from custom_messages.srv import Vect3, ReturnVect3, TFService

from easy_robot_control.EliaNode import myMain
# import trajectories as tj


SUCCESS = ""
TARGET_OVERWRITE_DIST = 20
QUAT_OVERWRITE_DIST = 0.05
TARGET_TIMEOUT_BEFORE_OVERWRITE = 1  # seconds
WAIT_AFTER_MOTION = 0.1  # seconds
STEPSIZE = 100


class LegNode(EliaNode):
    def __init__(self):
        # rclpy.init()
        super().__init__(f"ik_node")  # type: ignore

        self.rollFlip = False
        # self.guard_test = self.create_guard_condition(self.guar_test_cbk)

        # V Parameters V
        #   \  /   #
        #    \/    #
        self.WAIT_ANGLE_MES: Duration = Duration(seconds=1)
        self.WAIT_ANGLE_ABORT: Duration = Duration(seconds=3)

        self.declare_parameter("leg_number", 1)
        self.leg_num = (
            self.get_parameter("leg_number").get_parameter_value().integer_value
        )
        if self.leg_num == 1:
            self.Yapping = True
        else:
            self.Yapping = False
        self.Alias = f"L{self.leg_num}"

        self.declare_parameter("std_movement_time", 3.0)
        self.movementTime = (
            self.get_parameter("std_movement_time").get_parameter_value().double_value
        )

        self.declare_parameter("mvmt_update_rate", 30.0)
        self.movementUpdateRate = (
            self.get_parameter("mvmt_update_rate").get_parameter_value().double_value
        )
        #    /\    #
        #   /  \   #
        # ^ Parameters ^

        self.setAndBlockForNecessaryClients(f"ik_{self.leg_num}_alive")

        self.lastTarget: Optional[NDArray] = None
        # self.lastTarget: Optional[NDArray] = np.zeros(3, dtype=float)
        self.lastQuat = qt.one.copy()
        self.currentTipXYZ: Optional[NDArray] = None
        # self.currentTipXYZ = np.zeros(3, dtype=float)
        self.trajectory_q_xyz: NDArray = np.zeros((0, 3), dtype=float)
        self.trajectory_q_quat: qt.quaternion = qt.from_vector_part(
            self.trajectory_q_xyz  # zeros
        )
        self.trajectory_q_roll: NDArray = np.zeros(0, dtype=float)

        # self.

        # V Callback Groups V
        #   \  /   #
        #    \/    #
        movement_cbk_group = ReentrantCallbackGroup()
        self.trajectory_safe_cbkgrp: CallbackGroup = MutuallyExclusiveCallbackGroup()
        #    /\    #
        #   /  \   #
        # ^ Callback Groups ^

        # V Publishers V
        #   \  /   #
        #    \/    #
        self.ik_pub = self.create_publisher(
            Transform, f"set_ik_target_{self.leg_num}", 10
        )
        self.roll_pub = self.create_publisher(Float64, f"roll_{self.leg_num}", 10)
        #    /\    #
        #   /  \   #
        # ^ Publishers ^

        # V Subscribers V
        #   \  /   #
        #    \/    #
        self.tip_pos_sub = self.create_subscription(
            Transform,
            f"tip_pos_{self.leg_num}",
            self.tip_pos_received_cbk,
            10,
        )
        self.smart_roll_sub = self.create_subscription(
            Float64,
            f"smart_roll_{self.leg_num}",
            self.smart_roll_cbk,
            10,
        )
        #    /\    #
        #   /  \   #
        # ^ Subscribers ^

        # V Service server V
        #   \  /   #
        #    \/    #
        self.iAmAlive: Optional[Service] = None

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
        self.point_server = self.create_service(
            TFService, f"leg_{self.leg_num}_point", self.point_cbk
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
            1 / (self.movementUpdateRate * 1.0),
            self.trajectory_executor,
            callback_group=self.trajectory_safe_cbkgrp,
        )
        self.trajectory_timer.cancel()
        self.overwriteTargetTimer = self.create_timer(
            TARGET_TIMEOUT_BEFORE_OVERWRITE,
            self.overwrite_target,
            callback_group=self.trajectory_safe_cbkgrp,
        )
        self.overwriteTargetTimer.cancel()
        self.FisrtTS: Optional[Time] = None
        self.firstSpin = self.create_timer(
            timer_period_sec=0.1,
            callback=self.firstSpinCBK,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        #    /\    #
        #   /  \   #
        # ^ Timers ^

    @error_catcher
    def firstSpinCBK(self):
        if self.FisrtTS is None:
            self.FisrtTS = self.get_clock().now()
        areUnknownAngle = self.lastTarget is None
        if areUnknownAngle:
            sinceLaunch: Duration = self.get_clock().now() - self.FisrtTS
            if sinceLaunch > self.WAIT_ANGLE_MES:
                self.get_logger().warn("Waiting for end effector coordinates", once=True)
            if sinceLaunch > self.WAIT_ANGLE_ABORT:
                self.get_logger().warn("Waited too long, EE assumed zero", once=True)
                self.lastTarget = np.zeros(shape=(3,), dtype=float)
            return  # waits for the first EE position before being ready
        self.iAmAlive = self.create_service(
            Empty, f"leg_{self.leg_num}_alive", (lambda req, res: res)
        )
        self.destroy_timer(self.firstSpin)

    @error_catcher
    def publish_to_roll(self, roll: Optional[float] = None):
        """publishes roll value towards ik node

        Args:
            roll:
        """
        if roll is None:
            # roll = np.nan
            return

        msg = Float64()
        msg.data = float(roll)
        self.roll_pub.publish(msg)

    @error_catcher
    def smart_roll_cbk(self, msg: Float64):
        speed = msg.data
        if self.rollFlip:
            speed *= -1
        self.roll_pub.publish(Float64(data=speed))

    @error_catcher
    def publish_to_ik(
        self, xyz: Optional[NDArray] = None, quat: Optional[qt.quaternion] = None
    ):
        """publishes target towards ik node

        Args:
            target: np.array float(3,)
        """
        if xyz is None:
            if self.lastTarget is None:
                self.perror("No known last leg position")
            xyz = self.lastTarget
        else:
            self.lastTarget = xyz.copy()

        if quat is None:
            quat = self.lastQuat
        else:
            self.lastQuat = quat.copy()

        msg = self.np2tf(xyz, quat)
        self.ik_pub.publish(msg)

    @error_catcher
    def trajectory_executor(self) -> None:
        """pops target from trajectory queue and publishes it.
        This follows and handle the trajectory_timer"""
        xyz, quat = self.pop_xyzq_from_traj()
        roll = self.pop_roll_from_traj()
        if xyz is not None or quat is not None or roll is not None:
            self.publish_to_ik(xyz, quat)
            self.publish_to_roll(roll)
            # self.pinfo(target)
        else:
            self.trajectory_finished_cbk()

    @error_catcher
    def trajectory_finished_cbk(self) -> None:
        """executes after the last target in the trajectory is processed"""
        self.trajectory_timer.cancel()
        return

    def queue_xyz_empty(self) -> bool:
        return self.trajectory_q_xyz.shape[0] <= 0

    def queue_quat_empty(self) -> bool:
        return self.trajectory_q_quat.shape[0] <= 0

    def queue_roll_empty(self) -> bool:
        return self.trajectory_q_roll.shape[0] <= 0

    @error_catcher
    def pop_xyzq_from_traj(
        self, index: int = 0
    ) -> Tuple[Optional[NDArray], Optional[qt.quaternion]]:
        """deletes and returns the first value of the trajectory_queue for coordinates
        and quaternions.

        Args:
            index: if you wanna pop not the first but somewhere else

        Returns:
            value: np.array float(3,) - popped value
        """
        if self.queue_xyz_empty():
            xyz = None
        else:
            xyz = self.trajectory_q_xyz[0, :].copy()
            self.trajectory_q_xyz = np.delete(self.trajectory_q_xyz, index, axis=0)

        if self.queue_quat_empty():
            quat = None
        else:
            quat = self.trajectory_q_quat[0].copy()
            self.trajectory_q_quat = np.delete(self.trajectory_q_quat, index, axis=0)
        return xyz, quat

    @error_catcher
    def pop_roll_from_traj(self, index: int = 0) -> Optional[float]:
        """Deletes and returns the first value of the trajectory_queue for the roll.

        Args:
            index: if you wanna pop not the first but somewhere else

        Returns:
            value: np.array float(3,) - popped value
        """
        if self.queue_roll_empty():
            roll = None
        else:
            roll = self.trajectory_q_roll[0].copy()
            self.trajectory_q_roll = np.delete(self.trajectory_q_roll, index, axis=0)
        return roll

    def get_final_xyz(self) -> NDArray:
        if self.queue_xyz_empty():
            if self.lastTarget is None:
                self.perror("Unknown last leg position for trajectory. Assuming origin.")
                end_point = np.zeros(shape=3, dtype=float)
            else:
                end_point = self.lastTarget
        else:
            end_point = self.trajectory_q_xyz[-1, :]
        return end_point.copy()

    def get_final_quat(self) -> qt.quaternion:
        if self.queue_quat_empty():
            if self.lastQuat is None:
                self.perror("Unknown last leg quat for trajectory. Assuming origin.")
                end_quat = qt.one.copy()
            else:
                end_quat = self.lastQuat
        else:
            end_quat = self.trajectory_q_quat[-1]
        return end_quat.copy()

    def get_final_target(self) -> Tuple[NDArray, qt.quaternion]:
        """returns the final position of where the leg is. Or will be at the end of the
        current trajectory_queue.

        Returns:
            end_point: np.array float(3,) - final coordinates
        """
        return self.get_final_xyz(), self.get_final_quat()

    @error_catcher
    def fuse_xyz_trajectory(self, xyz_traj: Optional[NDArray] = None):
        if xyz_traj is None:
            return

        queue_final_xyz = self.get_final_xyz()
        final_len = max(xyz_traj.shape[0], self.trajectory_q_xyz.shape[0])
        fused_traj = np.zeros((final_len, 3), dtype=float)

        fused_traj[:, :] = queue_final_xyz

        fused_traj[: self.trajectory_q_xyz.shape[0], :] = self.trajectory_q_xyz

        fused_traj[: xyz_traj.shape[0] - 1, :] += xyz_traj[:-1, :] - queue_final_xyz
        fused_traj[xyz_traj.shape[0] - 1 :, :] += xyz_traj[-1, :] - queue_final_xyz

        self.trajectory_q_xyz = fused_traj

    @error_catcher
    def fuse_quat_trajectory(self, quat_traj: Optional[qt.quaternion] = None):
        if quat_traj is None:
            return

        queue_final_quat: qt.quaternion = self.get_final_quat()
        final_len: int = max(quat_traj.shape[0], self.trajectory_q_quat.shape[0])
        fused_traj = qt.from_vector_part(np.zeros((final_len, 3), dtype=float))

        fused_traj[:] = queue_final_quat

        fused_traj[: self.trajectory_q_quat.shape[0]] = self.trajectory_q_quat

        fused_traj[: quat_traj.shape[0] - 1] *= 1 / queue_final_quat * quat_traj[:-1]
        fused_traj[quat_traj.shape[0] - 1 :] *= 1 / queue_final_quat * quat_traj[-1]
        self.trajectory_q_quat = fused_traj

    @error_catcher
    def fuse_roll_trajectory(self, roll_traj: Optional[NDArray] = None):
        if roll_traj is None:
            return

        final_roll_len = max(roll_traj.shape[0], self.trajectory_q_roll.shape[0])
        fused_traj_roll = np.zeros((final_roll_len,), dtype=float)

        fused_traj_roll[: self.trajectory_q_roll.shape[0]] = self.trajectory_q_roll

        fused_traj_roll[: roll_traj.shape[0]] += roll_traj
        self.trajectory_q_roll: NDArray = fused_traj_roll

    @error_catcher
    def add_to_trajectory(
        self,
        xyz_traj: Optional[NDArray] = None,
        quat_traj: Optional[qt.quaternion] = None,
    ) -> None:
        """Adds a trajectory RELATIVE TO THE BODY CENTER to the trajectory queue

        Args:
            new_traj: np.array float(:, 3) - xyz trajectory RELATIVE TO THE BODY CENTER
            quat_traj: qt.quaternion shape(:) - quaternion trajectory RELATIVE TO THE BODY CENTER
        """
        self.fuse_xyz_trajectory(xyz_traj)
        self.fuse_quat_trajectory(quat_traj)

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
        if self.lastTarget is None:
            self.perror("No End Effector positon data. No angle data received yet.")
            return response
        response.vector.x = self.lastTarget[0]
        response.vector.y = self.lastTarget[1]
        response.vector.z = self.lastTarget[2]
        return response

    def smoother(self, x: NDArray) -> NDArray:
        """smoothes the interval [0, 1] to have a soft start and end
        (derivative is zero)
        """
        # x = (1 - np.cos(x * np.pi)) / 2
        # x = (1 - np.cos(x * np.pi)) / 2
        return x

    @error_catcher
    def smoother_complement(self, x: NDArray) -> NDArray:
        """changes the interval [0, 1] to 0->1->0
        and smoothes to have a soft start and end
        """
        x = (1 - np.cos(x * np.pi)) / 2
        # x = x * 1.1 - 0.1
        z = np.sin(x * np.pi)
        return z

    @error_catcher
    def rel_transl(
        self, xyz: Optional[np.ndarray] = None, quat: Optional[qt.quaternion] = None
    ) -> int:
        """performs translation to the target relative to body

        Args:
            target: np.array float(3,) - target relative to the body center
            quat: qt.quaternion - target quaternion relative to body center

        Returns:
            target: np.array float(3,) - target relative to the body center
        """
        samples = int(self.movementTime * self.movementUpdateRate)
        start_target, start_quat = self.get_final_target()

        t = np.linspace(0 + 1 / samples, 1, num=samples - 0)  # x: ]0->1]
        t = self.smoother(t)

        trajectory: Optional[NDArray] = None
        if xyz is not None:
            t3 = np.tile(t, (3, 1)).transpose()
            trajectory = xyz * t3 + start_target * (1 - t3)

        quaternion_slerp: Optional[qt.quaternion] = None
        if quat is not None:
            quaternion_slerp_arr = geometric_slerp(
                start=qt.as_float_array(start_quat),
                end=qt.as_float_array(quat.copy()),
                t=t,
            )
            quaternion_slerp = qt.as_quat_array(quaternion_slerp_arr)

        self.add_to_trajectory(trajectory, quaternion_slerp)
        return samples

    @error_catcher
    def rel_rotation(
        self,
        quat: qt.quaternion,
        center: NDArray = np.array([0, 0, 0], dtype=float),
    ) -> int:
        """performs rotation to the target relative to body

        Args:
            quaternion: qt.quaternion float(4,) - quaternion to rotate by
            center: np.array float(3,) - center of rotation

        Returns:
            steps until end of mvt
        """
        samples = int(self.movementTime * self.movementUpdateRate)
        start_target, start_quat = self.get_final_target()

        x = np.linspace(0 + 1 / samples, 1, num=samples - 0)  # x: ]0->1]
        x = self.smoother(x)
        quaternion_slerp_for_xyz = geometric_slerp(
            start=qt.as_float_array(qt.one.copy()),
            end=qt.as_float_array(quat.copy()),
            t=x,
        )
        quaternion_slerp_for_xyz = qt.as_quat_array(quaternion_slerp_for_xyz)

        trajectory = (
            qt.rotate_vectors(quaternion_slerp_for_xyz, start_target - center) + center
        )

        quaternion_slerp = geometric_slerp(
            start=qt.as_float_array(start_quat),
            end=qt.as_float_array(quat.copy() * start_quat),
            # end=qt.as_float_array(start_quat * quat.copy()),
            t=x,
        )
        quaternion_slerp = qt.as_quat_array(quaternion_slerp)
        self.add_to_trajectory(trajectory, quaternion_slerp)
        return samples

    @staticmethod
    def point_with_quat(vect: NDArray):
        x = vect
        z_pure = np.array([0, 0, 1], dtype=float)
        y = np.cross(z_pure, x)  # to define y in the horizontal plane
        z = np.cross(x, y)
        x_is_z_or_zero = np.isclose(np.linalg.norm(y), 0)
        if x_is_z_or_zero:
            return 0
        x, y, z = x / np.linalg.norm(x), y / np.linalg.norm(y), z / np.linalg.norm(z)

        rot_matrix = np.empty((3, 3), dtype=float)
        rot_matrix[0, :] = x
        rot_matrix[1, :] = y
        rot_matrix[2, :] = z

        rot: qt.quaternion = 1 / qt.from_rotation_matrix(rot_matrix)
        # IDK why the -1
        return rot

    @error_catcher
    def point_toward(self, vect: NDArray) -> int:
        rotDirect = self.point_with_quat(vect)
        rotFlip = self.point_with_quat(-vect)
        deltaDirect = self.get_final_quat() / rotDirect
        deltaFlip = self.get_final_quat() / rotFlip
        if np.linalg.norm(
            qt.as_float_array(deltaDirect) - qt.as_float_array(qt.one)
        ) < np.linalg.norm(qt.as_float_array(deltaFlip) - qt.as_float_array(qt.one)):
            rot = rotDirect
            self.rollFlip = False
        else:
            rot = rotFlip
            self.rollFlip = True

        allready_oriented = qt.isclose(self.get_final_quat(), rot, atol=0.01)
        if allready_oriented:
            rot = self.get_final_quat()
            steps_until_done = 0
        else:
            steps_until_done = self.rel_transl(xyz=None, quat=rot)
        return steps_until_done

    @error_catcher
    def point_wheel(self, direction: NDArray) -> None:
        # this rotates the end effector and we get the final quaternion
        until_rot_done: int = self.point_toward(direction)

        # distance = np.linalg.norm(direction)
        # samples = int(self.movementTime * self.movementUpdateRate)
        # traj = np.zeros(until_rot_done + samples)
        # traj[until_rot_done:] = distance / samples
        # self.fuse_roll_trajectory(traj)

    @error_catcher
    def shift(
        self, shift: Optional[NDArray] = None, quat: Optional[qt.quaternion] = None
    ) -> int:
        """performs translation to the target relative to current position

        Args:
            target: np.array float(3,) - target relative to current position

        Returns:
            target: np.array float(3,) - target relative to current position
        """
        finalXYZ, finalQUAT = self.get_final_target()
        newShift: Optional[NDArray]
        newQuat: Optional[NDArray]

        if shift is None:
            newShift = None
        else:
            newShift = finalXYZ + shift

        if quat is None:
            newQuat = None
        else:
            newQuat = finalQUAT * quat

        return self.rel_transl(newShift, newQuat)

    @error_catcher
    def rel_hop(self, target: np.ndarray) -> NDArray:
        """performs jump to the target relative to body

        Args:
            target: np.array float(3,) - target relative to the body center

        Returns:
            target: np.array float(3,) - target relative to the body center
        """
        samples = int(self.movementTime * self.movementUpdateRate)
        start_target, start_quat = self.get_final_target()

        x = np.linspace(0 + 1 / samples, 1, num=samples - 0)  # x: ]0->1]
        z = self.smoother_complement(x)
        x = self.smoother(x)
        x = np.tile(x, (3, 1)).transpose()
        trajectory = target * x + start_target * (1 - x)
        trajectory[:, 2] += z * STEPSIZE

        self.add_to_trajectory(trajectory)
        return trajectory[-1, :].copy()

    @error_catcher
    def tip_pos_received_cbk(self, msg: Transform) -> None:
        """callback when a new end effector position is received

        Args:
            msg (Vector3): real end effector position
        """
        self.currentTipXYZ, self.currentTipQuat = self.tf2np(msg)
        self.check_divergence()
        return

    @error_catcher
    def check_divergence(self) -> None:  # TODO: quaternions also
        """If the real end effector is not on the last target that was sent.
        Launches the timer to overwrite the target with the real position.

        If target and end effector correpsond, cancel the timer to overwrite the target.
        """
        if self.currentTipXYZ is None:
            self.pwarn("Tip positon unknown")
            return

        noTargetData = self.lastTarget is None
        if noTargetData:
            self.overwrite_target()
        if (
            np.linalg.norm(self.currentTipXYZ - self.lastTarget)
            > TARGET_OVERWRITE_DIST
            # or not np.all(
            # qt.isclose(self.currentTipQuat, self.lastQuat, atol=QUAT_OVERWRITE_DIST)
            # )
            # or np.linalg.norm(qt.as_float_array(self.currentTipQuat - self.lastQuat)) > QUAT_OVERWRITE_DIST
        ) and self.queue_xyz_empty():
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
        if self.currentTipXYZ is None:
            self.pwarn("Tip positon unknown")
            return

        if self.lastTarget is None:
            self.pinfo(
                f"Initial EE coord: v{np.round(self.currentTipXYZ)}, q{np.round(qt.as_float_array(self.currentTipQuat), 2)}",
                force=True,
            )
        else:
            self.pinfo(
                f"EE coord overwriten: v{np.round(self.currentTipXYZ)}, q{np.round(qt.as_float_array(self.currentTipQuat), 2)}",
                force=True,
            )
        self.lastTarget = self.currentTipXYZ
        self.lastQuat = self.currentTipQuat
        self.overwriteTargetTimer.cancel()

    @error_catcher
    def wait_end_of_motion(self) -> None:
        """waits for the trajectory to end. This function is very bad, but I don't need
        nor have time to do something better.

        We should keep track of which trajectory are beeing queued to improve"""
        self.sleep(self.movementTime + 0.0)

    def append_trajectory(self, trajectory_function: Callable) -> Future:
        """The function will be executed before the next read of the trajectory sequentialy
        This avoids trajectory_function in multiple threads modifying indentical data simultaneously.

        Args:
            trajectory_function - function: function to execute
        Returns:
            future: holds the future results of the function if needed
        """
        future = Future()

        def fun_with_future():
            result = trajectory_function()
            if self.trajectory_timer.is_canceled():
                self.trajectory_timer.reset()

                # this starts execution immediately,
                # not waiting for the first timer callback
                self.execute_in_cbk_group(
                    self.trajectory_executor, self.trajectory_safe_cbkgrp
                )
            future.set_result(result)
            return result

        # self.trajectory_update_queue.append(fun_with_future)
        self.execute_in_cbk_group(fun_with_future, self.trajectory_safe_cbkgrp)
        return future

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
        target: Optional[NDArray]
        quat: Optional[qt.quaternion]
        target, quat = self.tf2np(request.tf)

        if np.any(np.isnan(target)):
            target = None
        if np.any(np.isnan(qt.as_float_array(quat))):
            quat = None

        fun = lambda: self.shift(target, quat)
        self.append_trajectory(fun)

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
        target: Optional[NDArray]
        quat: Optional[qt.quaternion]
        target, quat = self.tf2np(request.tf)

        if np.any(np.isnan(target)):
            target = None
        if np.any(np.isnan(qt.as_float_array(quat))):
            quat = None

        fun = lambda: self.rel_transl(target, quat)
        self.append_trajectory(fun)

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
        target: Optional[NDArray]
        quat: Optional[qt.quaternion]
        target, quat = self.tf2np(request.tf)

        if np.any(np.isnan(target)):
            target = None
        if np.any(np.isnan(qt.as_float_array(quat))):
            quat = None

        fun = lambda: self.rel_hop(target)
        self.append_trajectory(fun)

        self.wait_end_of_motion()
        response.success_str.data = SUCCESS
        return response

    @error_catcher
    def rot_cbk(
        self, request: TFService.Request, response: TFService.Response
    ) -> TFService.Response:
        """Callback for leg rotation motion

        Args:
            request: TF for the rotation and center of the rotation
            response:

        Returns:
            success = True all the time
        """
        center, quat = self.tf2np(request.tf)

        fun = lambda: self.rel_rotation(quat, center)
        self.append_trajectory(fun)

        self.wait_end_of_motion()
        response.success_str.data = SUCCESS
        return response

    def point_cbk(
        self, request: TFService.Request, response: TFService.Response
    ) -> TFService.Response:
        roll_transl, quat = self.tf2np(request.tf)

        fun = lambda: self.point_wheel(roll_transl)
        self.append_trajectory(fun)

        self.wait_end_of_motion()
        response.success_str.data = SUCCESS
        return response


def main(args=None):
    myMain(LegNode, multiThreaded=True)


if __name__ == "__main__":
    main()
