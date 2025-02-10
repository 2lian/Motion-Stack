#!/usr/bin/env python3

"""
This node is responsible for semi-autonomous (human-in-the-loop) self-assembly of Moonbot Minimal.
Safe pose -> Alignment -> ...

Authors: Shamistan KARIMOV
Lab: SRL, Moonshot team
"""

import math
from os import environ, walk
from typing import Dict

import motion_stack.ros2.ros2_asyncio.ros2_asyncio as rao
import numpy as np
import rclpy
from keyboard_msgs.msg import Key
from motion_stack.api.joint_syncer import SensorSyncWarning
from motion_stack.api.ros2.ik_api import IkHandler, IkSyncerRos
from motion_stack.api.ros2.joint_api import JointHandler, JointSyncerRos
from motion_stack.core.utils.joint_state import JState
from motion_stack.core.utils.math import patch_numpy_display_light
from motion_stack.core.utils.pose import Pose, XyzQuat
from motion_stack.ros2.utils.conversion import ros_now, ros_to_time, transform_to_pose
from motion_stack.ros2.utils.executor import error_catcher, my_main
from rclpy.node import List, Node
from rclpy.task import Future
from tf2_ros import Buffer, TransformListener

patch_numpy_display_light()

# namespace for keyboard node
operator = str(environ.get("OPERATOR"))
LEG: int = 4
WHEEL: int = 14
INPUT_NAMESPACE = f"/{operator}"
ALIAS = "align_node"

LEG_LIST = [LEG]
# LEG_LIST = [1, 2, 3, 4]


class SafeAlignArmNode(Node):
    def __init__(self):
        super().__init__(ALIAS)

        # TF buffer & listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.joint_handlers = [JointHandler(self, l) for l in LEG_LIST]
        self.joint_syncer = JointSyncerRos(self.joint_handlers)
        self.ik_handlers = [IkHandler(self, l) for l in LEG_LIST]
        self.ik_syncer = IkSyncerRos(self.ik_handlers)
        self.startTMR = self.create_timer(0.1, self.startup)
        self.create_timer(1 / 30, self.exec_loop)  # regular execution

        # -------------- params --------------
        self.declare_parameter("ee_mocap_frame", f"mocap{LEG}gripper2_straight")
        self.declare_parameter("wheel_mocap_frame", f"mocap{WHEEL}_body_offset")
        self.declare_parameter("world_frame", "world")

        # thresholds
        self.declare_parameter("coarse_threshold", 0.2)
        self.declare_parameter("fine_threshold", 0.01)
        self.declare_parameter("orient_threshold_coarse", 0.1)
        self.declare_parameter("orient_threshold_fine", 0.03)

        self.ee_mocap_frame = self.get_parameter("ee_mocap_frame").value
        self.wheel_mocap_frame = self.get_parameter("wheel_mocap_frame").value
        self.world_frame = self.get_parameter("world_frame").value
        self.coarse_threshold = self.get_parameter("coarse_threshold").value
        self.fine_threshold = self.get_parameter("fine_threshold").value
        self.orient_threshold_coarse = self.get_parameter(
            "orient_threshold_coarse"
        ).value
        self.orient_threshold_fine = self.get_parameter("orient_threshold_fine").value

        # -------------- flags --------------
        self.safe_pose = False
        self.align_approved = False
        self.paused = False
        self.aligned = False
        self.grasped = False

        self.last_distance = None
        self.last_orient_dist = None

        # anti-spam console logs
        self.last_print_time_align = 0.0  # time of last alignment-step log
        self.align_print_interval = 8.0

        self.last_print_time_joints = 0.0  # time of last "joints waiting" log
        self.joints_print_interval = 8.0

        self.task = lambda: None
        self._last_future: Future = Future()
        self.waiting_for_input = True

        self.task_executor = self.create_timer(0.5, self.run_task)

        # --------------- subscriptions ---------------
        self.key_downSUB = self.create_subscription(
            Key, f"{INPUT_NAMESPACE}/keydown", self.key_downSUBCBK, 10
        )
        self.key_upSUB = self.create_subscription(
            Key, f"{INPUT_NAMESPACE}/keyup", self.key_upSUBCBK, 10
        )

        self.pinfo(
            f"SafeAlignArmNode started. \n'S' => safe pose, 'A' => alignment, '0' => zero position, 'Escape' => input mode, 'C' => close the EE gripper"
        )

    async def joints_ready(self):
        l = [jh.ready for jh in self.joint_handlers]
        try:
            print("Waiting for joints.")
            await rao.wait_for(self, rao.gather(self, *l), timeout_sec=100)
            print(f"Joints ready.")
            strlist = "\n".join(
                [f"limb {jh.limb_number}: {jh.tracked}" for jh in self.joint_handlers]
            )
            # print(f"Joints are:\n{strlist}")
            return
        except TimeoutError:
            raise TimeoutError("Joint data unavailable after 100 sec")

    async def ik_ready(self):
        l = [ih.ready for ih in self.ik_handlers]
        try:
            print("Waiting for ik.")
            await rao.wait_for(self, rao.gather(self, *l), timeout_sec=100)
            print(f"Ik ready.")
            strlist = "\n".join(
                [f"limb {ih.limb_number}: {ih.ee_pose}" for ih in self.ik_handlers]
            )
            # print(f"EE poses are:\n{strlist}")
            return
        except TimeoutError:
            raise TimeoutError("Ik data unavailable after 100 sec")

    @error_catcher
    async def main(self):
        await self.joints_ready()
        await self.ik_ready()
        print("Joint Handlers and IK Handlers are ready.")

    @error_catcher
    def startup(self):
        # rao.ensure_future(self, self.main())
        self.destroy_timer(self.startTMR)

    @error_catcher
    def run_task(self):
        self.task()
        if self.last_future.done():
            # self.pinfo("future done")
            # del self.task
            self.task = lambda: None
            return

    @property
    def last_future(self):
        """The last_future property."""
        return self._last_future

    @last_future.setter
    def last_future(self, value):
        self._last_future.cancel()
        self._last_future = value

    def safe_pose_task(self) -> Future:
        """
        Moves the arm to a known safe pose & checks if arrived.
        This can be triggered anytime by pressing 'S'.

        Cancels the previous task and future.
        Replaces it with a new function to execute and new future.

        Returns
            Future associated with the check_safe_pose's task.
        """
        self.safe_pose = False

        # jh.tracked possible
        target = {
            f"leg{LEG}joint1": 0.0,
            f"leg{LEG}joint2": 1.295,
            f"leg{LEG}joint3": 0.003,
            f"leg{LEG}joint4": 2.876,
            f"leg{LEG}joint5": -0.0023,
            f"leg{LEG}joint6": -0.0041,
            f"leg{LEG}joint7": 0.0,
        }

        self.rate_limited_print(
            "Sending arm to safe pose angles.", "info", self.joints_print_interval
        )

        sent_future = self.joint_syncer.lerp(target)

        def check_safe_pose():
            self.safe_pose = True
            self.pinfo("Arm is at the safe pose :)")
            self.wait_for_human_input()

        sent_future.add_done_callback(lambda *_: check_safe_pose())

        self.last_future = sent_future
        return sent_future

    def align_task(self):
        """
        This method launches two sequential tasks:
         1) coarse_check_task (offsets repeatedly until below coarse threshold)
         2) fine_check_task (settling checks: only offset if conditions are met,
            or finish if distance < fine threshold)

        Returns
            Future associated with the coarse_check task.
        """
        lerp_future = Future()
        coarse_future = Future()

        if not self.safe_pose:
            self.pwarn("Cannot proceed because arm is not in the safe pose.")
            self.wait_for_human_input()
            return

        def coarse_check_task():
            """
            Partial offset approach: if distance/orient > coarse threshold,
            send offset. Repeated in the timer until done.
            """
            nonlocal lerp_future
            if self.waiting_for_input:
                return

            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.ee_mocap_frame, self.wheel_mocap_frame, rclpy.time.Time()
                )
                pose_diff = transform_to_pose(tf_msg.transform, ros_now(self))
                dist = np.linalg.norm(pose_diff.xyz)
                w_clamp = max(min(abs(pose_diff.quat.w), 1.0), -1.0)
                odist = 2.0 * math.acos(w_clamp)

                # if below coarse => we're done with coarse
                if (
                    dist < self.coarse_threshold
                    and odist < self.orient_threshold_coarse
                    and lerp_future.done()
                ):
                    self.pinfo("Coarse check done. Proceeding to fine check.")
                    coarse_future.set_result(True)

                # otherwise send partial offset
                target_pose = self.scale_offset(pose_diff.xyz, pose_diff.quat)
                # self.pwarn(str(target_pose))
                lerp_future = self.ik_syncer.lerp(target_pose)

            except Exception as e:
                self.pwarn(f"coarse_check_task error: {e}")
                coarse_future.set_result(False)

        self.task = coarse_check_task
        self.last_future = coarse_future
        coarse_future.add_done_callback(lambda _: self.launch_fine_check_task())

        return coarse_future

    def launch_fine_check_task(self) -> Future:
        """
        Called once coarse_future is done.
        Creates a new future for fine_check_task,
        which does "settling" logic and partial offset only if motors & mocap are stable.

        Returns
            Future associated with the fine_check task.
        """
        lerp_future = Future()
        lerp_future.set_result(True)
        fine_future = Future()

        def fine_check_task():
            """
            Offset if:
              - end-eff TF is not moving
              - motors are not moving
              - distance > fine threshold
            If distance < fine, ee and motors are not moving => finish
            """
            nonlocal lerp_future
            if self.waiting_for_input:
                return

            try:
                tf2 = self.tf_buffer.lookup_transform(
                    self.ee_mocap_frame, self.wheel_mocap_frame, rclpy.time.Time()
                )
                pose2 = transform_to_pose(tf2.transform, ros_now(self))
                # pose_diff_abs = self.ik_syncer.abs_from_rel({LEG: pose_diff})
                dist2 = np.linalg.norm(pose2.xyz)
                w2 = max(min(abs(pose2.quat.w), 1.0), -1.0)
                odist2 = 2.0 * math.acos(w2)

                # check if mocap is not moving
                stable_tf = self._end_eff_is_stable(pose2.xyz)

                # if stable and close to targer => done
                if (
                    stable_tf
                    and dist2 < self.fine_threshold
                    and odist2 < self.orient_threshold_fine
                    and lerp_future.done()
                ):
                    fine_future.set_result(True)
                    self.aligned = True
                    self.pinfo("Fine check done :)")

                # if stable => send offset
                if stable_tf and dist2 > self.fine_threshold and lerp_future.done():
                    target_pose = self.scale_offset(pose2.xyz, pose2.quat)
                    lerp_future = self.ik_syncer.lerp(target_pose)

            except Exception as e:
                self.pwarn(f"fine_check_task error: {e}")
                fine_future.set_result(False)

        self.task = fine_check_task
        self.last_future = fine_future
        fine_future.add_done_callback(lambda *_: self.wait_for_human_input())
        return fine_future

    def grasp_task(self) -> Future:
        """ """
        self.last_future.cancel()
        grip_future = Future()

        if not self.aligned:
            self.pwarn("Cannot proceed because arm is not aligned with the wheel.")
            self.wait_for_human_input()
            return grip_future

        self.pinfo("Opening the gripper.")
        grip_future = self.open_gripper()

        def grip_done():
            self.pinfo("Gripper has been opened :)")
            self.launch_grasping()

        grip_future.add_done_callback(lambda *_: grip_done())
        self.last_future = grip_future

        return grip_future

    def launch_grasping(self) -> Future:
        """ """
        self.last_future.cancel()
        lerp_future = Future()
        lerp_future.set_result(True)
        grasp_future = Future()

        def grasping():
            """ """
            nonlocal lerp_future
            if self.waiting_for_input:
                return

            try:
                tf = self.tf_buffer.lookup_transform(
                    self.ee_mocap_frame, self.wheel_mocap_frame, rclpy.time.Time()
                )
                pose = transform_to_pose(tf.transform, ros_now(self))
                pose.xyz[0] += 0.33  # haven't tested this offset on real hardware
                dist = np.linalg.norm(pose.xyz)
                w = max(min(abs(pose.quat.w), 1.0), -1.0)
                odist = 2.0 * math.acos(w)

                # check if mocap is not moving
                stable_tf = self._end_eff_is_stable(pose.xyz)

                # if stable and close to targer => done
                if (
                    stable_tf
                    and dist < self.fine_threshold
                    and odist < self.orient_threshold_fine
                    and lerp_future.done()
                ):
                    grasp_future.set_result(True)
                    self.grasped = True
                    self.pinfo("Grasping done :)")

                # if stable => send offset
                if stable_tf and dist > self.fine_threshold and lerp_future.done():
                    target_pose = self.scale_offset(pose.xyz, pose.quat)
                    lerp_future = self.ik_syncer.lerp(target_pose)

            except Exception as e:
                self.pwarn(f"fine_check_task error: {e}")
                grasp_future.set_result(False)

        self.task = grasping
        self.last_future = grasp_future
        grasp_future.add_done_callback(lambda *_: self.wait_for_human_input())
        return grasp_future

    def close_gripper_task(self) -> Future:
        """ """
        self.last_future.cancel()
        grip_future = Future()

        if not self.grasped:
            self.pwarn("Cannot proceed because arm has not grasped the wheel.")
            self.wait_for_human_input()
            return grip_future

        self.pinfo("Closing the gripper.")
        grip_future = self.close_gripper()

        def grip_done():
            self.pinfo("Gripper has been closed :)")
            self.wait_for_human_input()

        grip_future.add_done_callback(lambda *_: grip_done())
        self.last_future = grip_future

        return grip_future

    # ------------- KEYBOARD -------------
    def key_upSUBCBK(self, msg: Key):
        pass  # handle key up if needed

    def key_downSUBCBK(self, msg: Key):
        key_code = msg.code
        key_modifier = msg.modifiers

        if key_code == Key.KEY_ESCAPE:
            self.waiting_for_input = True
            self.paused = True
            self.last_future.cancel()
            self.task = lambda: None
            self.stop_all_joints()
            self.pinfo(
                "Input mode. Ongoing task and associated future has been cancelled."
            )
            self.pinfo(
                f"'S' => safe pose, 'A' => alignment, '0' => zero position, 'Escape' => input mode"
            )

        if self.waiting_for_input:
            if key_code == Key.KEY_A:
                self.waiting_for_input = False
                self.pinfo("User requested alignment.")
                self.align_task()
            elif key_code == Key.KEY_G:
                self.waiting_for_input = False
                self.aligned = True  # dev
                self.pinfo("User requested grasping of the wheel.")
                self.grasp_task()
            elif key_code == Key.KEY_F:
                self.waiting_for_input = False
                self.safe_pose = True
                self.pinfo("User requested alignment without safe pose check.")
                self.align_task()
            elif key_code == Key.KEY_S:
                self.aligned = False
                self.waiting_for_input = False
                self.pinfo("User requested safe pose.")
                self.safe_pose_task()
            elif key_code == Key.KEY_O:
                self.waiting_for_input = False
                self.pinfo("User requested to open the gripper.")
                self.open_gripper()
            elif key_code == Key.KEY_C:
                self.grasped = True  # dev
                self.waiting_for_input = False
                self.pinfo("User requested to close the gripper.")
                self.close_gripper_task()
            elif key_code == Key.KEY_0:
                self.aligned = False
                self.safe_pose = False
                self.waiting_for_input = False
                self.pinfo("User requested zero position.")
                self.zero_without_grippers()

    # ------------- UTILITY -------------
    def wait_for_human_input(self):
        self.waiting_for_input = True
        # self.pinfo("bruh")
        self.pinfo(
            f"'S' => safe pose, 'A' => alignment, '0' => zero position, 'Escape' => input mode, 'C' => close the EE gripper"
        )

    def rate_limited_print(self, message: str, level: str, interval: float):
        """
        Print at most every 'interval' seconds.
        """
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if level == "info":
            if (now_sec - self.last_print_time_joints) > interval:
                self.last_print_time_joints = now_sec
                self.pinfo(message)
        elif level == "warn":
            if (now_sec - self.last_print_time_joints) > interval:
                self.last_print_time_joints = now_sec
                self.pwarn(message)
        else:
            self.pinfo(message)

    def stop_all_joints(self):
        """
        Stops all joint by sending the current angle as target.
        If speed was set, sends a speed of 0 instead.
        """
        self.last_future.cancel()
        self.ik_syncer.last_future.cancel()
        self.joint_syncer.last_future.cancel()

    def zero_without_grippers(self):
        """
        Sends all joints except grippers to zero.
        """
        target = {}
        for jh in self.joint_handlers:
            target.update(
                {
                    jname: 0.0
                    for jname in jh.tracked
                    if "grip1" not in jname and "grip2" not in jname
                }
            )

        self.joint_syncer.lerp(target)

    def open_gripper(self):
        """
        Open the EE gripper.
        """
        target = {}
        for jh in self.joint_handlers:
            target.update({jname: 0.0 for jname in jh.tracked if "grip2" in jname})

        return self.joint_syncer.lerp(target)

    def close_gripper(self):
        """
        Close the EE gripper.
        """
        target = {}
        for jh in self.joint_handlers:
            target.update({jname: -0.025 for jname in jh.tracked if "grip2" in jname})

        return self.joint_syncer.lerp(target)

    # def _motors_are_stable(self, vel_tol=0.01):
    #     """
    #     Check if all joints have velocity < vel_tol.
    #     """
    #     for jn, jobj in self.leg.joints.items():
    #         # self.pwarn(f"{jobj.speed}")
    #         if jobj is None or jobj.speed is None:
    #             continue
    #         if abs(jobj.speed) > vel_tol:
    #             return False
    #     return True

    def _end_eff_is_stable(self, current_xyz: np.ndarray, dist_tol=0.0005):
        """
        Check if end-eff (MoCap) hasn't moved too much since last iteration.
        """
        if self.last_distance is None:
            self.last_distance = np.linalg.norm(current_xyz)
            return False

        new_dist = np.linalg.norm(current_xyz)
        if abs(new_dist - self.last_distance) < dist_tol:
            # self.pwarn("yes")
            return True
        else:
            self.last_distance = new_dist
            return False

    def scale_offset(self, diff_xyz, diff_quat):
        """
        If dist>200 => 30mm
        if dist<60 => 5mm
        else linear interpolation
        Orientation => full for now

        Returns:
            Pose for IK
        """
        dist_mm = np.linalg.norm(diff_xyz) * 1000.0

        max_range = 200.0
        min_range = 30.0
        max_move = 30.0
        min_move = 5.0

        if dist_mm > max_range:
            step_mm = max_move
        elif dist_mm < min_range:
            step_mm = min_move
        else:
            fraction = (dist_mm - min_range) / (max_range - min_range)
            step_mm = min_move + fraction * (max_move - min_move)

        direction = diff_xyz / (1e-12 + np.linalg.norm(diff_xyz))
        pos_offset_mm = direction * step_mm

        new_pose_rel = Pose(ros_now(self), pos_offset_mm, diff_quat)
        # self.pwarn(new_pose_rel)
        self.ik_syncer.clear()
        new_pose_abs = self.ik_syncer.abs_from_rel({LEG: new_pose_rel})
        # self.pwarn(new_pose_abs)

        return new_pose_abs

    # logging
    def perror(self, object):
        self.get_logger().error(f"[{ALIAS}] {object}")

    def pwarn(self, object):
        self.get_logger().warn(f"[{ALIAS}] {object}")

    def pinfo(self, object):
        self.get_logger().info(f"[{ALIAS}] {object}")

    @error_catcher
    def exec_loop(self):
        """Regularly executes the syncers"""
        self.joint_syncer.execute()
        self.ik_syncer.execute()


def main(args=None):
    my_main(SafeAlignArmNode)


if __name__ == "__main__":
    main()
