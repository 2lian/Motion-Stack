#!/usr/bin/env python3

"""
This node is responsible for semi-autonomous (human-in-the-loop) self-assembly of Moonbot Minimal.
Safe pose -> Alignment -> ...

Authors: Shamistan KARIMOV
Lab: SRL, Moonshot team
"""

import math
from os import environ

import numpy as np
import rclpy
from easy_robot_control.EliaNode import EliaNode, error_catcher, myMain, tf2np
from easy_robot_control.gait_key_dev import KeyGaitNode
from easy_robot_control.leg_api import Leg
from easy_robot_control.utils.math import qt
from keyboard_msgs.msg import Key
from motion_stack.ros2.utils.conversion import ros_to_time, transform_to_pose
from rclpy.task import Future
from tf2_ros import Buffer, TransformListener

# namespace for keyboard node
operator = str(environ.get("OPERATOR"))
LEG: int = 4
WHEEL: int = 14
INPUT_NAMESPACE = f"/{operator}"


class SafeAlignArmNode(EliaNode):
    def __init__(self):
        super().__init__("align_node")

        # TF buffer & listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.leg = Leg(number=LEG, parent=self)

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

        self.task_future: Future = Future()
        self.task_future.set_result(True)
        self.task = lambda: None
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
            f"SafeAlignArmNode started. \n'S' => safe pose, 'A' => alignment, '0' => zero position, 'Escape' => input mode"
        )

    @error_catcher
    def run_task(self):
        self.task()
        if self.task_future.done():
            # self.pinfo("future done")
            # del self.task
            self.task = lambda: None
            return

    def safe_pose_task(self) -> Future:
        """
        Moves the arm to a known safe pose & checks if arrived.
        This can be triggered anytime by pressing 'S'.

        Cancels the previous task and future.
        Replaces it with a new function to execute and new future.

        Returns
            Future associated with the check_safe_pose's task.
        """
        self.task_future.cancel()

        self.safe_pose = False
        self.leg.look_for_joints()
        future = Future()

        angs = {
            0: -0.002,
            3: 1.295,
            4: 0.003,
            5: 2.876,
            6: -0.0023,
            7: -0.0041,
            8: -0.00235,
        }
        tolerance = 0.01

        def send_safe_pose():
            if len(self.leg.joints) <= 7:
                self.rate_limited_print(
                    "Not enough joints -> can't do safe pose now. Try again.",
                    "warn",
                    self.joints_print_interval,
                )
                self.waiting_for_input = True
                return

            self.rate_limited_print(
                "Sending arm to safe pose angles.", "info", self.joints_print_interval
            )

            for num, ang in angs.items():
                j = self.leg.get_joint_obj(num)
                if j:
                    j.apply_angle_target(ang)

            return True

        def check_safe_pose():
            if future.cancelled():
                self.pinfo("Task cancelled :(")
                self.task = lambda: None
                return
            if future.done():
                self.pwarn("ugh")
                self.task = lambda: None
                return

            all_ok = True
            for num, ang in angs.items():
                j = self.leg.get_joint_obj(num)
                if not j or j.angle is None:
                    all_ok = False
                    break
                if abs(j.angle - ang) > tolerance:
                    all_ok = False
                    break

            if all_ok:
                self.safe_pose = all_ok
                future.set_result(all_ok)
                self.pinfo("Arm is at the safe pose :)")
            else:
                self.safe_pose = False
                self.rate_limited_print(
                    "Still waiting for joints to reach safe pose...",
                    "info",
                    self.joints_print_interval - 3,
                )

        sent = send_safe_pose()
        if sent:
            self.task = check_safe_pose
            self.task_future = future
            future.add_done_callback(lambda *_: self.wait_for_human_input())
        return future

    def align_task(self) -> Future:
        """
        This method launches two sequential tasks:
         1) coarse_check_task (offsets repeatedly until below coarse threshold)
         2) fine_check_task (settling checks: only offset if conditions are met,
            or finish if distance < fine threshold)

        Returns
            Future associated with the coarse_check task.
        """

        self.task_future.cancel()
        coarse_future = Future()

        if not self.safe_pose:
            self.pwarn("Cannot proceed because arm is not in the safe pose.")
            self.waiting_for_input = True
            return coarse_future

        def coarse_check_task():
            """
            Partial offset approach: if distance/orient > coarse threshold,
            send offset. Repeated in the timer until done.
            """
            if coarse_future.cancelled():
                self.pinfo("Task cancelled :(")
                self.task = lambda: None
                return
            if coarse_future.done():
                self.pwarn("ugh")
                self.task = lambda: None
                return
            if self.waiting_for_input:
                return

            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.ee_mocap_frame, self.wheel_mocap_frame, rclpy.time.Time()
                )
                pose_diff = transform_to_pose(
                    tf_msg.transform, ros_to_time(self.getNow())
                )
                xyz = pose_diff.xyz
                dist = np.linalg.norm(xyz)
                w_clamp = max(min(abs(pose_diff.quat.w), 1.0), -1.0)
                odist = 2.0 * math.acos(w_clamp)

                # if below coarse => we're done with coarse
                if (
                    dist < self.coarse_threshold
                    and odist < self.orient_threshold_coarse
                ):
                    self.pinfo("Coarse check done. Proceeding to fine check.")
                    coarse_future.set_result(True)

                # otherwise send partial offset
                move_xyz_mm, orientation_quat = self.scale_offset(xyz, pose_diff.quat)
                # self.pwarn("check")
                self.leg.ik2.offset(move_xyz_mm, orientation_quat, ee_relative=True)

            except Exception as e:
                self.pwarn(f"coarse_check_task error: {e}")
                coarse_future.set_result(False)

        coarse_future.add_done_callback(lambda _: self.launch_fine_check_task())

        self.task = coarse_check_task
        self.task_future = coarse_future
        return coarse_future

    def launch_fine_check_task(self) -> Future:
        """
        Called once coarse_future is done.
        Creates a new future for fine_check_task,
        which does "settling" logic and partial offset only if motors & mocap are stable.

        Returns
            Future associated with the fine_check task.
        """
        self.task_future.cancel()
        fine_future = Future()

        def fine_check_task():
            """
            Offset if:
              - end-eff TF is not moving
              - motors are not moving
              - distance > fine threshold
            If distance < fine, ee and motors are not moving => finish
            """
            if fine_future.cancelled():
                self.pinfo("Task cancelled :(")
                self.task = lambda: None
                return
            if fine_future.done():
                self.pwarn("ugh")
                self.task = lambda: None
                return
            if self.waiting_for_input:
                return

            try:
                tf2 = self.tf_buffer.lookup_transform(
                    self.ee_mocap_frame, self.wheel_mocap_frame, rclpy.time.Time()
                )
                pose2 = transform_to_pose(tf2.transform, ros_to_time(self.getNow()))
                dist2 = np.linalg.norm(pose2.xyz)
                w2 = max(min(abs(pose2.quat.w), 1.0), -1.0)
                odist2 = 2.0 * math.acos(w2)

                # check if motors are not moving
                motors_stable = self._motors_are_stable()

                # check if mocap is not moving
                stable_tf = self._end_eff_is_stable(pose2.xyz)

                # self.pinfo(f"Motors: {motors_stable}, MoCap EE: {stable_tf}")

                # if stable and close to targer => done
                if (
                    motors_stable
                    and stable_tf
                    and dist2 < self.fine_threshold
                    and odist2 < self.orient_threshold_fine
                ):
                    fine_future.set_result(True)
                    self.aligned = True
                    self.pinfo("Fine check done :)")

                # if stable => send offset
                if motors_stable and stable_tf and dist2 > self.fine_threshold:
                    move_mm, quat = self.scale_offset(pose2.xyz, pose2.quat)
                    self.leg.ik2.offset(move_mm, quat, ee_relative=True)

            except Exception as e:
                self.pwarn(f"fine_check_task error: {e}")
                fine_future.set_result(False)

        self.task = fine_check_task
        self.task_future = fine_future
        fine_future.add_done_callback(lambda *_: self.wait_for_human_input())
        return fine_future

    def grasp_task(self) -> Future:
        """ """
        self.task_future.cancel()
        grip_future = Future()

        if not self.aligned:
            self.pwarn("Cannot proceed because arm is not aligned with the wheel.")
            self.wait_for_human_input()
            return grip_future

        def ee_gripper_open():
            jobj = self.leg.get_joint_obj(2)
            if jobj is None:
                self.pwarn("bruh")
                return
            # jobj.apply_angle_target(-0.35)
            return True

        def ee_grip_check():
            if grip_future.cancelled():
                self.pinfo("Task cancelled :(")
                self.task = lambda: None
                return
            if grip_future.done():
                self.pwarn("ugh")
                self.task = lambda: None
                return

            all_ok = True
            j = self.leg.get_joint_obj(2)
            if not j or j.angle is None:
                all_ok = False
                return
            if abs(j.angle) >= 0.365:
                all_ok = False
                return

            # self.pinfo("yes")

            if all_ok:
                grip_future.set_result(all_ok)
                self.pinfo("Gripper is open :)")
            else:
                self.safe_pose = False
                self.rate_limited_print(
                    "Opening the gripper...",
                    "info",
                    self.joints_print_interval - 3,
                )

        zero = ee_gripper_open()
        if zero:
            self.task = ee_grip_check
            self.task_future = grip_future
            grip_future.add_done_callback(lambda *_: self.launch_grasping())

        return grip_future

    def launch_grasping(self) -> Future:
        """ """
        self.task_future.cancel()
        grasp_future = Future()

        def grasping():
            """ """
            if grasp_future.cancelled():
                self.pinfo("Task cancelled :(")
                self.task = lambda: None
                return
            if grasp_future.done():
                self.pwarn("ugh")
                self.task = lambda: None
                return
            if self.waiting_for_input:
                return

            try:
                tf = self.tf_buffer.lookup_transform(
                    self.ee_mocap_frame, self.wheel_mocap_frame, rclpy.time.Time()
                )
                pose = transform_to_pose(tf.transform, ros_to_time(self.getNow()))
                pose.xyz[0] += 0.33  # haven't tested this offset on real hardware
                dist = np.linalg.norm(pose.xyz)
                w = max(min(abs(pose.quat.w), 1.0), -1.0)
                odist = 2.0 * math.acos(w)

                # check if motors are not moving
                motors_stable = self._motors_are_stable()

                # check if mocap is not moving
                stable_tf = self._end_eff_is_stable(pose.xyz)

                # if stable and close to targer => done
                if (
                    motors_stable
                    and stable_tf
                    and dist < self.fine_threshold
                    and odist < self.orient_threshold_fine
                ):
                    grasp_future.set_result(True)
                    self.grasped = True
                    self.pinfo("Grasping task done :)")

                # if stable => send offset
                if motors_stable and stable_tf and dist > self.fine_threshold:
                    move_mm, quat = self.scale_offset(pose.xyz, pose.quat)
                    self.leg.ik2.offset(move_mm, quat, ee_relative=True)

            except Exception as e:
                self.pwarn(f"grasp_task error: {e}")
                grasp_future.set_result(False)

        self.task = grasping
        self.task_future = grasp_future
        grasp_future.add_done_callback(lambda *_: self.wait_for_human_input())
        return grasp_future

    # ------------- KEYBOARD -------------
    def key_upSUBCBK(self, msg: Key):
        pass  # handle key up if needed

    def key_downSUBCBK(self, msg: Key):
        key_code = msg.code
        key_modifier = msg.modifiers

        if key_code == Key.KEY_ESCAPE:
            self.waiting_for_input = True
            self.paused = True
            self.task_future.cancel()
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
                self.leg.ik2.reset()
                self.align_task()
            elif key_code == Key.KEY_G:
                self.waiting_for_input = False
                self.aligned = True
                self.pinfo("User requested grasping of the wheel.")
                self.leg.ik2.reset()
                self.grasp_task()
            elif key_code == Key.KEY_F:
                self.waiting_for_input = False
                self.safe_pose = True
                self.pinfo("User requested alignment without safe pose check.")
                self.leg.ik2.reset()
                self.align_task()
            elif key_code == Key.KEY_S:
                self.aligned = False
                self.waiting_for_input = False
                self.pinfo("User requested safe pose.")
                self.safe_pose_task()
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
            f"'S' => safe pose, 'A' => alignment, '0' => zero position, 'Escape' => input mode"
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
        If speed was set, sends a speed of 0 instead
        """
        self.leg.ik2.task_future.cancel()
        for joint in self.leg.joints.keys():
            jobj = self.leg.get_joint_obj(joint)
            if jobj is None:
                continue
            if jobj.angle is None:
                continue
            if jobj.last_sent_js.position is None:
                continue

            if jobj._speed_target is None:
                jobj.apply_angle_target(angle=jobj.angle)
            else:
                jobj.apply_speed_target(0)

    def zero_without_grippers(self):
        angs = {
            0: 0.0,
            3: 0.0,
            4: 0.0,
            5: 0.0,
            6: 0.0,
            7: 0.0,
            8: 0.0,
        }
        for num, ang in angs.items():
            jobj = self.leg.get_joint_obj(num)
            if jobj is None:
                continue
            jobj.apply_angle_target(ang)

    def _motors_are_stable(self, vel_tol=0.01):
        """
        Check if all joints have velocity < vel_tol.
        """
        for jn, jobj in self.leg.joints.items():
            # self.pwarn(f"{jobj.speed}")
            if jobj is None or jobj.speed is None:
                continue
            if abs(jobj.speed) > vel_tol:
                return False
        return True

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

        return pos_offset_mm, diff_quat


def main(args=None):
    myMain(SafeAlignArmNode)


if __name__ == "__main__":
    main()
