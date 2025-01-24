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
from easy_robot_control.EliaNode import Duration, EliaNode, myMain, tf2np
from easy_robot_control.leg_api import Leg
from easy_robot_control.utils.math import qt
from geometry_msgs.msg import TransformStamped
from keyboard_msgs.msg import Key
from motion_stack.core.utils.time import Time
from motion_stack.ros2.utils.conversion import ros_to_time, transform_to_pose
from rclpy.task import Future
from tf2_ros import Buffer, TransformListener

# namespace for keyboard node
operator = str(environ.get("OPERATOR"))
INPUT_NAMESPACE = f"/{operator}"


class SafeAlignArmNode(EliaNode):
    def __init__(self):
        super().__init__("align_node")

        # TF buffer & listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.leg = Leg(number=3, parent=self)

        # -------------- params --------------
        self.declare_parameter("ee_mocap_frame", "mocap3gripper2_straight")
        self.declare_parameter("wheel_mocap_frame", "mocap11_body_offset")
        self.declare_parameter("ee_urdf_frame", "leg3gripper2_straight")
        self.declare_parameter("world_frame", "world")

        # thresholds
        self.declare_parameter("coarse_threshold", 0.01)  # e.g. 1 cm
        self.declare_parameter("fine_threshold", 0.002)  # e.g. 2 mm
        self.declare_parameter("orient_threshold_coarse", 0.1)
        self.declare_parameter("orient_threshold_fine", 0.04)

        self.ee_mocap_frame = self.get_parameter("ee_mocap_frame").value
        self.wheel_mocap_frame = self.get_parameter("wheel_mocap_frame").value
        self.ee_urdf_frame = self.get_parameter("ee_urdf_frame").value
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

        self.task_executor = self.create_timer(0.1, self.run_task)

        # --------------- subscriptions ---------------
        self.key_downSUB = self.create_subscription(
            Key, f"{INPUT_NAMESPACE}/keydown", self.key_downSUBCBK, 10
        )
        self.key_upSUB = self.create_subscription(
            Key, f"{INPUT_NAMESPACE}/keyup", self.key_upSUBCBK, 10
        )

        self.pinfo(
            "SafeAlignArmNode started. Press 'Enter' to start alignment, 'S' for safe pose, 'P' to pause, 'R' to resume, 'F' to finalize."
        )

    def run_task(self):
        self.task()
        if self.task_future.done():
            # del self.task
            self.task = lambda: None
            return

    def safe_pose_task(self) -> Future:
        """
        Moves the arm to a known safe pose & checks if arrived.
        This can be triggered anytime by pressing 'S'.
        """
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
                    "Not enough joints -> can't do safe pose now.",
                    "warn",
                    self.joints_print_interval,
                )
                return

            self.rate_limited_print(
                "Sending arm to safe pose angles.", "info", self.joints_print_interval
            )

            for num, ang in angs.items():
                j = self.leg.get_joint_obj(num)
                if j:
                    j.apply_angle_target(ang)

        def check_safe_pose():
            if future.cancelled():
                self.pinfo("Task cancelled :(")
                return
            if future.done():
                self.pwarn("ugh")
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
                self.safe_pose = True
                self.rate_limited_print(
                    "All joints at safe pose! :)",
                    "info",
                    self.joints_print_interval - 3,
                )
            else:
                self.rate_limited_print(
                    "Still waiting for joints to reach safe pose...",
                    "info",
                    self.joints_print_interval - 3,
                )

        # self.paused = False
        # self.align_approved = False
        # self.aligned = False

        
        self.task = check_safe_pose
        self.task_future = future
        future.add_done_callback()
        return future

    def main_loop(self):
        """Called every 0.5s. Safe-pose check & alignment steps."""
        # to-do Future() implementation
        if self.aligned:
            return

        if not self.safe_pose:
            # self.safe_pose_task()
            return

        if not self.align_approved:
            self.rate_limited_print(
                "Press ENTER to start alignment...", "info", self.align_print_interval
            )
            return

        if self.paused:
            self.rate_limited_print(
                "Alignment paused. Press 'R' to resume, 'F' to finalize, or 'S' to do safe pose again.",
                "info",
                self.align_print_interval,
            )
            return

        # this will be Future()
        done = self.align_step()
        if done:
            self.aligned = True
            self.pinfo("Alignment finished => no more offsets.")

    def align_step(self):
        """
        Looks up transform from ee_mocap_frame -> wheel_mocap_frame,
        uses coarse/fine threshold approach,
        scale speed => bigger step if far, smaller step if near.
        """
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.ee_mocap_frame, self.wheel_mocap_frame, rclpy.time.Time()
            )
            pose_diff = transform_to_pose(tf_msg.transform, ros_to_time(self.getNow()))
            diff_xyz = pose_diff.xyz
            diff_quat = pose_diff.quat

            dist = np.linalg.norm(diff_xyz)
            w_clamped = max(min(abs(diff_quat.w), 1.0), -1.0)
            orient_dist = 2.0 * math.acos(w_clamped)

            self.rate_limited_print(
                f"Align Step => dist={dist:.3f}, orient_diff={math.degrees(orient_dist):.2f} deg",
                "info",
                self.align_print_interval - 3,
            )

            # coarse threshold => re-check with fine
            if (
                dist < self.coarse_threshold
                and orient_dist < self.orient_threshold_coarse
            ):
                self.pinfo("Below coarse thr => wait 1s, re-check fine thr.")
                self.sleep(3.0)
                tf2 = self.tf_buffer.lookup_transform(
                    self.ee_mocap_frame, self.wheel_mocap_frame, rclpy.time.Time()
                )
                pose2 = transform_to_pose(tf2.transform, ros_to_time(self.getNow()))
                dist2 = np.linalg.norm(pose2.xyz)
                w2 = max(min(abs(pose2.quat.w), 1.0), -1.0)
                orient2 = 2.0 * math.acos(w2)

                if dist2 < self.fine_threshold and orient2 < self.orient_threshold_fine:
                    self.pinfo(
                        "Below fine thr => pausing alignment. Press 'R' to continue or 'F' to finalize."
                    )
                    self.paused = True
                    return False
                else:
                    self.pinfo("Not within fine => continuing alignment steps.")

            self.last_distance = dist
            self.last_orient_dist = orient_dist

            # scale offset
            pos_offset_mm, orientation_quat = self.scale_offset(diff_xyz, diff_quat)

            # send offset
            self.leg.ik2.offset(
                xyz=pos_offset_mm, quat=orientation_quat, ee_relative=True
            )

            return False

        except Exception as e:
            self.pwarn(f"align_step error: {e}")
            return True

    def scale_offset(self, diff_xyz, diff_quat):
        """
        If dist>200 => 50mm
        if dist<20 => 5mm
        else linear interpolation
        Orientation => full for now
        """
        dist_mm = np.linalg.norm(diff_xyz) * 1000.0

        max_range = 200.0
        min_range = 20.0
        max_move = 50.0
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

    def key_upSUBCBK(self, msg: Key):
        pass  # handle key up if needed

    def key_downSUBCBK(self, msg: Key):
        key_code = msg.code

        if key_code == Key.KEY_RETURN:
            self.pinfo("User pressed ENTER => alignment can proceed.")
            self.leg.ik2.reset()
            self.align_approved = True

        elif key_code == Key.KEY_P:
            self.paused = True
            self.pinfo("User paused alignment.")
        elif key_code == Key.KEY_R:
            self.paused = False
            self.pinfo("User resumed alignment.")
        elif key_code == Key.KEY_A:
            self.pinfo("User requested single alignment step.")
            self.align_step()
        elif key_code == Key.KEY_F:
            self.pinfo("User finalizing => alignment done.")
            self.aligned = True
        elif key_code == Key.KEY_S:
            self.pinfo("User requested safe pose.")
            self.safe_pose_task()

    def rate_limited_print(self, message: str, level: str, interval: float = 2.0):
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


def main(args=None):
    myMain(SafeAlignArmNode)


if __name__ == "__main__":
    main()
