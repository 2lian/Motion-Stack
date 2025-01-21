#!/usr/bin/env python3

"""
SafeAlignArmNode 
----------------
1) Moves the arm to a "safe pose."
2) Waits for user "enter" or keyboard input to start alignment
3) Aligns the end-effector (mocap3gripper2_straight) to a target 
   (mocap11_body_offset) in position & orientation:
   - We skip local coordinate transforms because 
     the mocap frames match the real orientation
4) Implements "fast when far, slow when close" speed scaling
5) Uses coarse/fine threshold approach
6) Keyboard commands let you re-run alignment or finalize manually
7) Limits console spam by printing "Align Step" info only occasionally
"""

import math
import threading
import time
from os import environ

import numpy as np
import rclpy
from easy_robot_control.EliaNode import Duration, EliaNode, myMain, tf2np
from easy_robot_control.leg_api import Leg
from easy_robot_control.utils.math import qt
from geometry_msgs.msg import TransformStamped
from keyboard_msgs.msg import Key

# If your code uses these or remove them if not needed:
from motion_stack.core.utils.time import Time
from motion_stack.ros2.utils.conversion import ros_to_time, transform_to_pose
from tf2_ros import Buffer, TransformListener

# namespace
operator = str(environ.get("OPERATOR"))
INPUT_NAMESPACE = f"/{operator}"


class SafeAlignArmNode(EliaNode):
    def __init__(self):
        super().__init__("align_node")

        # TF buffer & listener for transform lookups
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a Leg object for controlling the real arm
        self.leg = Leg(number=3, parent=self)

        # -------------- Parameters --------------
        self.declare_parameter("ee_mocap_frame", "mocap3gripper2_straight")
        self.declare_parameter("wheel_mocap_frame", "mocap11_body_offset")
        self.declare_parameter("ee_urdf_frame", "leg3gripper2_straight")
        self.declare_parameter("world_frame", "world")

        # Thresholds
        self.declare_parameter("coarse_threshold", 0.01)  # e.g. 5 cm
        self.declare_parameter("fine_threshold", 0.002)  # e.g. 1 cm
        self.declare_parameter("orient_threshold_coarse", 0.1)  # ~5.7 deg
        self.declare_parameter("orient_threshold_fine", 0.05)  # ~2.86 deg

        # Retrieve param values
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

        # -------------- Internal State --------------
        self.safe_pose = False  # set after safe pose is reached
        self.user_approved = False
        self.paused = False
        self.aligned = False  # final or user "f" confirm

        self.last_distance = None
        self.last_orient_dist = None

        # For limiting console spam
        self.last_print_time = 0.0  # track the last time we printed
        self.print_interval = 4.0  # seconds between prints

        # -------------- Timers & Input --------------
        threading.Thread(target=self.wait_for_user_input, daemon=True).start()

        self.main_timer = self.create_timer(0.5, self.main_loop)

        # Subscribe to keyboard down/up
        self.key_downSUB = self.create_subscription(
            Key, f"{INPUT_NAMESPACE}/keydown", self.key_downSUBCBK, 10
        )
        self.key_upSUB = self.create_subscription(
            Key, f"{INPUT_NAMESPACE}/keyup", self.key_upSUBCBK, 10
        )

        self.pinfo(
            "SafeAlignArmNode started. No local transform, new scaling approach."
        )

    # -------------------------------------------------------------------------
    # 1) Safe Pose
    # -------------------------------------------------------------------------
    def go_to_safe_pose(self):
        """
        Moves the arm to a known safe pose & checks if arrived.
        """
        self.leg.look_for_joints()
        if len(self.leg.joints) <= 7:
            self.pwarn("Not enough joints -> can't do safe pose now.")
            return

        self.pinfo("Sending arm to safe pose angles.")
        angs = {
            0: -0.002,
            3: 1.295,
            4: 0.003,
            5: 2.876,
            6: -0.0023,
            7: -0.0041,
            8: -0.00235,
        }
        for num, ang in angs.items():
            j = self.leg.get_joint_obj(num)
            if j:
                j.apply_angle_target(ang)

        # Quick check
        tolerance = 0.01
        all_ok = True
        for num, tgt in angs.items():
            j = self.leg.get_joint_obj(num)
            if not j or j.angle is None:
                all_ok = False
                break
            if abs(j.angle - tgt) > tolerance:
                all_ok = False
                break

        if all_ok:
            self.safe_pose = True
            self.pinfo("All joints at safe pose!")
        else:
            self.pinfo("Still waiting for joints to reach safe pose.")

    # -------------------------------------------------------------------------
    # 2) Wait for user "Enter"
    # -------------------------------------------------------------------------
    def wait_for_user_input(self):
        input("\n=== Press Enter to proceed with alignment ===\n")
        self.pinfo("User pressed Enter => alignment stage can proceed.")
        self.leg.ik2.reset()
        self.user_approved = True

    # -------------------------------------------------------------------------
    # 3) main_loop
    # -------------------------------------------------------------------------
    def main_loop(self):
        if self.aligned:
            return

        if not self.safe_pose:
            self.go_to_safe_pose()
            return

        if not self.user_approved:
            self.pinfo("Waiting user input to align...")
            return

        if self.paused:
            self.pinfo("Paused. Press 'r' to resume or 'f' to finalize.")
            return

        done = self.align_step()
        if done:
            self.aligned = True
            self.pinfo("Alignment finished => no more offsets.")

    # -------------------------------------------------------------------------
    # 4) Alignment Step
    # -------------------------------------------------------------------------
    def align_step(self):
        """
        Looks up the transform from ee_mocap_frame -> wheel_mocap_frame,
        does a coarse/fine threshold approach, scaling speed so
        we move faster when far, slower when close.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.ee_mocap_frame, self.wheel_mocap_frame, rclpy.time.Time()
            )
            pose_diff = transform_to_pose(
                transform.transform, ros_to_time(self.getNow())
            )
            diff_xyz = pose_diff.xyz
            diff_quat = pose_diff.quat

            dist = np.linalg.norm(diff_xyz)
            w_clamped = max(min(abs(diff_quat.w), 1.0), -1.0)
            orient_dist = 2.0 * math.acos(w_clamped)

            # We only print if enough time has passed
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if (now_sec - self.last_print_time) > self.print_interval:
                self.last_print_time = now_sec
                self.pinfo(
                    f"Align Step:\n"
                    f"   dx={diff_xyz[0]:.3f}, dy={diff_xyz[1]:.3f}, dz={diff_xyz[2]:.3f}, dist={dist:.3f}\n"
                    f"   orientation diff={math.degrees(orient_dist):.2f} deg"
                )

            # two-phase threshold
            if (
                dist < self.coarse_threshold
                and orient_dist < self.orient_threshold_coarse
            ):
                self.pinfo("Below coarse thr => settle 1s, re-check with fine thr.")
                self.sleep(5.0)
                transform2 = self.tf_buffer.lookup_transform(
                    self.ee_mocap_frame, self.wheel_mocap_frame, rclpy.time.Time()
                )
                pd2 = transform_to_pose(
                    transform2.transform, ros_to_time(self.getNow())
                )
                dist2 = np.linalg.norm(pd2.xyz)
                w2 = max(min(abs(pd2.quat.w), 1.0), -1.0)
                orient2 = 2.0 * math.acos(w2)
                if dist2 < self.fine_threshold and orient2 < self.orient_threshold_fine:
                    self.pinfo("Below fine thr => auto-pause alignment.")
                    self.paused = True
                    return False
                else:
                    self.pinfo("Not within fine => continuing alignment steps.")

            self.last_distance = dist
            self.last_orient_dist = orient_dist

            # scale offset => faster far, slower close
            pos_offset_mm, new_quat = self.scale_offset(diff_xyz, diff_quat)

            self.leg.ik2.offset(xyz=pos_offset_mm, quat=new_quat, ee_relative=True)
            # returns false => keep iterating
            return False

        except Exception as e:
            self.pwarn(f"align_step error: {e}")
            return True

    # -------------------------------------------------------------------------
    # Utility: scale_offset => faster far, slower near
    # -------------------------------------------------------------------------
    def scale_offset(self, diff_xyz, diff_quat):
        """
        If dist is big => bigger step,
        if dist is small => smaller step
        Orientation partial step is optional => here we just do full orientation
        """
        dist_mm = np.linalg.norm(diff_xyz) * 1000.0

        # We'll define a simple approach:
        # if dist>200mm => move 50mm
        # if dist<20mm => move 10mm
        # else scale linearly
        max_range = 200.0  # mm (far)
        min_range = 20.0  # mm (close)
        max_move = 50.0  # mm
        min_move = 5.0  # mm

        if dist_mm > max_range:
            # far => big step
            step_mm = max_move
        elif dist_mm < min_range:
            # close => small step
            step_mm = min_move
        else:
            # linear interpolation
            fraction = (dist_mm - min_range) / (max_range - min_range)
            step_mm = min_move + fraction * (max_move - min_move)
            # fraction=0 => dist=20 => step=5
            # fraction=1 => dist=200 => step=50

        # direction + step
        direction = diff_xyz / (1e-9 + np.linalg.norm(diff_xyz))
        pos_offset_mm = direction * step_mm

        # For orientation, we'll just use the full quaternion => no partial step
        new_quat = diff_quat

        return pos_offset_mm, new_quat

    # -------------------------------------------------------------------------
    # Keyboard callback
    # -------------------------------------------------------------------------
    def key_upSUBCBK(self, msg: Key):
        key_code = msg.code
        # handle key up if needed

    def key_downSUBCBK(self, msg: Key):
        key_code = msg.code
        if key_code == Key.KEY_P:
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

    # Optional: partial orientation step => if you want to re-add it
    # def clip_orientation_quat(self, diff_quat, max_angle):
    #     ... see previous code


def main(args=None):
    myMain(SafeAlignArmNode, multiThreaded=True, args=args)


if __name__ == "__main__":
    main()
