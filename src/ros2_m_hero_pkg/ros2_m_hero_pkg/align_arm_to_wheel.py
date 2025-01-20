#!/usr/bin/env python3

import math
import threading
import time

import numpy as np
import rclpy
from easy_robot_control.EliaNode import EliaNode, myMain, np2tf, tf2np
from easy_robot_control.leg_api import Leg
from easy_robot_control.utils.math import qt
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener


class SafeAlignArmNode(EliaNode):
    """
    Single node that:
      1) Moves arm to a safe pose
      2) Waits for user to press Enter (human in the loop)
      3) Performs alignment (offset) until target is reached or threshold
    """

    def __init__(self):
        super().__init__("align_node")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.leg = Leg(number=3, parent=self)

        self.declare_parameter("arm_end_effector_frame", "leg3gripper2_straight")
        self.declare_parameter("wheel_frame", "wheel11_body")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("safety_offset_z", 0.1)  # e.g. 10 cm
        self.declare_parameter("distance_threshold", 0.03)  # e.g. 3 cm

        self.ee_frame = self.get_parameter("arm_end_effector_frame").value
        self.wheel_frame = self.get_parameter("wheel_frame").value
        self.world_frame = self.get_parameter("world_frame").value
        self.safety_offset_z = self.get_parameter("safety_offset_z").value
        self.distance_threshold = self.get_parameter("distance_threshold").value

        # flags
        self.safe_pose_done = False
        self.user_approved = False
        self.aligned = False
        self.last_distance = None

        self.go_to_safe_pose()

        #  start a background thread that blocks for user input
        #  so the main ROS spin continues in parallel
        threading.Thread(target=self.wait_for_user_input, daemon=True).start()

        #  create a timer that runs the main loop every 2s
        self.timer = self.create_timer(2.0, self.main_loop)

        self.pinfo("SafeAlignArmNode initialized.")

    def go_to_safe_pose(self):
        """Sends to the pre-align position"""
        self.pinfo("Sending arm to safe pose angles...")

        angs = {
            0: 0.07147328709278614,
            3: 1.399104316777219,
            4: -0.1664058079065805,
            5: -0.4821810625119574,
            6: 0.01569426844382693,
            7: 2.5838983308690566,
            8: 1.3951540963836382,
        }
        for num, ang in angs.items():
            jobj = self.leg.get_joint_obj(num)
            if jobj is None:
                continue
            jobj.apply_angle_target(ang)

        self.pinfo("Safe pose angles commanded; waiting a few seconds to settle.")

    def wait_for_user_input(self):
        """
        Runs in a separate thread.
        Blocks on user input (Press Enter),
        then sets user_approved = True.
        """
        input(
            "\n==== HUMAN IN THE LOOP ====\nPress Enter to proceed with alignment, or Ctrl+C to abort.\n"
        )
        self.pinfo("User pressed Enter. Proceeding with alignment.")
        self.user_approved = True

    def main_loop(self):
        """
        Called every 2s by the ROS timer.
          - Wait for safe pose to "settle"
          - Wait for user_approved
          - Then run alignment steps
          - Stop once aligned
        """
        # if already aligned, do nothing
        if self.aligned:
            return

        if not self.safe_pose_done:
            self.safe_pose_done = True
            self.pinfo("Safe pose assumed to be done now.")
            return

        # wait for user prompt
        if not self.user_approved:
            self.pinfo("Waiting for human approval to align...")
            return

        # safe_pose_done and user_approved => do alignment step
        done = self.align_step()
        if done:
            self.aligned = True
            self.pinfo(
                "Alignment completed. Timer will keep spinning, but we won't do more alignment."
            )
            # self.timer.cancel()

    def align_step(self):
        """
        - Lookup TF for end-effector & wheel
        - Compute difference (plus safety_offset_z)
        - If difference < threshold or overshoot => done
        - else offset
        """
        try:
            ee_tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.ee_frame, rclpy.time.Time()
            )
            wheel_tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.wheel_frame, rclpy.time.Time()
            )

            ee_xyz, ee_q = tf2np(ee_tf.transform)
            wh_xyz, wh_q = tf2np(wheel_tf.transform)

            diff_world = wh_xyz - ee_xyz

            # offset for safety (z axis)
            diff_world[2] += self.safety_offset_z

            dist = np.linalg.norm(diff_world)
            self.pinfo(
                f"Align Step: dx={diff_world[0]:.3f}, "
                f"dy={diff_world[1]:.3f}, dz={diff_world[2]:.3f}, dist={dist:.3f}"
            )

            # if distance < threshold => done
            if dist < self.distance_threshold:
                self.pinfo(
                    f"Dist {dist:.3f} < threshold {self.distance_threshold:.3f}. Done."
                )
                return True

            # if distance is bigger than last time => overshoot => stop
            if self.last_distance is not None and dist > self.last_distance:
                self.pinfo(
                    "Distance is larger than last time => overshoot or noise. Stopping."
                )
                return True

            self.last_distance = dist

            # convert difference to local coords, with respect to "leg3gripper1" base
            inv_q = ee_q.conjugate()
            diff_local = qt.rotate_vectors(inv_q, diff_world)
            diff_local_mm = diff_local * 1000.0

            # send offset
            self.leg.ik2.offset(xyz=diff_local_mm, quat=None, ee_relative=False)
            self.pinfo("IK offset command sent.")

            return False

        except Exception as e:
            self.pwarn(f"Alignment step error: {e}")
            return True


def main(args=None):
    myMain(SafeAlignArmNode, multiThreaded=True, args=args)


if __name__ == "__main__":
    main()
