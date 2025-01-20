#!/usr/bin/env python3

import math
import threading
import time

import numpy as np
import rclpy
from easy_robot_control.EliaNode import EliaNode, myMain, tf2np
from easy_robot_control.leg_api import Leg
from easy_robot_control.utils.math import qt
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener


class SafeAlignArmNode(EliaNode):
    """
    Single node that:
      1) (Optional) safe pose
      2) Waits for user "enter"
      3) Aligns position+orientation by referencing MOCAP frames:
         - end-eff: "mocap3gripper2_straight"
         - wheel:   "mocap11_body"
      4) Uses 'leg3gripper2_straight' local coords (ee_relative=True)
         to send offsets with ik2
    """

    def __init__(self):
        super().__init__("align_node")

        # TF buffer & listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.leg = Leg(number=3, parent=self)

        # Parameters
        # The MOCAP frames for your end-effector & wheel
        self.declare_parameter("ee_mocap_frame", "mocap3gripper2_straight")
        self.declare_parameter("wheel_mocap_frame", "mocap11_body_offset")

        # The real URDF link for end-eff. (We need this for local coords)
        self.declare_parameter("ee_urdf_frame", "leg3gripper2_straight")
        self.declare_parameter("world_frame", "world")

        # Offsets / thresholds
        self.declare_parameter("safety_offset_z", 0.0)  # e.g. 10 cm
        self.declare_parameter("distance_threshold", 0.03)  # e.g. 3 cm
        self.declare_parameter("orientation_threshold", 0.1)  # e.g. ~5.7 deg

        # Retrieve param values
        self.ee_mocap_frame = self.get_parameter("ee_mocap_frame").value
        self.wheel_mocap_frame = self.get_parameter("wheel_mocap_frame").value
        self.ee_urdf_frame = self.get_parameter("ee_urdf_frame").value
        self.world_frame = self.get_parameter("world_frame").value

        self.safety_offset_z = self.get_parameter("safety_offset_z").value
        self.distance_threshold = self.get_parameter("distance_threshold").value
        self.orientation_threshold = self.get_parameter("orientation_threshold").value

        # State flags
        self.safe_pose = False
        self.user_approved = False
        self.aligned = False

        # Track last iteration's positional & orientation error
        self.last_distance = None
        self.last_orient_dist = None

        # Start a background thread for user input
        threading.Thread(target=self.wait_for_user_input, daemon=True).start()

        # Create a timer for main loop
        self.timer = self.create_timer(0.5, self.main_loop)

        self.pinfo(
            "SafeAlignArmNode: using MOCAP frames + orientation alignment, ee_relative=True"
        )

    def go_to_safe_pose(self):
        """Sends the arm to a pre-alignment (safe) joint configuration."""
        self.leg.look_for_joints()
        self.pinfo("Sending arm to a known safe pose.")
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
            joint_obj = self.leg.get_joint_obj(num)
            if joint_obj is not None:
                joint_obj.apply_angle_target(ang)

        # check joint_states for safer check
        # joint_obj = self.leg.get_joint_obj(1)
        # ang = joint_obj.angle
        self.safe_pose = True

    def wait_for_user_input(self):
        """Blocks on user input in separate thread. Press Enter to proceed."""
        input("\n=== HUMAN IN THE LOOP ===\nPress Enter to proceed with alignment.\n")
        self.pinfo("User pressed Enter. Proceeding with alignment.")
        self.leg.ik2.reset()
        self.user_approved = True

    def main_loop(self):
        """Called periodically by the ROS timer to attempt alignment once user is ready."""
        if self.aligned:
            return

        if not self.safe_pose:
            self.go_to_safe_pose()
            return

        if not self.user_approved:
            self.pinfo("Waiting for user approval...")
            return

        # Attempt alignment
        done = self.align_step()
        if done:
            self.aligned = True
            self.pinfo("Alignment done. No more offsets will be sent.")

    def align_step(self):
        """
        - Lookup MOCAP frames: world->mocap3gripper2_straight, world->mocap11_body
        - Compute position & orientation difference in world
        - Apply safety_offset_z
        - Convert difference to local coords of *real end-eff* (leg3gripper2_straight)
        - Send offsets with ee_relative=True
        - Stop if under threshold or overshoot
        """
        try:
            # 1) Get TF for MOCAP end-eff & wheel
            ee_tf = self.tf_buffer.lookup_transform(
                self.ee_mocap_frame, self.wheel_mocap_frame, rclpy.time.Time()
            )

            # Convert to arrays/quaternions
            diff_world, rot_diff_world = tf2np(ee_tf.transform)

            dist = np.linalg.norm(diff_world)

            # Orientation diff in world
            w_clamped = max(min(abs(rot_diff_world.w), 1.0), -1.0)
            orient_dist = 2.0 * math.acos(w_clamped)

            self.pinfo(
                f"Align Step:\n"
                f"  MOCAP PosDiff=dx={diff_world[0]:.3f}, dy={diff_world[1]:.3f}, dz={diff_world[2]:.3f}, dist={dist:.3f}\n"
                f"  MOCAP OriDiff={math.degrees(orient_dist):.2f} deg"
            )

            # Check thresholds
            if (
                dist < self.distance_threshold
                and orient_dist < self.orientation_threshold
            ):
                self.pinfo(
                    f"Position < {self.distance_threshold:.3f} and orientation < {self.orientation_threshold:.3f} rad => done"
                )
                return True

            # Check overshoot
            # if self.last_distance is not None and dist > self.last_distance:
            #     self.pinfo("Position overshoot => stopping.")
            #     return True
            # if self.last_orient_dist is not None and orient_dist > self.last_orient_dist:
            #     self.pinfo("Orientation overshoot => stopping.")
            #     return True

            self.last_distance = dist
            self.last_orient_dist = orient_dist

            # 2) Convert these diffs to local coords of the *real* end-eff (leg3gripper2_straight)
            #    Because we want to do ee_relative=True => interpret offset in local coords

            #   a) Get "world->leg3gripper2_straight" for orientation
            diff_local_vec_mm = diff_world * 1000.0

            #   c) Orientation local:
            #   local_rot_diff = inverse(real_ee_quat) * rot_diff_world
            local_rot_diff = rot_diff_world

            # 3) Send offset => ee_relative=True
            self.leg.ik2.offset(
                xyz=diff_local_vec_mm, quat=local_rot_diff, ee_relative=True
            )
            self.pinfo("IK offset command sent (pos+ori, ee_relative=True).")

            return False

        except Exception as e:
            self.pwarn(f"align_step error: {e}")
            return True


def main(args=None):
    myMain(SafeAlignArmNode, multiThreaded=True, args=args)


if __name__ == "__main__":
    main()
