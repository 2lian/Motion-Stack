#!/usr/bin/env python3

import math
from os import walk

import numpy as np
import rclpy
from easy_robot_control.EliaNode import EliaNode, myMain
from easy_robot_control.leg_api import Leg
from easy_robot_control.utils.math import qt  # For rotating vectors/quaternions
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener


class AlignArmToWheelNode(EliaNode):
    def __init__(self):
        super().__init__("align_arm_to_wheel_node")

        # TF buffer+listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Parameters
        self.declare_parameter("arm_end_effector_frame", "leg3gripper2_straight")
        self.declare_parameter("wheel_frame", "wheel11_body")
        self.declare_parameter("world_frame", "world")

        # Distances in meters:
        self.declare_parameter("safety_offset_z", 0.1)  # 10 cm
        self.declare_parameter("distance_threshold", 0.03)  # 3 cm to stop

        self.ee_frame = self.get_parameter("arm_end_effector_frame").value
        self.wheel_frame = self.get_parameter("wheel_frame").value
        self.world_frame = self.get_parameter("world_frame").value
        self.safety_offset_z = self.get_parameter("safety_offset_z").value
        self.distance_threshold = self.get_parameter("distance_threshold").value

        # Create the Leg object for your arm (leg3)
        self.leg = Leg(number=3, parent=self)

        # Flags/state to avoid repeating offsets
        self.aligned = False
        self.last_distance = None

        # Timer: attempt alignment every 2s
        self.timer = self.create_timer(2.0, self.align_arm_to_wheel)
        self.pinfo("AlignArmToWheelNode started.")

    def align_arm_to_wheel(self):
        # 1) If we already decided to stop, do nothing
        if self.aligned:
            return

        try:
            # 2) Get end-effector & wheel transforms in world coords
            ee_tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.ee_frame, rclpy.time.Time()
            )
            wheel_tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.wheel_frame, rclpy.time.Time()
            )

            # self.pinfo(ee_tf)
            # self.pinfo(wheel_tf)

            # 3) Extract positions (world)
            ee_xyz_world = np.array(
                [
                    ee_tf.transform.translation.x,
                    ee_tf.transform.translation.y,
                    ee_tf.transform.translation.z,
                ]
            )
            wheel_xyz_world = np.array(
                [
                    wheel_tf.transform.translation.x,
                    wheel_tf.transform.translation.y,
                    wheel_tf.transform.translation.z,
                ]
            )

            # 4) Compute difference: (wheel - end-effector)
            diff_world = wheel_xyz_world - ee_xyz_world

            # Approach from above by subtracting offset in z
            diff_world[2] += self.safety_offset_z

            # 5) Check distance
            distance = np.linalg.norm(diff_world)
            self.pinfo(
                f"Difference (WORLD): dx={diff_world[0]:.3f}, "
                f"dy={diff_world[1]:.3f}, dz={diff_world[2]:.3f}, dist={distance:.3f}"
            )

            # 6) Decide if we should stop
            # If distance is small OR if it starts getting bigger => we're done
            if self.last_distance is not None:
                if distance < self.distance_threshold:
                    self.get_logger().info(
                        f"Stopping alignment. distance={distance:.3f}, last_distance={self.last_distance:.3f}"
                    )
                    self.aligned = True
                    self.timer.cancel()
                    return

            self.last_distance = distance

            # 7) Convert difference to local end-effector coords
            ee_q = qt.quaternion(
                ee_tf.transform.rotation.w,
                ee_tf.transform.rotation.x,
                ee_tf.transform.rotation.y,
                ee_tf.transform.rotation.z,
            )
            # rotate_vectors(ee_q.inverse, diff_world) => local coords
            ee_q_inv = ee_q.conjugate()
            diff_local = qt.rotate_vectors(ee_q_inv, diff_world)

            # 8) Convert meters -> millimeters if IK2 uses mm
            diff_local_mm = diff_local * 1000.0

            # self.pinfo(ee_q)
            # self.pinfo(ee_q_inv)
            # self.pinfo(diff_local)
            # self.pinfo(diff_local_mm)
            # 9) Send IK offset in local end-effector space
            self.leg.ik2.offset(
                xyz=diff_local_mm,
                quat=None,  # Keep orientation
                ee_relative=False,  # local coords
            )
            self.get_logger().info("Sent IK offset command in end-effector space.")

        except Exception as e:
            self.get_logger().warn(f"TF lookup or alignment failed: {e}")


def main(args=None):
    myMain(AlignArmToWheelNode)


if __name__ == "__main__":
    main()
