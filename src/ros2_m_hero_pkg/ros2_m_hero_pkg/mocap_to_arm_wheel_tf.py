#!/usr/bin/env python3

"""
Node that creates transforms between MoCap tfs and Robot tfs.

Author: Shamistan KARIMOV
Lab: SRL, Moonshot team
"""

import math

import numpy as np
import rclpy
from easy_robot_control.EliaNode import EliaNode, error_catcher, myMain, np2tf
from easy_robot_control.utils.math import Quaternion, qt
from geometry_msgs.msg import PoseStamped, Transform, TransformStamped
from rclpy.task import Future
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

LEG: int = 2
WHEEL: int = 14


class MocapToArmAndTargetTF(EliaNode):
    def __init__(self):
        super().__init__("mocap_to_arm_target_tf_node")

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 1) Arm (MoCap -> URDF)
        self.declare_parameter("simulation_mode", False)
        self.declare_parameter("arm_mocap_frame", f"mocap{LEG}gripper1")
        self.declare_parameter("arm_frame", f"leg{LEG}gripper1")
        self.declare_parameter("arm_offset_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("arm_offset_rotation_rvec", [0.0, 0.0, 0.0])

        # 2) Target (Generalized)
        self.declare_parameter("target_config", "sled")

        self.declare_parameter("target_mocap_frame", f"mocap{WHEEL}_body")
        self.declare_parameter("target_frame", f"wheel{WHEEL}_in")
        self.declare_parameter("target_offset_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("target_offset_rotation_rvec", [0.0, 0.0, 0.0])

        # 3) Additional anchor offset (target anchor frame)
        self.declare_parameter(
            "target_offset_anchor_frame", f"mocap{WHEEL}_body_offset"
        )
        self.declare_parameter("target_offset_anchor_translation", [0.0, 0.0, 0.3])
        self.declare_parameter(
            "target_offset_anchor_rotation_rvec",
            [math.pi / 2, math.pi / 2, -math.pi / 2],
        )

        # 4) End-Effector MoCap in simulation
        self.declare_parameter("eef_mocap_frame", f"mocap{LEG}gripper2_straight")
        self.declare_parameter("eef_urdf_frame", f"leg{LEG}gripper2_straight")
        self.declare_parameter("eef_offset_translation", [0.05, -0.05, -0.02])
        self.declare_parameter("eef_offset_rotation_rvec", [0.0, 0.0, 0.0])

        # Read parameters
        self.simulation_mode = self.get_parameter("simulation_mode").value
        self.target_config = self.get_parameter("target_config").value.lower()

        self.arm_mocap_frame = self.get_parameter("arm_mocap_frame").value
        self.arm_frame = self.get_parameter("arm_frame").value
        self.arm_offset_t = self.get_parameter("arm_offset_translation").value
        self.arm_offset_rvec = self.get_parameter("arm_offset_rotation_rvec").value

        self.target_mocap_frame = self.get_parameter("target_mocap_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.target_offset_t = self.get_parameter("target_offset_translation").value
        self.target_offset_rvec = self.get_parameter(
            "target_offset_rotation_rvec"
        ).value

        self.target_offset_anchor_frame = self.get_parameter(
            "target_offset_anchor_frame"
        ).value
        self.target_offset_anchor_t = self.get_parameter(
            "target_offset_anchor_translation"
        ).value
        self.target_offset_anchor_rvec = self.get_parameter(
            "target_offset_anchor_rotation_rvec"
        ).value

        self.eef_mocap_frame = self.get_parameter("eef_mocap_frame").value
        self.eef_urdf_frame = self.get_parameter("eef_urdf_frame").value
        self.eef_offset_t = self.get_parameter("eef_offset_translation").value
        self.eef_offset_rvec = self.get_parameter("eef_offset_rotation_rvec").value

        # Store mock transforms
        self.mock_arm_pos = np.array([0.3, 0.4, 0.05], dtype=float)
        self.mock_arm_quat = self.euler_rvec_to_quat([0.0, 0.0, 0.0])
        self.mock_target_pos = np.array([1.15, 0.35, 0.2], dtype=float)
        self.mock_target_quat = self.euler_rvec_to_quat([0.0, 0.0, -3.34])

        # Configure target based on target_config
        self.configure_target()

        # Convert rotation vectors to quaternions
        self.arm_offset_quat = self.euler_rvec_to_quat(self.arm_offset_rvec)
        self.target_offset_quat = self.euler_rvec_to_quat(self.target_offset_rvec)
        self.target_offset_anchor_quat = self.euler_rvec_to_quat(
            self.target_offset_anchor_rvec
        )
        self.eef_offset_quat = self.euler_rvec_to_quat(self.eef_offset_rvec)

        # Timers for offsets
        self.arm_timer = self.create_timer(0.1, self.update_arm_tf)
        self.target_timer = self.create_timer(0.1, self.update_target_tf)
        self.anchor_timer = self.create_timer(0.1, self.update_target_offset_anchor_tf)

        if self.simulation_mode:
            self.eef_timer = self.create_timer(0.1, self.update_eef_mocap_tf)
            self.mock_arm_timer = self.create_timer(0.1, self.mock_arm_mocap_tf)
            self.mock_wheel_timer = self.create_timer(0.1, self.mock_target_mocap_tf)
            self.mock_arm_sub = self.create_subscription(
                Transform, "/mock_mocap_control_arm", self.mock_arm_cb, 10
            )
            self.mock_wheel_sub = self.create_subscription(
                Transform, "/mock_mocap_control_wheel", self.mock_wheel_cb, 10
            )

        self.eef_pose_pub = self.create_publisher(
            PoseStamped, f"/{self.eef_urdf_frame}/pose", 10
        )
        self.eef_pose_timer = self.create_timer(0.015, self.publish_eef_urdf_pose)

        self.pinfo("MoCap->Robot TF Node started.")

    def configure_target(self):
        """
        Configure target-related parameters based on `target_config`.
        The default configuration ("target") uses the base parameters.
        If set to 'sled', override target parameters accordingly.
        """
        if self.target_config == "sled":
            self.pinfo(
                "Target configuration set to 'sled'. Overriding target parameters."
            )
            self.target_mocap_frame = "mocapSled"
            self.target_frame = "sled_frame"
            self.target_offset_t = [0.0, 0.0, 0.0]
            self.target_offset_rvec = [0.0, 0.0, 0.0]
            self.target_offset_anchor_frame = "mocapSled_anchor"
            self.target_offset_anchor_t = [0.0, 0.0, 0.15]
            self.target_offset_anchor_rvec = [math.pi / 2, math.pi / 2, math.pi / 2]
            self.mock_target_pos = np.array([1.15, 0.35, 0.2], dtype=float)
            self.mock_target_quat = self.euler_rvec_to_quat([0.0, -1.57, -0.2])

        elif self.target_config == "wheel":
            self.pinfo(
                "Target configuration set to 'wheel'. Overriding target parameters."
            )

    def euler_rvec_to_quat(self, rvec_list):
        """
        Convert rotation vector [roll, pitch, yaw] to a quaternion.
        """
        roll, pitch, yaw = map(float, rvec_list)
        qx = qt.from_rotation_vector(np.array([roll, 0.0, 0.0], dtype=float))
        qy = qt.from_rotation_vector(np.array([0.0, pitch, 0.0], dtype=float))
        qz = qt.from_rotation_vector(np.array([0.0, 0.0, yaw], dtype=float))
        return qz * qy * qx

    # -------------------------------------------------
    # Publish Pose of eef_urdf_frame in 'world'
    # => used for data comparison with MoCap pose
    # -------------------------------------------------
    @error_catcher
    def publish_eef_urdf_pose(self):
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                "world", self.eef_urdf_frame, rclpy.time.Time()
            )
            pose_st = PoseStamped()
            pose_st.header.stamp = self.get_clock().now().to_msg()
            pose_st.header.frame_id = "world"
            pose_st.pose.position.x = tf_stamped.transform.translation.x
            pose_st.pose.position.y = tf_stamped.transform.translation.y
            pose_st.pose.position.z = tf_stamped.transform.translation.z
            pose_st.pose.orientation = tf_stamped.transform.rotation

            self.eef_pose_pub.publish(pose_st)

        except Exception as e:
            # self.pwarn(f"publish_eef_urdf_pose error: {e}")
            pass

    # -------------------------------------------------
    # Simulation: Mock "world->mocap" transforms
    # -------------------------------------------------
    def mock_arm_cb(self, msg: Transform):
        self.mock_arm_pos = np.array(
            [msg.translation.x, msg.translation.y, msg.translation.z], dtype=float
        )
        self.mock_arm_quat = qt.quaternion(
            msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z
        )

    def mock_wheel_cb(self, msg: Transform):
        self.mock_target_pos = np.array(
            [msg.translation.x, msg.translation.y, msg.translation.z], dtype=float
        )
        self.mock_target_quat = qt.quaternion(
            msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z
        )

    def mock_arm_mocap_tf(self):
        t = np2tf(self.mock_arm_pos, self.mock_arm_quat)
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.child_frame_id = self.arm_mocap_frame
        msg.transform = t
        self.tf_broadcaster.sendTransform(msg)

    def mock_target_mocap_tf(self):
        t = np2tf(self.mock_target_pos, self.mock_target_quat)
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.child_frame_id = self.target_mocap_frame
        msg.transform = t
        self.tf_broadcaster.sendTransform(msg)

    @error_catcher
    def update_arm_tf(self):
        transform = np2tf(
            np.array(self.arm_offset_t, dtype=float), self.arm_offset_quat
        )
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.arm_mocap_frame
        msg.child_frame_id = self.arm_frame
        msg.transform = transform
        self.tf_broadcaster.sendTransform(msg)

    @error_catcher
    def update_target_tf(self):
        transform = np2tf(
            np.array(self.target_offset_t, dtype=float), self.target_offset_quat
        )
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_mocap_frame
        msg.child_frame_id = self.target_frame
        msg.transform = transform
        self.tf_broadcaster.sendTransform(msg)

    @error_catcher
    def update_target_offset_anchor_tf(self):
        transform = np2tf(
            np.array(self.target_offset_anchor_t, dtype=float),
            self.target_offset_anchor_quat,
        )
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_mocap_frame
        msg.child_frame_id = self.target_offset_anchor_frame
        msg.transform = transform
        self.tf_broadcaster.sendTransform(msg)

    # -------------------------------------------------
    # EEF offset in sim: 'legXgripper2_straight'->'mocapXgripper2_straight'
    # -------------------------------------------------
    @error_catcher
    def update_eef_mocap_tf(self):
        offs_xyz = np.array(self.eef_offset_t, dtype=float)
        eef_quat = self.eef_offset_quat
        transform = np2tf(offs_xyz, eef_quat)
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.eef_urdf_frame
        msg.child_frame_id = self.eef_mocap_frame
        msg.transform = transform
        self.tf_broadcaster.sendTransform(msg)


def main(args=None):
    myMain(MocapToArmAndTargetTF, args=args)


if __name__ == "__main__":
    main()
