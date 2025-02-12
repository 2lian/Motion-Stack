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


class MocapToArmAndWheelTF(EliaNode):
    def __init__(self):
        super().__init__("mocap_to_arm_wheel_tf_node")

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 1) Arm (mocap -> URDF)
        self.declare_parameter("simulation_mode", False)
        self.declare_parameter("arm_mocap_frame", f"mocap{LEG}gripper1")
        self.declare_parameter("arm_frame", f"leg{LEG}gripper1")
        self.declare_parameter("arm_offset_translation", [0.0, 0.0, 0.035])
        self.declare_parameter("arm_offset_rotation_rvec", [0.0, 0.0, math.pi])

        # 2) Wheel (mocap -> URDF)
        self.declare_parameter("wheel_mocap_frame", f"mocap{WHEEL}_body")
        self.declare_parameter("wheel_frame", f"wheel{WHEEL}_in")
        self.declare_parameter("wheel_frame_simu", f"wheel{WHEEL}_body")
        self.declare_parameter("wheel_offset_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("wheel_offset_rotation_rvec", [0.0, 0.0, math.pi / 2])

        # 3) Additional anchor offset (mocap11_body->mocap11_body_offset)
        self.declare_parameter("wheel_offset_anchor_frame", f"mocap{WHEEL}_body_offset")
        self.declare_parameter("wheel_offset_anchor_translation", [0.0, 0.0, 0.3])
        self.declare_parameter(
            "wheel_offset_anchor_rotation_rvec", [math.pi / 2, math.pi / 2, -math.pi]
        )

        # 4) End-Effector MOCAP in simulation
        self.declare_parameter("eef_mocap_frame", f"mocap{LEG}gripper2_straight")
        self.declare_parameter("eef_urdf_frame", f"leg{LEG}gripper2_straight")
        self.declare_parameter("eef_offset_translation", [0.02, 0.0, 0.0])
        self.declare_parameter("eef_offset_rotation_rvec", [0.0, 0.0, 0.0])

        # Read param values
        self.simulation_mode = self.get_parameter("simulation_mode").value

        self.arm_mocap_frame = self.get_parameter("arm_mocap_frame").value
        self.arm_frame = self.get_parameter("arm_frame").value
        self.arm_offset_t = self.get_parameter("arm_offset_translation").value
        self.arm_offset_rvec = self.get_parameter("arm_offset_rotation_rvec").value

        self.wheel_mocap_frame = self.get_parameter("wheel_mocap_frame").value
        self.wheel_frame = self.get_parameter("wheel_frame").value
        self.wheel_offset_t = self.get_parameter("wheel_offset_translation").value
        self.wheel_offset_rvec = self.get_parameter("wheel_offset_rotation_rvec").value

        self.wheel_offset_anchor_frame = self.get_parameter(
            "wheel_offset_anchor_frame"
        ).value
        self.wheel_offset_anchor_t = self.get_parameter(
            "wheel_offset_anchor_translation"
        ).value
        self.wheel_offset_anchor_rvec = self.get_parameter(
            "wheel_offset_anchor_rotation_rvec"
        ).value

        self.eef_mocap_frame = self.get_parameter("eef_mocap_frame").value
        self.eef_urdf_frame = self.get_parameter("eef_urdf_frame").value
        self.eef_offset_t = self.get_parameter("eef_offset_translation").value
        self.eef_offset_rvec = self.get_parameter("eef_offset_rotation_rvec").value

        # Convert "rotation_rvec" (roll->pitch->yaw) => single quaternion
        self.arm_offset_quat = self.euler_rvec_to_quat(self.arm_offset_rvec)
        self.wheel_offset_quat = self.euler_rvec_to_quat(self.wheel_offset_rvec)
        self.wheel_offset_anchor_quat = self.euler_rvec_to_quat(
            self.wheel_offset_anchor_rvec
        )
        self.eef_offset_quat = self.euler_rvec_to_quat(self.eef_offset_rvec)

        # Timers for offsets
        self.arm_timer = self.create_timer(0.1, self.update_arm_tf)
        self.wheel_timer = self.create_timer(0.1, self.update_wheel_tf)
        self.anchor_timer = self.create_timer(0.1, self.update_wheel_offset_anchor_tf)

        # If sim => track eef & publish mock transforms
        if self.simulation_mode:
            self.eef_timer = self.create_timer(0.1, self.update_eef_mocap_tf)
            self.mock_arm_timer = self.create_timer(0.1, self.mock_arm_mocap_tf)
            self.mock_wheel_timer = self.create_timer(0.1, self.mock_wheel_mocap_tf)
            self.mock_arm_sub = self.create_subscription(
                Transform, "/mock_mocap_control_arm", self.mock_arm_cb, 10
            )
            self.mock_wheel_sub = self.create_subscription(
                Transform, "/mock_mocap_control_wheel", self.mock_wheel_cb, 10
            )

        # Store mock transforms
        self.mock_arm_pos = np.array([0.3, 0.4, 0.05], dtype=float)
        self.mock_arm_quat = self.euler_rvec_to_quat([0.0, 0.0, 0.0])
        self.mock_wheel_pos = np.array([-0.55, 0.35, 0.2], dtype=float)
        self.mock_wheel_quat = self.euler_rvec_to_quat([0.0, 0.0, math.pi / 2])

        # for direct comparison with the MoCap pose
        from geometry_msgs.msg import PoseStamped

        self.eef_pose_pub = self.create_publisher(
            PoseStamped, f"/{self.eef_urdf_frame}/pose", 10
        )
        self.eef_pose_timer = self.create_timer(0.0095, self.publish_eef_urdf_pose)

        self.pinfo(
            "MoCap->Robot TF Node started, plus eef pose publisher in world frame."
        )

    def euler_rvec_to_quat(self, rvec_list):
        """
        rvec_list as [roll, pitch, yaw] => single quaternion (X->Y->Z).
        """
        roll = float(rvec_list[0])
        pitch = float(rvec_list[1])
        yaw = float(rvec_list[2])
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
        self.mock_wheel_pos = np.array(
            [msg.translation.x, msg.translation.y, msg.translation.z], dtype=float
        )
        self.mock_wheel_quat = qt.quaternion(
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

    def mock_wheel_mocap_tf(self):
        t = np2tf(self.mock_wheel_pos, self.mock_wheel_quat)
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.child_frame_id = self.wheel_mocap_frame
        msg.transform = t
        self.tf_broadcaster.sendTransform(msg)

    # -------------------------------------------------
    # Arm offset: 'mocapX' -> 'legX'
    # -------------------------------------------------
    @error_catcher
    def update_arm_tf(self):
        offs_xyz = np.array(self.arm_offset_t, dtype=float)
        offs_quat = self.arm_offset_quat
        transform = np2tf(offs_xyz, offs_quat)

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.arm_mocap_frame
        msg.child_frame_id = self.arm_frame
        msg.transform = transform
        self.tf_broadcaster.sendTransform(msg)

    # -------------------------------------------------
    # Wheel offset: 'mocapY' -> 'wheelY'
    # -------------------------------------------------
    @error_catcher
    def update_wheel_tf(self):
        offs_xyz = np.array(self.wheel_offset_t, dtype=float)
        offs_quat = self.wheel_offset_quat
        transform = np2tf(offs_xyz, offs_quat)

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.wheel_mocap_frame
        msg.child_frame_id = self.wheel_frame
        msg.transform = transform
        self.tf_broadcaster.sendTransform(msg)

    # -------------------------------------------------
    # Anchor offset: 'mocapY_body->mocapY_body_offset'
    # -------------------------------------------------
    @error_catcher
    def update_wheel_offset_anchor_tf(self):
        offs_xyz = np.array(self.wheel_offset_anchor_t, dtype=float)
        offs_quat = self.wheel_offset_anchor_quat
        transform = np2tf(offs_xyz, offs_quat)

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.wheel_mocap_frame
        msg.child_frame_id = self.wheel_offset_anchor_frame
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
    myMain(MocapToArmAndWheelTF, args=args)


if __name__ == "__main__":
    main()
