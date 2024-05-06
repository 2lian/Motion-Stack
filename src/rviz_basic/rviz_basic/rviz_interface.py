import time

import numpy as np
from numpy.typing import NDArray
import quaternion as qt
from scipy.spatial import geometric_slerp
import rclpy
from rclpy.node import Node, ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import tf2_ros

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import TransformStamped, Transform

from easy_robot_control.EliaNode import EliaNode


class CallbackHolder:
    def __init__(self, leg, joint, parent_node, joint_state):
        self.leg = leg
        self.joint_state = joint_state
        self.joint = joint
        self.parent_node = parent_node
        self.parent_node.create_subscription(
            Float64,
            f"set_joint_{leg}_{joint}_real",
            self.set_joint_cbk,
            10,
            callback_group=self.parent_node.cbk_legs,
        )
        self.pub_back_to_ros2_structure = self.parent_node.create_publisher(
            Float64,
            f"angle_{self.leg}_{self.joint}",
            10,
        )
        self.last_msg_data = 0
        self.tmr_angle_to_ros2 = self.parent_node.create_timer(
            0.2,
            self.publish_back_up_to_ros2,
            callback_group=self.parent_node.cbk_legs,
        )

    def set_joint_cbk(self, msg):
        # if self.joint == 0 and self.leg == 0:
        # self.parent_node.get_logger().warn(f"angle got")
        # angle = msg.data
        self.last_msg_data = msg.data
        angle = -msg.data

        self.joint_state.position[self.leg * 3 + self.joint] = angle
        # self.publish_back_up_to_ros2()

        self.parent_node.request_refresh()
        return

    def publish_back_up_to_ros2(self, angle: float | None = None) -> None:
        if angle is None:
            angle = self.last_msg_data
        msg = Float64()
        msg.data = float(angle)
        self.pub_back_to_ros2_structure.publish(msg)

        # next_update_in = (np.random.randn()) * 0.1 + 0.2
        # self.tmr_angle_to_ros2.timer_period_ns = abs(next_update_in * 1e9)
        # self.tmr_angle_to_ros2.reset()


class RVizInterfaceNode(EliaNode):

    def __init__(self):
        # rclpy.init()
        super().__init__("joint_state_rviz")  # type: ignore

        self.NAMESPACE = self.get_namespace()

        self.necessary_node_names = ["rviz", "rviz2"]
        nodes_connected = False
        silent_trial = 3

        while not nodes_connected:
            for name in self.necessary_node_names:
                node_info = self.get_node_names_and_namespaces()
                for node_name, node_namespace in node_info:
                    if node_name == name:
                        nodes_connected = True
                        break

            if not nodes_connected and silent_trial < 0:
                self.get_logger().warn(
                    f"""Waiting for lower level, check that the {self.NAMESPACE}/{self.necessary_node_names} node is running"""
                )
                time.sleep(1)
            elif not nodes_connected:
                silent_trial -= -1
                time.sleep(1)

        self.get_logger().warning(f"""Rviz connected :)""")

        self.declare_parameter("std_movement_time", float(0.5))
        self.MOVEMENT_TIME = (
            self.get_parameter("std_movement_time").get_parameter_value().double_value
        )

        self.declare_parameter("frame_prefix", "")
        self.FRAME_PREFIX = (
            self.get_parameter("frame_prefix").get_parameter_value().string_value
        )

        if True:
            leg_num_remapping = [3, 4, 1, 2]
            joint_num_remapping = [1, 2, 3]
            begining = "joint"
            middle = "-"

            self.joint_state = JointState()
            self.joint_state.name = [
                f"{begining}{leg_num_remapping[0]}{middle}{joint_num_remapping[0]}",
                f"{begining}{leg_num_remapping[0]}{middle}{joint_num_remapping[1]}",
                f"{begining}{leg_num_remapping[0]}{middle}{joint_num_remapping[2]}",
                f"{begining}{leg_num_remapping[1]}{middle}{joint_num_remapping[0]}",
                f"{begining}{leg_num_remapping[1]}{middle}{joint_num_remapping[1]}",
                f"{begining}{leg_num_remapping[1]}{middle}{joint_num_remapping[2]}",
                f"{begining}{leg_num_remapping[2]}{middle}{joint_num_remapping[0]}",
                f"{begining}{leg_num_remapping[2]}{middle}{joint_num_remapping[1]}",
                f"{begining}{leg_num_remapping[2]}{middle}{joint_num_remapping[2]}",
                f"{begining}{leg_num_remapping[3]}{middle}{joint_num_remapping[0]}",
                f"{begining}{leg_num_remapping[3]}{middle}{joint_num_remapping[1]}",
                f"{begining}{leg_num_remapping[3]}{middle}{joint_num_remapping[2]}",
            ]
            self.joint_state.position = [0.0] * (3 * 4)

        if False:
            leg_num_remapping = [0, 1, 2, 3]
            joint_num_remapping = [1, 2, 3]
            begining = "Limb"
            middle = "Pitch"

            self.joint_state = JointState()
            self.joint_state.name = [
                f"{begining}{leg_num_remapping[0]}{middle}{joint_num_remapping[0]}",
                f"{begining}{leg_num_remapping[0]}{middle}{joint_num_remapping[1]}",
                f"{begining}{leg_num_remapping[0]}{middle}{joint_num_remapping[2]}",
                f"{begining}{leg_num_remapping[1]}{middle}{joint_num_remapping[0]}",
                f"{begining}{leg_num_remapping[1]}{middle}{joint_num_remapping[1]}",
                f"{begining}{leg_num_remapping[1]}{middle}{joint_num_remapping[2]}",
                f"{begining}{leg_num_remapping[2]}{middle}{joint_num_remapping[0]}",
                f"{begining}{leg_num_remapping[2]}{middle}{joint_num_remapping[1]}",
                f"{begining}{leg_num_remapping[2]}{middle}{joint_num_remapping[2]}",
                f"{begining}{leg_num_remapping[3]}{middle}{joint_num_remapping[0]}",
                f"{begining}{leg_num_remapping[3]}{middle}{joint_num_remapping[1]}",
                f"{begining}{leg_num_remapping[3]}{middle}{joint_num_remapping[2]}",
            ]

            leg_num_remapping = [0, 1, 2, 3]
            joint_num_remapping = [1, 2, 3]
            begining = "Limb"
            middle = "Roll"

            self.joint_state.name = self.joint_state.name + [
                f"{begining}{leg_num_remapping[0]}{middle}{joint_num_remapping[0]}",
                f"{begining}{leg_num_remapping[0]}{middle}{joint_num_remapping[1]}",
                f"{begining}{leg_num_remapping[0]}{middle}{joint_num_remapping[2]}",
                f"{begining}{leg_num_remapping[1]}{middle}{joint_num_remapping[0]}",
                f"{begining}{leg_num_remapping[1]}{middle}{joint_num_remapping[1]}",
                f"{begining}{leg_num_remapping[1]}{middle}{joint_num_remapping[2]}",
                f"{begining}{leg_num_remapping[2]}{middle}{joint_num_remapping[0]}",
                f"{begining}{leg_num_remapping[2]}{middle}{joint_num_remapping[1]}",
                f"{begining}{leg_num_remapping[2]}{middle}{joint_num_remapping[2]}",
                f"{begining}{leg_num_remapping[3]}{middle}{joint_num_remapping[0]}",
                f"{begining}{leg_num_remapping[3]}{middle}{joint_num_remapping[1]}",
                f"{begining}{leg_num_remapping[3]}{middle}{joint_num_remapping[2]}",
            ]

            self.joint_state.position = [0.0] * (len(self.joint_state.name))

        self.set_joint_subs = []

        # V Subscriber V
        #   \  /   #
        #    \/    #
        self.cbk_legs = MutuallyExclusiveCallbackGroup()
        self.cbk_holder_list = []
        for leg in range(4):
            for joint in range(3):
                holder = CallbackHolder(leg, joint, self, self.joint_state)
                self.cbk_holder_list.append(holder)

        self.body_pose_sub = self.create_subscription(
            Transform,
            "robot_body",
            self.robot_body_pose_cbk,
            10,
            callback_group=self.cbk_legs,
        )

        self.smooth_body_pose_sub = self.create_subscription(
            Transform,
            "smooth_body_rviz",
            self.smooth_body_trans,
            10,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        #    /\    #
        #   /  \   #
        # ^ Subscriber ^

        # V Publisher V
        #   \  /   #
        #    \/    #
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        # self.body_pose_pub = self.create_publisher(
        # TFMessage,
        # '/BODY', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        #    /\    #
        #   /  \   #
        # ^ Publisher ^

        # V Service V
        #   \  /   #
        #    \/    #
        self.iAmAlive = self.create_service(Empty, "rviz_interface_alive", lambda: None)
        #    /\    #
        #   /  \   #
        # ^ Service ^

        # V Timer V
        #   \  /   #
        #    \/    #
        self.refresh_timer = self.create_timer(1 / 60, self.refresh)
        self.refresh_every_seconds_slow = self.create_timer(1, self.request_refresh)
        #    /\    #
        #   /  \   #
        # ^ Timer ^
        self.current_body_xyz: NDArray = np.array([0, 0, 0.200], dtype=float)
        self.current_body_quat: qt.quaternion = qt.one
        # self.movement_time = 1.5 # is a ros param
        self.MVMT_UPDATE_RATE: int = 30

    def refresh(self, now=None):
        if now is None:
            now = self.get_clock().now()
        time_now_stamp = now.to_msg()
        xyz = self.current_body_xyz
        rot = self.current_body_quat
        msgTF = self.np2tf(xyz, rot)

        body_transform = TransformStamped()
        body_transform.header.stamp = time_now_stamp
        body_transform.header.frame_id = "world"
        body_transform.child_frame_id = f"{self.FRAME_PREFIX}base_link"
        body_transform.transform = msgTF

        self.joint_state.header.stamp = time_now_stamp
        self.joint_state_pub.publish(self.joint_state)
        self.tf_broadcaster.sendTransform(body_transform)

        self.refresh_timer.cancel()
        return

    def robot_body_pose_cbk(self, msg):
        tra, quat = self.tf2np(msg)
        self.current_body_xyz = tra
        self.current_body_quat = quat
        self.request_refresh()

    def smoother(self, x: NDArray) -> NDArray:
        """smoothes the interval [0, 1] to have a soft start and end
        (derivative is zero)
        """
        x = (1 - np.cos(x * np.pi)) / 2
        x = (1 - np.cos(x * np.pi)) / 2
        return x

    def smooth_body_trans(self, request: Transform):
        tra, quat = self.tf2np(request)
        final_coord = self.current_body_xyz + tra / 1000
        final_quat = self.current_body_quat * quat

        samples = int(self.MOVEMENT_TIME * self.MVMT_UPDATE_RATE)
        start_coord = self.current_body_xyz
        start_quat = self.current_body_quat

        x = np.linspace(0, 1, num=samples)  # x: [0->1]
        x = self.smoother(x)

        quaternion_interpolation = geometric_slerp(
            start=qt.as_float_array(start_quat), end=qt.as_float_array(final_quat), t=x
        )
        quaternion_interpolation = qt.as_quat_array(quaternion_interpolation)

        x = np.tile(x, (3, 1)).transpose()
        coord_interpolation = final_coord * x + start_coord * (1 - x)

        for i in range(1, samples):  # ]0->1]
            self.current_body_xyz = coord_interpolation[i, :]
            self.current_body_quat = quaternion_interpolation[i]

            self.request_refresh()

            self.sleep(1/self.MVMT_UPDATE_RATE)
        return

    def request_refresh(self):
        if self.refresh_timer.is_canceled():
            self.refresh_timer.reset()
            self.refresh_every_seconds_slow.reset()


def main(args=None):
    rclpy.init()
    joint_state_publisher = RVizInterfaceNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(joint_state_publisher)
    try:
        executor.spin()
    except KeyboardInterrupt:
        joint_state_publisher.get_logger().debug(
            "KeyboardInterrupt caught, node shutting down cleanly\nbye bye <3"
        )
    joint_state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
