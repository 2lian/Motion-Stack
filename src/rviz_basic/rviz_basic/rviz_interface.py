import time

import numpy as np
import rclpy
from rclpy.node import Node, ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import tf2_ros

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import TransformStamped, Transform


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
            Float64, f"angle_{self.leg}_{self.joint}", 10
        )

    def set_joint_cbk(self, msg):
        new_msg = Float64()
        # if self.joint == 0:
        # angle = msg.data
        # else:
        angle = -msg.data
        # if self.leg == 3 and self.joint == 2:
        # self.parent_node.get_logger().info(f'{angle} {self.joint} {self.leg}')
        new_msg.data = msg.data
        self.joint_state.position[self.leg * 3 + self.joint] = angle
        self.pub_back_to_ros2_structure.publish(new_msg)

        self.parent_node.request_refresh()
        return


class RVizInterfaceNode(Node):

    def __init__(self):
        # rclpy.init()
        super().__init__("joint_state_rviz")  #type: ignore

        #    node_list = self.get_node_names()
        rviz_is_running = False

        while not rviz_is_running:
            node_info = self.get_node_names_and_namespaces()
            for node_name, node_namespace in node_info:
                if node_name == "rviz2" or node_name == "rviz":
                    rviz_is_running = True
            self.get_logger().info(
                f"""Waiting for rviz, check that the [/rviz] node is running"""
            )
            time.sleep(1)

        self.get_logger().warning(f"""Rviz connected :)""")

        self.declare_parameter("std_movement_time", float(0.5))
        self.movement_time = (
            self.get_parameter("std_movement_time")
            .get_parameter_value()
            .double_value
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
        self.loop_rate = 30  # Hz

        self.cbk_legs = MutuallyExclusiveCallbackGroup()

        # V Subscriber V
        #   \  /   #
        #    \/    #
        cbk_holder_list = []
        for leg in range(4):
            for joint in range(3):
                holder = CallbackHolder(leg, joint, self, self.joint_state)
                cbk_holder_list.append(holder)

        self.body_pose_sub = self.create_subscription(
            Transform, "robot_body", self.robot_body_pose_cbk, 10
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
        self.iAmAlive = self.create_service(
            Empty, "rviz_interface_alive", lambda: None
        )
        #    /\    #
        #   /  \   #
        # ^ Service ^

        # V Timer V
        #   \  /   #
        #    \/    #
        self.refresh_timer = self.create_timer(1 / 100, self.refresh)
        self.refresh_every_seconds = self.create_timer(1, self.request_refresh)
        #    /\    #
        #   /  \   #
        # ^ Timer ^
        self.current_body_xyz = np.array([0, 0, 0.200], dtype=float)
        self.current_body_rot = np.array([0, 0, 0, 1], dtype=float)
        # self.movement_time = 1.5 # is a ros param
        self.movement_update_rate = 30

    def refresh(self, now=None):
        if now is None:
            now = self.get_clock().now()
        time_now_stamp = now.to_msg()
        xyz = self.current_body_xyz
        rot = self.current_body_rot
        msgTF = Transform()
        msgTF.translation.x, msgTF.translation.y, msgTF.translation.z = tuple(
            xyz.tolist()
        )
        msgTF.rotation.x, msgTF.rotation.y, msgTF.rotation.z, msgTF.rotation.w = (
            tuple(rot.tolist())
        )

        body_transform = TransformStamped()
        body_transform.header.stamp = time_now_stamp
        body_transform.header.frame_id = "world"
        body_transform.child_frame_id = "r1/base_link"
        body_transform.transform = msgTF

        self.joint_state.header.stamp = time_now_stamp
        self.joint_state_pub.publish(self.joint_state)
        self.tf_broadcaster.sendTransform(body_transform)

        self.refresh_timer.cancel()
        return

    def robot_body_pose_cbk(self, msg):
        tra = np.array(
            [msg.translation.x, msg.translation.y, msg.translation.z], dtype=float
        )
        rot = np.array(
            [msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w],
            dtype=float,
        )
        self.current_body_xyz = tra
        self.current_body_rot = rot

    def smooth_body_trans(self, request):
        tra = (
            self.current_body_xyz
            + np.array(
                [
                    request.translation.x,
                    request.translation.y,
                    request.translation.z,
                ],
                dtype=float,
            )
            / 1000
        )
        rot = (
            self.current_body_rot
            + np.array(
                [
                    request.rotation.x,
                    request.rotation.y,
                    request.rotation.z,
                    request.rotation.w,
                ],
                dtype=float,
            )
            / 1000
        )

        samples = int(self.movement_time * self.movement_update_rate)
        rate = self.create_rate(self.movement_update_rate)

        start_tra = self.current_body_xyz
        start_rot = self.current_body_rot

        for x in np.linspace(0 + 1 / samples, 1, num=samples - 1):
            x = (1 - np.cos(x * np.pi)) / 2
            intermediate_tra = tra * x + start_tra * (1 - x)
            intermediate_rot = rot * x + start_rot * (1 - x)

            self.current_body_xyz = intermediate_tra
            self.current_body_rot = intermediate_rot

            self.request_refresh()

            rate.sleep()
        return

    def request_refresh(self):
        if self.refresh_timer.is_canceled():
            self.refresh_timer.reset()
            self.refresh_every_seconds.reset()


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
