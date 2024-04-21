"""
This node is responsible for recieving targets in the body reference frame, and send the
corresponding angles to the motors.

Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

import numpy as np
import time
import quaternion as qt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3
import python_package_include.inverse_kinematics as ik

WAIT_FOR_NODES_OF_LOWER_LEVEL = True


class IKNode(Node):
    def __init__(self):
        super().__init__(f"ik_node")  # type: ignore
        self.NAMESPACE = self.get_namespace()

        self.necessary_clients = [
            self.create_client(Empty, f"rviz_interface_alive"),
            self.create_client(Empty, f"remapper_alive"),
        ]
        while not any(
            [client.wait_for_service(timeout_sec=1) for client in self.necessary_clients]
        ):
            self.get_logger().warning(
                f"""Waiting for node, check that the [rviz_interface_alive or dynamixel_interface_alive] service is running"""
            )
            if not WAIT_FOR_NODES_OF_LOWER_LEVEL:
                break

        self.get_logger().warning(f"""Lower level connected :)""")

        self.declare_parameter("leg_number", 0)
        self.leg_num = (
            self.get_parameter("leg_number").get_parameter_value().integer_value
        )

        # V Parameters V
        #   \  /   #
        #    \/    #
        self.declare_parameter("bodyToCoxa", float())
        self.declare_parameter("coxaLength", float())
        self.declare_parameter("femurLength", float())
        self.declare_parameter("tibiaLength", float())

        self.declare_parameter("coxaMax", float())
        self.declare_parameter("coxaMin", float())
        self.declare_parameter("femurMax", float())
        self.declare_parameter("femurMin", float())
        self.declare_parameter("tibiaMax", float())
        self.declare_parameter("tibiaMin", float())

        bodyToCoxa = self.get_parameter("bodyToCoxa").get_parameter_value().double_value
        coxaLength = self.get_parameter("coxaLength").get_parameter_value().double_value
        femurLength = self.get_parameter("femurLength").get_parameter_value().double_value
        tibiaLength = self.get_parameter("tibiaLength").get_parameter_value().double_value

        coxaMax = self.get_parameter("coxaMax").get_parameter_value().double_value
        coxaMin = self.get_parameter("coxaMin").get_parameter_value().double_value
        femurMax = self.get_parameter("femurMax").get_parameter_value().double_value
        femurMin = self.get_parameter("femurMin").get_parameter_value().double_value
        tibiaMax = self.get_parameter("tibiaMax").get_parameter_value().double_value
        tibiaMin = self.get_parameter("tibiaMin").get_parameter_value().double_value
        #    /\    #
        #   /  \   #
        # ^ Parameters ^

        self.leg_param = ik.LegParameters(
            mounting_point=np.array([bodyToCoxa, 0, 0], dtype=float),
            mounting_quaternion=qt.from_rotation_vector(np.zeros(3)),
            coxa_lengthX=coxaLength,
            coxa_lengthZ=0,
            femur_length=femurLength,
            tibia_length=tibiaLength,
            coxaMax_degree=np.rad2deg(coxaMax),
            coxaMin_degree=np.rad2deg(coxaMin),
            femurMax_degree=np.rad2deg(femurMax),
            femurMin_degree=np.rad2deg(femurMin),
            tibiaMax_degree=np.rad2deg(tibiaMax),
            tibiaMin_degree=np.rad2deg(tibiaMin),
        )

        leg_offset_quat = ik.PI_OVER_2_Z_QUAT ** (self.leg_num)
        self.leg_param = ik.rotate_legparam_by(self.leg_param, leg_offset_quat)

        self.joints_angle_arr = np.zeros(3, dtype=float)

        # V Publishers V
        #   \  /   #
        #    \/    #
        self.joint_pub_list = []
        for joint in range(3):
            pub = self.create_publisher(
                Float64, f"set_joint_{self.leg_num}_{joint}_real", 10
            )
            self.joint_pub_list.append(pub)

        self.pub_tip = self.create_publisher(Vector3, f"tip_pos_{self.leg_num}", 10)
        #    /\    #
        #   /  \   #
        # ^ Publishers ^

        # V Subscribers V
        #   \  /   #
        #    \/    #
        self.sub_rel_target = self.create_subscription(
            Vector3, f"set_ik_target_{self.leg_num}", self.set_ik_cbk, 10
        )
        self.sub_angle_0 = self.create_subscription(
            Float64, f"angle_{self.leg_num}_0", self.angle_0_cbk, 10
        )
        self.sub_angle_1 = self.create_subscription(
            Float64, f"angle_{self.leg_num}_1", self.angle_1_cbk, 10
        )
        self.sub_angle_2 = self.create_subscription(
            Float64, f"angle_{self.leg_num}_2", self.angle_2_cbk, 10
        )
        #    /\    #
        #   /  \   #
        # ^ Subscribers ^

        # V Service V
        #   \  /   #
        #    \/    #
        self.iAmAlive = self.create_service(
            Empty, f"ik_{self.leg_num}_alive", lambda req, res: res
        )
        #    /\    #
        #   /  \   #
        # ^ Service ^

        # V Timers V
        #   \  /   #
        #    \/    #
        self.forwardKinemticsTimer = self.create_timer(0.001, self.publish_tip_pos)
        self.forwardKinemticsTimer.cancel()  # this timer executes 0.001 after every new angle received
        #    /\    #
        #   /  \   #
        # ^ Timers ^

    def set_ik_cbk(self, msg: Vector3) -> None:
        """recieves target from leg, converts to numpy, computes IK, sends angle results to joints

        Args:
            msg: target as Ros2 Vector3
        """
        target = np.array([msg.x, msg.y, msg.z], dtype=float)
        angles = ik.leg_ik(
            target=target,
            previous_angles=self.joints_angle_arr,
            leg_param=self.leg_param,
        )
        for joint in range(3):
            out_msg = Float64()
            out_msg.data = angles[joint]
            self.joint_pub_list[joint].publish(out_msg)

    def angle_0_cbk(self, msg: Float64) -> None:
        """recieves angle reading from joint, stores value in array.
        Starts timer to publish new tip position.

        Args:
            msg: Ros2 Float64 - angle reading
        """
        self.joints_angle_arr[0] = msg.data
        if self.forwardKinemticsTimer.is_canceled():
            self.forwardKinemticsTimer.reset()

    def angle_1_cbk(self, msg: Float64) -> None:
        """recieves angle reading from joint, stores value in array.
        Starts timer to publish new tip position.

        Args:
            msg: Ros2 Float64 - angle reading
        """
        self.joints_angle_arr[1] = msg.data
        if self.forwardKinemticsTimer.is_canceled():
            self.forwardKinemticsTimer.reset()

    def angle_2_cbk(self, msg: Float64) -> None:
        """recieves angle reading from joint, stores value in array.
        Starts timer to publish new tip position.

        Args:
            msg: Ros2 Float64 - angle reading
        """
        self.joints_angle_arr[2] = msg.data
        if self.forwardKinemticsTimer.is_canceled():
            self.forwardKinemticsTimer.reset()

    def publish_tip_pos(self) -> None:
        """Computes foward kinematics given angles stored in array,
        publishes tip position result.
        This is executed x ms after an angle reading is received"""
        msg = Vector3()
        tip_pos = ik.forward_kine_body_zero(self.joints_angle_arr, self.leg_param)[-1, :]
        msg.x = tip_pos[0]
        msg.y = tip_pos[1]
        msg.z = tip_pos[2]
        self.pub_tip.publish(msg)
        self.forwardKinemticsTimer.cancel()


def main():
    rclpy.init()
    joint_state_publisher = IKNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(joint_state_publisher)
    try:
        executor.spin()
    except KeyboardInterrupt as e:
        joint_state_publisher.get_logger().debug(
            "KeyboardInterrupt caught, node shutting down cleanly\nbye bye <3"
        )
    joint_state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
