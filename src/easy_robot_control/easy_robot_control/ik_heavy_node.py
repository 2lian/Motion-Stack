"""
This node is responsible for recieving targets in the body reference frame, and send the
corresponding angles to the motors.

Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

from EliaNode import EliaNode
from typing import List, Optional
import numpy as np
import time
from numpy.typing import NDArray
import quaternion as qt
import rclpy
from rclpy.node import Node
from roboticstoolbox.robot.ET import SE3
from roboticstoolbox.tools import URDF
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3
import python_package_include.inverse_kinematics as ik
from roboticstoolbox import ET, ETS, Link, Robot
import roboticstoolbox as rtb
import traceback

from easy_robot_control.EliaNode import loadAndSet_URDF


def error_catcher(func):
    # This is a wrapper to catch and display exceptions
    def wrap(*args, **kwargs):
        try:
            out = func(*args, **kwargs)
        except Exception as exception:
            if exception is KeyboardInterrupt:
                raise KeyboardInterrupt
            else:
                traceback_logger_node = Node("error_node")  # type: ignore
                traceback_logger_node.get_logger().error(traceback.format_exc())
                raise KeyboardInterrupt
        return out

    return wrap


class JointCallbackHolder:
    def __init__(self, joint_name: str, index: int, parent_node):
        self.joint_name = joint_name
        self.index = index
        self.parent_node = parent_node
        self.parent_node.create_subscription(
            Float64,
            f"read_{self.joint_name}",
            self.angle_received_from_below,
            10,
        )

        self.to_angle_below = self.parent_node.create_publisher(
            Float64, f"set_{self.joint_name}", 10
        )

    @error_catcher
    def angle_received_from_below(self, msg):
        """recieves angle reading from joint, stores value in array.
        Starts timer to publish new tip position.

        Args:
            msg: Ros2 Float64 - angle reading
        """
        self.parent_node.joints_angle_arr[self.index] = msg.data
        if self.parent_node.forwardKinemticsTimer.is_canceled():
            self.parent_node.forwardKinemticsTimer.reset()

    @error_catcher
    def publish_angle_below(self, angle: float) -> None:
        out_msg = Float64()
        out_msg.data = angle
        self.to_angle_below.publish(out_msg)


class IKNode(EliaNode):
    def __init__(self):
        super().__init__(f"ik_node")  # type: ignore
        self.NAMESPACE = self.get_namespace()
        # self.WAIT_FOR_NODES_OF_LOWER_LEVEL = True
        self.WAIT_FOR_NODES_OF_LOWER_LEVEL = False

        self.declare_parameter("leg_number", 0)
        self.leg_num = (
            self.get_parameter("leg_number").get_parameter_value().integer_value
        )
        if self.leg_num == 0:
            self.Yapping = True
        else:
            self.Yapping = False
        self.Alias = f"IK{self.leg_num}"

        self.necessary_clients = [f"rviz_interface_alive", f"remapper_alive"]
        self.setAndBlockForNecessaryClients(self.necessary_clients, all_requiered=False)

        # V Parameters V
        #   \  /   #
        #    \/    #
        self.declare_parameter("urdf_path", str())
        self.urdf_path = (
            self.get_parameter("urdf_path").get_parameter_value().string_value
        )
        leg_num_remapping = [3, 4, 1, 2]
        self.declare_parameter(
            "end_effector_name", str(f"end{leg_num_remapping[self.leg_num]}")
        )
        self.end_effector = (
            self.get_parameter("end_effector_name").get_parameter_value().string_value
        )
        (
            self.model,
            self.ETchain,
            self.joint_names,
            self.joints_objects,
            self.joint_index,
        ) = loadAndSet_URDF(self.urdf_path, self.end_effector)

        for et in self.ETchain:
            et: ET
            if et.qlim is not None:
                if et.qlim[0] == 0.0 and et.qlim[1] == 0.0 or True:
                    et.qlim = None
        # self.ETchain: ETS
        self.ETchain = ETS(self.ETchain.compile())

        self.end_link = [x for x in self.model.links if x.name == self.end_effector][0]
        self.pinfo(f"Last link is: {self.end_link}")
        self.pinfo(f"Kinematic chain is:\n{self.ETchain}")
        # self.pinfo(f"Kinematic chain is:\n{self.ETchain.__dict__}")
        self.pinfo(f"Ordered joints names are: {self.joint_names}")
        # m = ETS(self.ETchain)
        # self.pwarn(f"\n{m}")
        # self.pinfo(f"Kinematic chain is:\n{self.ETchain.__dict__}")

        # self.subChain: Robot = rtb.Robot(self.ETchain)
        #    /\    #
        #   /  \   #
        # ^ Parameters ^

        # self.ETchain = (
        #     ET.tx(0)
        #     * ET.Rz(self.leg_num * np.pi / 2)
        #     * ET.tx(bodyToCoxa)
        #     * ET.Rz(qlim=np.array([coxaMin, coxaMax]))
        #     * ET.tx(coxaLength)
        #     * ET.Ry(flip=True, qlim=np.array([femurMin, femurMax]))
        #     * ET.tx(femurLength)
        #     * ET.Ry(flip=True, qlim=np.array([tibiaMin, tibiaMax]))
        #     * ET.tx(tibiaLength)
        # )
        # self.model = rtb.Robot(self.ETchain)

        # with open(self.urdf_path, "r") as file:
        # self.urdf_content = file.read()
        # self.pinfo(model.name)
        # self.pinfo(model.links)
        # self.pinfo(model)
        # d = pickle.dumps(e.data[0], protocol=pickle.HIGHEST_PROTOCOL)
        # l = pickle.loads(d)

        # self.get_logger().info(f"{self.model}")
        # self.get_logger().info(f"leg[{self.leg_num}] kinematic chain:\n{self.model}")
        # self.get_logger().info(f"leg[{self.leg_num}] {self.model.links[0]}")

        self.joints_angle_arr = np.zeros(self.ETchain.n, dtype=float)

        # V Publishers V
        #   \  /   #
        #    \/    #
        self.cbk_holder_list: List[JointCallbackHolder] = []
        for index, name in enumerate(self.joint_names):
            corrected_name = name.replace("-", "_")
            self.cbk_holder_list.append(JointCallbackHolder(corrected_name, index, self))

        self.pub_tip = self.create_publisher(Vector3, f"tip_pos_{self.leg_num}", 10)
        #    /\    #
        #   /  \   #
        # ^ Publishers ^

        # V Subscribers V
        #   \  /   #
        #    \/    #
        self.sub_rel_target = self.create_subscription(
            Vector3, f"set_ik_target_{self.leg_num}", self.ik_target_received, 10
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
        self.forwardKinemticsTimer = self.create_timer(0.1, self.publish_tip_pos)
        self.forwardKinemticsTimer.cancel()  # this timer executes 0.01 after every new angle received
        #    /\    #
        #   /  \   #
        # ^ Timers ^

    @error_catcher
    def ik_target_received(self, msg: Vector3) -> None:
        """recieves target from leg, converts to numpy, computes IK, sends angle results to joints

        Args:
            msg: target as Ros2 Vector3
        """
        target: NDArray = np.array([msg.x, msg.y, msg.z], dtype=float) / 1000
        motion: SE3 = SE3(target)

        m = rtb.Robot(self.ETchain)
        # ik_result = m.ik_LM(
        ik_result = self.ETchain.ik_LM(
        # ik_result = self.ETchain.ikine_LM(
            Tep=motion,
            q0=self.joints_angle_arr,
            mask=np.array([1, 1, 1, 0, 0, 0], dtype = float),
            ilimit=10,
            slimit=3,
            joint_limits=False,
        )
        angles = ik_result[0]
        # angles = ik_result.q[-3:]
        # self.pwarn(np.round(target * 1000))
        # self.pwarn(ik_result)
        # self.pwarn(np.round(np.rad2deg(angles)))

        for i in range(len(self.cbk_holder_list)):
            cbk_holder = self.cbk_holder_list[i]
            angle = angles[i]
            cbk_holder.publish_angle_below(angle)

    @error_catcher
    def publish_tip_pos(self) -> None:
        """Computes foward kinematics given angles stored in array,
        publishes tip position result.
        This is executed x ms after an angle reading is received"""
        msg = Vector3()
        m = (self.ETchain)
        fw_result: List[SE3] = self.ETchain.fkine(self.joints_angle_arr)  # type: ignore
        tip_coord: NDArray = fw_result[-1].t * 1000
        # self.get_logger().warn(f"{tip_coord}")
        msg.x = tip_coord[0]
        msg.y = tip_coord[1]
        msg.z = tip_coord[2]
        self.pub_tip.publish(msg)
        self.forwardKinemticsTimer.cancel()


def main():
    rclpy.init()
    joint_state_publisher = IKNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    # executor = rclpy.executors.MultiThreadedExecutor()
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
