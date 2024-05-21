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
from geometry_msgs.msg import Transform, Vector3
import python_package_include.inverse_kinematics as ik
from roboticstoolbox import ET, ETS, Link, Robot
import roboticstoolbox as rtb
import traceback

from easy_robot_control.EliaNode import loadAndSet_URDF
from easy_robot_control.EliaNode import loadAndSet_URDF, replace_incompatible_char_ros2


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
                raise exception
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
        self.WAIT_FOR_NODES_OF_LOWER_LEVEL = True
        # self.WAIT_FOR_NODES_OF_LOWER_LEVEL = False

        self.declare_parameter("leg_number", 0)
        self.leg_num = (
            self.get_parameter("leg_number").get_parameter_value().integer_value
        )
        if self.leg_num == 2:
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
        leg_num_remapping = [3, 0, 1, 2, 4]
        # leg_num_remapping = [0, 1, 2, 3]
        self.declare_parameter(
            "end_effector_name", str(f"{leg_num_remapping[self.leg_num]}")
        )
        end_effector: str = (
            self.get_parameter("end_effector_name").get_parameter_value().string_value
        )
        self.end_effector_name: str | int
        if end_effector.isdigit():
            self.end_effector_name = int(end_effector)
        else:
            if end_effector == "":
                self.end_effector_name = self.leg_num
            else:
                self.end_effector_name = end_effector

        (
            self.model,
            self.ETchain,
            self.joint_names,
            self.joints_objects,
            self.last_link,
        ) = loadAndSet_URDF(self.urdf_path, self.end_effector_name)

        self.ETchain: ETS
        self.subModel: Robot = rtb.Robot(self.ETchain)
        # self.ETchain = ETS(self.ETchain.compile())

        self.end_link = self.last_link
        if type(self.end_effector_name) is int:
            self.pwarn(
                f"Precise end effector name not given. IK leg {self.leg_num} is using [{self.end_link.name}] as end effector.",
                force=True,
            )
        self.pinfo(f"Last link is: {self.end_link}")
        # self.pinfo(f"Whole model is:\n{self.model}")
        self.pinfo(f"Sub model is:\n{self.subModel}")
        self.pinfo(f"Kinematic chain is:\n{self.ETchain}")
        self.joints_angle_arr = np.zeros(self.ETchain.n, dtype=float)
        # self.pwarn(f"{self.joints_angle_arr}")
        chain = self.ETchain.copy()
        prev = np.zeros(3, dtype=float)
        for i in range(self.ETchain.m):
            fw_result: List[SE3] = chain.fkine(q=np.zeros(chain.n, dtype=float))
            coord = np.round(fw_result[0].t, decimals=3)
            chain.pop()
            if not np.all(np.isclose(prev, coord)):
                self.pwarn(coord)
                prev = coord
        # self.pinfo(f"Kinematic chain is:\n{self.ETchain.__dict__}")
        self.pinfo(f"Ordered joints names are: {self.joint_names}")
        # used_joints = [j for j in self.joints_objects if j.name in self.joint_names]
        # self.pwarn([j.__dict__ for j in used_joints][0])
        # links, name, urdf_string, urdf_filepath = rtb.Robot.URDF_read(
        #     file_path=self.urdf_path
        # )
        # joints_objects = URDF.loadstr(urdf_string, urdf_filepath)
        # self.pwarn(joints_objects.__dict__)
        # for s in self.model.segments():
        # self.pinfo(f"{[x.name for x in s if x is not None]}")
        # m = ETS(self.ETchain)
        # self.pwarn(f"\n{m}")
        # self.pinfo(f"Kinematic chain is:\n{self.ETchain.__dict__}")

        #    /\    #
        #   /  \   #
        # ^ Parameters ^

        self.joints_angle_arr = np.zeros(len(self.joint_names), dtype=float)

        # V Publishers V
        #   \  /   #
        #    \/    #
        self.cbk_holder_list: List[JointCallbackHolder] = []
        for index, name in enumerate(self.joint_names):
            corrected_name = replace_incompatible_char_ros2(name)
            self.cbk_holder_list.append(JointCallbackHolder(corrected_name, index, self))

        self.pub_tip = self.create_publisher(Transform, f"tip_pos_{self.leg_num}", 10)
        #    /\    #
        #   /  \   #
        # ^ Publishers ^

        # V Subscribers V
        #   \  /   #
        #    \/    #
        self.sub_rel_target = self.create_subscription(
            Transform, f"set_ik_target_{self.leg_num}", self.ik_target_received, 10
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
    def ik_target_received(self, msg: Transform) -> None:
        """recieves target from leg, converts to numpy, computes IK, sends angle results to joints

        Args:
            msg: target as Ros2 Vector3
        """
        xyz, quat = self.tf2np(msg)
        xyz /= 1_000  # to mm
        # self.pwarn(np.round(xyz, 0))
        # self.pwarn(np.round(qt.as_float_array(quat), 1))
        motion: SE3 = SE3(xyz) 
        motion.A[:3, :3] = qt.as_rotation_matrix(quat)
        # motion: SE3 = SE3(xyz) * SE3(qt.as_rotation_matrix(quat))
        # motion: SE3 = SE3(qt.as_rotation_matrix(quat)) * SE3(xyz)
        # self.pwarn(motion)
        # self.pwarn(SE3(qt.as_rotation_matrix(quat)))

        ik_result = self.subModel.ik_LM(
            # ik_result = self.subModel.ik_NR(
            Tep=motion,
            q0=self.joints_angle_arr,
            mask=np.array([1, 1, 1, 1, 1, 1], dtype=float),
            ilimit=30,
            # slimit=3,
            joint_limits=False,
            # pinv=True,
            # tol=0.001,
        )
        angles = ik_result[0]
        # angles = ik_result.q[:]
        # self.pwarn(np.round(target * 1000))
        # self.pwarn(ik_result)
        # self.pwarn(np.round(np.rad2deg(angles)))

        # chain = self.ETchain.copy()
        # for i in range(self.ETchain.m):
        #     fw_result: List[SE3] = chain.fkine(q=angles)
        #     self.pwarn(np.round(fw_result[0].t, decimals=3))
        #     chain.pop()

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
        fw_result: List[SE3] = self.subModel.fkine(self.joints_angle_arr)  # type: ignore
        tip_coord: NDArray = fw_result[-1].t * 1000
        tip_quat: qt.quaternion = qt.from_rotation_matrix(
            np.array(fw_result[-1].R, dtype=float)
        )
        # self.pwarn(np.round(tip_coord))
        # self.pwarn(np.round(qt.as_float_array(tip_quat)))
        msg = self.np2tf(coord=tip_coord, quat=tip_quat)
        self.pub_tip.publish(msg)
        self.forwardKinemticsTimer.cancel()

        # chain = self.ETchain.copy()
        # for i in range(self.ETchain.m):
        #     fw_result: List[SE3] = chain.fkine(q=self.joints_angle_arr)
        #     self.pwarn(np.round(fw_result[0].t, decimals=3))
        #     chain.pop()


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
