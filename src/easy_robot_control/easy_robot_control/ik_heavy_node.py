"""
This node is responsible for recieving targets in the body reference frame, and send the
corresponding angles to the motors.

Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

from EliaNode import EliaNode
from typing import List
import numpy as np
import time
from numpy.typing import NDArray
import quaternion as qt
import rclpy
from rclpy.node import Node
from roboticstoolbox.robot.ET import SE3
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3
import python_package_include.inverse_kinematics as ik
from roboticstoolbox import ET
import roboticstoolbox as rtb

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
    def __init__(self, leg: int, joint: int, parent_node):
        self.leg = leg
        self.joint = joint
        self.parent_node = parent_node
        self.parent_node.create_subscription(
            Float64, f"angle_{self.leg}_{self.joint}", self.angle_received_from_below, 10
        )

        self.to_angle_below = self.parent_node.create_publisher(
            Float64, f"set_joint_{self.leg}_{self.joint}_real", 10
        )

    @error_catcher
    def angle_received_from_below(self, msg):
        """recieves angle reading from joint, stores value in array.
        Starts timer to publish new tip position.

        Args:
            msg: Ros2 Float64 - angle reading
        """
        self.parent_node.joints_angle_arr[self.joint] = msg.data
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

        self.ETchain = (
            ET.tx(0)
            * ET.Rz(self.leg_num * np.pi / 2)
            * ET.tx(bodyToCoxa)
            * ET.Rz(qlim=np.array([coxaMin, coxaMax]))
            * ET.tx(coxaLength)
            * ET.Ry(flip=True, qlim=np.array([femurMin, femurMax]))
            * ET.tx(femurLength)
            * ET.Ry(flip=True, qlim=np.array([tibiaMin, tibiaMax]))
            * ET.tx(tibiaLength)
        )
        self.model = rtb.Robot(self.ETchain)

        # self.get_logger().info(f"{self.model}")
        # self.get_logger().info(f"leg[{self.leg_num}] kinematic chain:\n{self.model}")
        # self.get_logger().info(f"leg[{self.leg_num}] {self.model.links[0]}")

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
        self.cbk_holder_list: List[JointCallbackHolder] = []
        for joint in range(len(self.model.links) - 1):
            self.cbk_holder_list.append(JointCallbackHolder(self.leg_num, joint, self))

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
        self.forwardKinemticsTimer = self.create_timer(0.01, self.publish_tip_pos)
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
        target: NDArray = np.array([msg.x, msg.y, msg.z], dtype=float)
        motion: SE3 = SE3(target)

        ik_result = self.model.ik_LM(
            Tep=motion,
            q0=self.joints_angle_arr,
            mask=[True, True, True, False, False, False],
            ilimit=10,
            slimit=3,
        )
        angles = ik_result[0]

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
        fw_result: List[SE3] = self.model.fkine_all(self.joints_angle_arr) # type: ignore
        tip_coord: NDArray = fw_result[-1].t
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
