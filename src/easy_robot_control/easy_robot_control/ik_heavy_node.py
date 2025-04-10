"""
This node is responsible for recieving targets in the body reference frame, and send the
corresponding angles to the motors.

Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

import matplotlib
from sensor_msgs.msg import JointState

matplotlib.use("Agg")  # fix for when there is no display

from typing import List, Optional, Tuple, Union

import numpy as np
import quaternion as qt
import roboticstoolbox as rtb
from geometry_msgs.msg import Transform, Vector3
from numpy.typing import NDArray
from rclpy.node import Node, Parameter, Service, Timer
from rclpy.time import Duration, Time
from roboticstoolbox import ET, ETS, Link, Robot
from roboticstoolbox.robot.ET import SE3
from roboticstoolbox.tools import Joint
from std_msgs.msg import Float64
from std_srvs.srv import Empty

from easy_robot_control.EliaNode import (
    EliaNode,
    EZRate,
    error_catcher,
    loadAndSet_URDF,
    myMain,
    np2tf,
    replace_incompatible_char_ros2,
    rosTime2Float,
    tf2np,
    transform_joint_to_transform_Rx,
)

float_formatter = "{:.2f}".format
np.set_printoptions(formatter={"float_kind": float_formatter})

# IK_MAX_VEL = 0.003  # changes depending on the refresh rate idk why. This is bad
IK_MAX_VEL = (
    1  # changes depending on the refresh rate and dimensions idk why. This is bad
)


class WheelMiniNode:
    def __init__(self, joint_name: str, wheel_size_mm: float, parent_node: EliaNode):
        self.joint_name = joint_name
        self.wheel_radius = wheel_size_mm
        self.parent_node = parent_node
        self.angularSpeed = 0

        self.to_angle_below = self.parent_node.create_publisher(
            Float64, f"spe_{self.joint_name}_set", 10
        )
        # self.last_sent: Time = self.parent_node.get_clock().now()
        # self.angle_update_cooldown = Duration(seconds=1, nanoseconds=0)

    def publish_speed_below(self, speed: float) -> None:
        """Sends speed to nodes below

        Args:
            angle float:
        """
        # self.parent_node.pwarn("speed sent", force = True)
        out_msg = Float64()
        out_msg.data = speed
        self.to_angle_below.publish(out_msg)

    def roll(self, speed: float) -> None:
        """Increases the angular speed correspongin to the linear speed

        Args:
            distance float: distance to roll
        """
        self.angularSpeed = speed / (self.wheel_radius)
        # self.parent_node.pwarn(
        # f"speed mm {speed}, speed angular {self.angularSpeed}", force=False
        # )
        self.publish_speed_below(self.angularSpeed)
        # self.last_sent: Time = self.parent_node.get_clock().now()


class IKNode(EliaNode):
    def __init__(self):
        super().__init__(f"ik")  # type: ignore
        self.NAMESPACE = self.get_namespace()
        self.WAIT_FOR_NODES_OF_LOWER_LEVEL = True
        self.RESET_LAST_SENT: Duration = Duration(seconds=0.5)  # type: ignore
        self.WAIT_ANGLE_MES: Duration = Duration(seconds=2)
        self.WAIT_ANGLE_ABORT: Duration = Duration(seconds=4)
        # self.WAIT_FOR_NODES_OF_LOWER_LEVEL = False

        self.declare_parameter("leg_number", 0)
        self.leg_num = (
            self.get_parameter("leg_number").get_parameter_value().integer_value
        )
        if self.leg_num == 3:
            self.Yapping = True
        else:
            self.Yapping = True
        self.Alias = f"IK{self.leg_num}"

        self.necessary_clients = ["joint_alive"]
        self.wait_for_lower_level(self.necessary_clients)

        # V Parameters V
        #   \  /   #
        #    \/    #
        self.declare_parameter("mvmt_update_rate", float(10))
        self.REFRESH_RATE: float = (
            self.get_parameter("mvmt_update_rate").get_parameter_value().double_value
        )

        self.declare_parameter("wheel_size_mm", float(100.0))
        self.wheel_size_mm = (
            self.get_parameter("wheel_size_mm").get_parameter_value().double_value
        )

        self.declare_parameter("ignore_limits", False)
        self.IGNORE_LIM = (
            self.get_parameter("ignore_limits").get_parameter_value().bool_value
        )

        self.declare_parameter("urdf_path", str())
        self.urdf_path = (
            self.get_parameter("urdf_path").get_parameter_value().string_value
        )

        self.declare_parameter("start_effector_name", str(f""))
        self.start_effector: str | None = (
            self.get_parameter("start_effector_name").get_parameter_value().string_value
        )
        if self.start_effector == "":
            self.start_effector = None

        leg_num_remapping = range(1000)
        self.declare_parameter("end_effector_name", str(f"{self.leg_num}"))
        end_effector: str = (
            self.get_parameter("end_effector_name").get_parameter_value().string_value
        )

        self.end_effector_name: Union[str, int]
        if end_effector.isdigit():
            self.end_effector_name = int(end_effector)
        else:
            if end_effector == "":
                self.end_effector_name = self.leg_num
            else:
                self.end_effector_name = end_effector

        if isinstance(self.end_effector_name, int):
            self.end_effector_name = leg_num_remapping[self.end_effector_name]
            new_param_value = Parameter(
                "end_effector_name", Parameter.Type.STRING, f"{self.end_effector_name}"
            )
            self.set_parameters([new_param_value])

        # we need double traversal here
        (
            self.model,
            self.ETchain,
            self.joint_names,
            self.joints_objects,
            self.last_link,
        ) = loadAndSet_URDF(self.urdf_path, self.end_effector_name, self.start_effector)

        try:
            if isinstance(self.end_effector_name, str) and isinstance(
                self.start_effector, str
            ):
                (  # don't want to see this
                    model2,
                    ETchain2,
                    joint_names2,
                    joints_objects2,
                    last_link2,
                ) = loadAndSet_URDF(
                    self.urdf_path, self.start_effector, self.end_effector_name
                )
                if len(joint_names2) > len(self.joint_names) and len(
                    joints_objects2
                ) > len(self.joints_objects):
                    joint_names2.reverse()
                    self.joint_names = joint_names2
                    joints_objects2.reverse()
                    self.joints_objects = joints_objects2
        except:
            self.pinfo(f"link tree could not be reversed")

        # self.pinfo(self.model)
        self.ETchain: ETS
        # self.pwarn(len(self.joints_objects))
        # self.ETchain = ETS(self.ETchain.compile())

        self.end_link: Link = self.last_link
        self.pinfo(f"Kinematic chain is:\n{self.ETchain}")
        chain = self.ETchain.copy()
        prev = np.zeros(3, dtype=float)
        counter = 0
        was_joint = False
        coordinate_info = f"Default forward kinematics of joints:"
        for i in range(self.ETchain.m):
            fw_result: List[SE3] = chain.fkine(q=np.zeros(chain.n, dtype=float))
            coord = np.round(fw_result[0].t, decimals=3)
            j: ET = chain.pop()
            if j.isjoint:
                was_joint = True
            if not np.all(np.isclose(prev, coord)):
                if not was_joint:
                    coordinate_info += f"\nFixed: {coord}"
                elif counter >= len(self.joint_names):
                    coordinate_info += f"\nOut of range?: {coord}"
                else:
                    coordinate_info += f"\n{(self.joint_names)[-counter-1]}: {coord}"
                    counter += 1
                    was_joint = False
                prev = coord
        self.pinfo(coordinate_info)

        self.wheels = []

        for wheels_start_effector in (
            self.end_link.children if self.end_link.children is not None else []
        ):  # looks for wheels in the current end effector children
            (
                modelw,
                ETchainw,
                joint_namesw,
                joints_objectsw,
                last_linkw,
            ) = loadAndSet_URDF(
                self.urdf_path, 0, start_effector_name=wheels_start_effector.name
            )

            isNotWheel = False

            revolutCount = 0
            # self.pwarn(joint_namesw)
            for jt in joints_objectsw:
                if jt is None:
                    continue
                if jt.joint_type == "fixed":
                    continue
                elif jt.joint_type in ["revolute", "continuous"]:
                    revolutCount += 1
                    if revolutCount >= 2:
                        isNotWheel = True
                        self.pinfo(
                            f"Wheel {wheels_start_effector.name} rejected because more than 1 joint",
                            force=True,
                        )
                        break
                else:
                    self.pinfo(
                        f"Wheel {wheels_start_effector.name} rejected: joint is [{jt.joint_type}], not revolte or continuous",
                        force=True,
                    )
                    isNotWheel = True
                    break

            if revolutCount <= 0 and not isNotWheel:
                self.pinfo(
                    f"Wheel {wheels_start_effector.name} rejected: not enough joints",
                    force=True,
                )
                isNotWheel = True

            if not isNotWheel:
                self.wheels.append(
                    (modelw, ETchainw, joint_namesw, joints_objectsw, last_linkw)
                )

        if self.wheels:
            self.pinfo(f"Wheels joints: {[x[2] for x in self.wheels]}", force=True)

        self.wheel_axis: Optional[ET] = None
        if self.wheels:  # first wheel will define the axis
            (modelw, ETchainw, joint_namesw, joints_objectsw, last_linkw) = self.wheels[0]
            ETchainw = ETchainw.compile()
            e: ET = transform_joint_to_transform_Rx(ETchainw[0], ETchainw[1])
            self.wheel_axis: ET = e
            y = e.A()[0, :3]
            # if y[1] < 0:
            # y *= -1
            z_pure = np.array([0, 0, 1], dtype=float)
            x = np.cross(y, z_pure)
            z = np.cross(x, y)

            x, y, z = x / np.linalg.norm(x), y / np.linalg.norm(y), z / np.linalg.norm(z)
            rot_matrix = np.empty((3, 3), dtype=float)
            rot_matrix[0, :] = x
            rot_matrix[1, :] = y
            rot_matrix[2, :] = z
            se = SE3()
            se.A[:3, :3] = rot_matrix
            e = ET.SE3(se)
            # self.pinfo(np.round(rot_matrix, 2))
            self.pinfo(
                f"Effector rotated on wheel axis: \n\
 forward vect: {np.round(x)}\n\
 axis vect: {np.round(y)}\n\
 steering vect: {np.round(z)}"
            )
            self.ETchain.append(e)
            # self.pinfo(self.ETchain)

        # self.ETchain = self.all_limits(self.ETchain, self.joints_objects)
        self.subModel: Robot = rtb.Robot(self.ETchain)
        self.pinfo(
            f"Using base link: {self.model.base_link.name} "
            f"end effector: {self.end_link.name}"
        )
        #    /\    #
        #   /  \   #
        # ^ Parameters ^

        self.angleReadings: NDArray = np.empty(len(self.joint_names), dtype=float)
        self.angleReadings[:] = np.nan
        self.last_sent: NDArray = self.angleReadings.copy()
        self.prevIKstamp: Time = self.get_clock().now()

        # V Publishers V
        #   \  /   #
        #    \/    #
        self.wheelList: List[WheelMiniNode] = []
        for wheel in self.wheels:
            self.wheel_axis: ET
            (modelw, ETchainw, joint_namesw, joints_objectsw, last_linkw) = wheel
            ETchainw = ETchainw.compile()

            axis_transf = transform_joint_to_transform_Rx(ETchainw[0], ETchainw[1])
            ax1: SE3 = SE3(self.wheel_axis.A())
            ax2: SE3 = SE3(axis_transf.A())
            diff: SE3 = ax1 / ax2
            diff_to_unit: NDArray = diff.A - SE3().A

            needs_to_be_flipped = not np.all(np.isclose(a=diff_to_unit, b=0, atol=0.1))
            if needs_to_be_flipped:
                wheel_size = self.wheel_size_mm * -1
                self.pinfo(f"Wheel {wheel[2][0]} flipped.")
            else:
                wheel_size = self.wheel_size_mm

            for index, name in enumerate(wheel[2]):
                corrected_name = replace_incompatible_char_ros2(name)
                self.wheelList.append(
                    WheelMiniNode(
                        corrected_name, wheel_size_mm=wheel_size, parent_node=self
                    )
                )

        self.pub_tip = self.create_publisher(Transform, f"tip_pos", 10)
        self.joint_setPUB = self.create_publisher(JointState, "joint_set", 10)
        #    /\    #
        #   /  \   #
        # ^ Publishers ^

        # V Subscribers V
        #   \  /   #
        #    \/    #
        self.sub_rel_target = self.create_subscription(
            Transform, f"set_ik_target", self.set_ik_CBK, 10
        )
        self.roll_sub = self.create_subscription(Float64, f"roll", self.roll_CBK, 10)
        self.joint_readSUB = self.create_subscription(
            JointState,
            "joint_read",
            self.joint_readSUBCBK,
            10,
        )
        #    /\    #
        #   /  \   #
        # ^ Subscribers ^

        # V Service V
        #   \  /   #
        #    \/    #
        self.iAmAlive: Optional[Service] = None
        #    /\    #
        #   /  \   #
        # ^ Service ^

        # V Timers V
        #   \  /   #
        #    \/    #
        self.forwardKinemticsTimer = self.create_timer(0.1, self.publish_tip_pos)
        self.forwardKinemticsTimer.cancel()  # this timer executes 0.01 after every new angle received
        self.lastTimeIK: Time = self.get_clock().now()
        self.FisrtTS: Optional[Time] = None
        self.firstSpin: Timer = self.create_timer(1 / 5, self.firstSpinCBK)
        #    /\    #
        #   /  \   #
        # ^ Timers ^

    @error_catcher
    def firstSpinCBK(self):
        # bug = 1/0
        if self.FisrtTS is None:
            self.FisrtTS = self.get_clock().now()
        areUnknownAngle = np.any(np.isnan(self.angleReadings))
        if areUnknownAngle:
            sinceLaunch: Duration = self.get_clock().now() - self.FisrtTS
            if sinceLaunch > self.WAIT_ANGLE_MES:
                self.get_logger().warn("Waiting for angle data", once=True)
            if sinceLaunch > self.WAIT_ANGLE_ABORT:
                self.get_logger().warn("Waited too long, angles assumed zero", once=True)
                self.angleReadings[:] = 0.0
                self.last_sent[:] = self.angleReadings
            return
        self.iAmAlive = self.create_service(Empty, f"ik_alive", lambda req, res: res)

        # self.pwarn(self.current_fk())
        x, q = self.current_fk()
        self.pinfo(x)
        self.pinfo(q)
        # x += np.array([10, 0, 0])
        # q = qt.as_float_array(q)
        # q = q / np.linalg.norm(q)
        # q = qt.from_float_array(q)
        # result = self.find_next_ik(x / 1000, q)

        self.destroy_timer(self.firstSpin)

    def all_limits(self, et_chain: ETS, jobjL: List[Joint]):
        new_chain: List[ET] = []
        li = 0
        for j in et_chain:
            if j.isjoint:
                jobj = jobjL[li]
                li += 1
                self.perror(
                    f"{jobj.name}, {jobj.limit.lower}, {jobj.limit.upper}, {j._qlim}"
                )
        return ETS(et_chain)

    @error_catcher
    def roll_CBK(self, msg: Union[Float64, float]) -> None:
        if isinstance(msg, Float64):
            distance = msg.data
        else:
            distance = float(msg)
        # self.pinfo(distance)
        for wheel in self.wheelList:
            wheel.roll(distance)

    def compute_raw_ik(
        self,
        xyz: NDArray,
        quat: qt.quaternion,
        start: NDArray,
        compute_budget: Optional[Duration] = None,  #  type: ignore
        mvt_duration: Optional[Duration] = None,  #  type: ignore
    ) -> Tuple[Optional[NDArray], bool]:

        computeBudget: Duration
        deltaTime: Duration
        if mvt_duration is None:
            deltaTime = Duration(seconds=1 / self.REFRESH_RATE)  # type: ignore
        else:
            deltaTime = mvt_duration
        if compute_budget is None:
            computeBudget = Duration(seconds=1 / self.REFRESH_RATE)
        else:
            computeBudget = compute_budget
        finish_by: Time = self.getNow() + computeBudget

        motion: SE3 = SE3(xyz)  # type: ignore
        motion.A[:3, :3] = qt.as_rotation_matrix(quat)  # type: ignore
        # motion: SE3 = SE3(xyz) * SE3(qt.as_rotation_matrix(quat))
        # motion: SE3 = SE3(qt.as_rotation_matrix(quat)) * SE3(xyz)
        # self.pwarn(motion)
        # self.pwarn(SE3(qt.as_rotation_matrix(quat)))
        if self.angleReadings.shape[0] > 5:
            mask = np.array([1, 1, 1, 1, 1, 1], dtype=float)
        else:
            mask = np.array([1, 1, 1, 0, 0, 0], dtype=float)

        angles: NDArray = self.angleReadings.copy()
        np.nan_to_num(x=angles, nan=0.0, copy=False)
        np.nan_to_num(x=start, nan=0.0, copy=False)
        # self.pinfo(f"start: {start}")
        # for trial in range(4):
        trial = -1
        trialLimit = 20
        bestSolution: Optional[NDArray] = None
        velMaybe: float = 1000000
        validSolFound = False
        compBudgetExceeded = lambda: self.getNow() > finish_by
        # compBudgetExceeded = lambda: False
        while trial < trialLimit and not compBudgetExceeded():
            # self.pinfo(f"trial {trial}")
            trial += 1
            startingPose = start.copy()

            if trial == 0:
                i = 10
                s = 1
            if trial == 1:
                i = 50
                s = 1
            else:
                i = 50
                s = 1_000
                # s = 100

                stpose = np.empty((s, startingPose.shape[0]), float)
                stpose[:, :] = startingPose.reshape(1, -1)
                r = np.random.rand(stpose.shape[0], stpose.shape[1])
                r = r * 2 - 1
                maxi = 1 / 100
                mini = maxi / 100
                r = r * np.linspace(mini, maxi, s, endpoint=True).reshape(-1, 1)
                startingPose = stpose + r
                # self.pwarn(startingPose)

            ik_result = self.subModel.ik_LM(
                # ik_result = self.subModel.ik_NR(
                Tep=motion,
                q0=startingPose,
                mask=mask,
                ilimit=i,
                slimit=s,
                joint_limits=not self.IGNORE_LIM,
                # pinv=True,
                # pinv_damping=0.2,
                tol=1e-6,
            )
            # self.pwarn(not self.IGNORE_LIM)

            solFound = ik_result[1]

            sol = np.array(ik_result[0], dtype=float)
            sol_im = 1 * np.exp(1j * sol)
            star = 1 * np.exp(1j * start)
            delt = sol_im / star
            real_delta = np.angle(delt)
            real_angles = start + real_delta
            # self.pinfo(sol)
            # self.pwarn(real_angles)
            for ind, a in enumerate(real_angles):
                l = self.joints_objects[ind].limit
                if l is None:
                    continue
                lup = l.upper
                llo = l.lower
                # if limit exceeded we go back to the original solution
                if lup is not None and not self.IGNORE_LIM:
                    if real_angles[ind] > lup:
                        real_angles[ind] = sol[ind]
                if llo is not None and not self.IGNORE_LIM:
                    if real_angles[ind] < llo:
                        real_angles[ind] = sol[ind]

            # self.pwarn(real_angles)
            delta = real_angles - start
            # dist = float(np.linalg.norm(delta, ord=np.inf))
            dist = float(np.linalg.norm(delta, ord=3))
            velocity: float = dist / rosTime2Float(deltaTime)

            if solFound:
                if abs(velocity) < abs(IK_MAX_VEL):
                    angles = real_angles
                    validSolFound = True
                    velMaybe = velocity
                    bestSolution = real_angles
                    break
                isBetter = velocity < velMaybe
                if isBetter:
                    bestSolution = real_angles
                    velMaybe = velocity

        if compBudgetExceeded():
            self.pwarn("IK slow, compute terminated", force=True)

        return bestSolution, validSolFound

    def find_next_ik(
        self,
        xyz: NDArray,
        quat: qt.quaternion,
        compute_budget: Optional[Union[Duration, EZRate]] = None,  #  type: ignore
        mvt_duration: Optional[Duration] = None,  #  type: ignore
    ) -> NDArray:

        arriveTime: Time = self.getNow()
        deltaTime: Duration = arriveTime - self.lastTimeIK
        self.lastTimeIK = arriveTime
        ikIsRecent = deltaTime < self.RESET_LAST_SENT
        if ikIsRecent:
            start: NDArray = self.last_sent.copy()
        else:
            start: NDArray = self.angleReadings.copy()

        assert start.shape == self.angleReadings.shape

        bestSolution, validSolutionFound = self.compute_raw_ik(
            xyz,
            quat,
            start,
            compute_budget=Duration(seconds=1 / self.REFRESH_RATE),  #  type: ignore
            mvt_duration=Duration(seconds=1 / self.REFRESH_RATE),  #  type: ignore
        )

        if bestSolution is None:
            self.pwarn("""no IK found :C""", force=True)
            return start

        if not validSolutionFound:
            pass
            self.pwarn("no continuous IK found :C", force=True)

        return bestSolution

    @error_catcher
    def set_ik_CBK(self, msg: Transform) -> None:
        """
        recieves target from leg, converts to numpy, computes IK, sends angle
        results to joints

        Args:
            msg: target as Ros2 Vector3
        """
        xyz, quat = tf2np(msg)

        # self.pwarn(f"x{xyz}, q{qt.as_float_array(quat)}")
        xyz, quat = self.replace_none_target(xyz, quat)
        # self.pinfo(f"x{xyz}, q{qt.as_float_array(quat)}")

        xyz /= 1_000  # from mm to m

        angles = self.find_next_ik(
            xyz,
            quat,
            compute_budget=Duration(seconds=self.REFRESH_RATE),  #  type: ignore
            mvt_duration=Duration(seconds=self.REFRESH_RATE),  #  type: ignore
        )

        self.send_command(angles)
        return

    def replace_none_target(
        self, xyz: NDArray, quat: qt.quaternion
    ) -> Tuple[NDArray, qt.quaternion]:
        fk = None
        if np.any(np.isnan(xyz)):
            if fk is None:
                fk = self.current_fk()
            xyz = fk[0]
        if np.any(np.isnan(qt.as_float_array(quat))):
            if fk is None:
                fk = self.current_fk()
            quat = fk[1]
        return xyz, quat

    @error_catcher
    def joint_readSUBCBK(self, js: JointState):
        if len(js.position) < 1:
            return
        di = dict(zip(js.name, js.position))
        names = set(self.joint_names) & set(di.keys())
        for name in names:
            ind = self.joint_names.index(name)
            self.angleReadings[ind] = di[name]

        if self.forwardKinemticsTimer.is_canceled():
            self.forwardKinemticsTimer.reset()

    def send_command(self, angles: NDArray):
        assert self.last_sent.shape == angles.shape
        assert angles.dtype in [float, np.float32]

        self.last_sent: NDArray = angles.copy()
        msg = JointState(name=self.joint_names, position=list(angles))
        msg.header.stamp = self.getNow().to_msg()
        self.joint_setPUB.publish(msg)
        return

    def current_fk(self) -> Tuple[NDArray, qt.quaternion]:
        fw_result: List[SE3] = self.subModel.fkine(self.angleReadings)  # type: ignore
        rot_matrix = np.array(fw_result[-1].R, dtype=float)
        tip_coord: NDArray = fw_result[-1].t * 1000
        tip_quat: qt.quaternion = qt.from_rotation_matrix(rot_matrix)
        return tip_coord, tip_quat

    @error_catcher
    def publish_tip_pos(self) -> None:
        """
        Computes foward kinematics given angles stored in array,
        publishes tip position result.
        This is executed x ms after an angle reading is received
        """
        nanIsHere: bool = bool(np.any(np.isnan(self.angleReadings)))
        if nanIsHere:
            return
        msg = Vector3()
        tip_coord, tip_quat = self.current_fk()
        # self.pwarn(np.round(tip_coord))
        # self.pinfo(np.round(rot_matrix, 2))
        msg = np2tf(coord=tip_coord, quat=tip_quat)
        self.pub_tip.publish(msg)
        self.forwardKinemticsTimer.cancel()

        # chain = self.ETchain.copy()
        # for i in range(self.ETchain.m):
        #     fw_result: List[SE3] = chain.fkine(q=self.joints_angle_arr)
        #     self.pwarn(np.round(fw_result[0].t, decimals=3))
        #     chain.pop()


def main():
    myMain(IKNode, multiThreaded=False)


if __name__ == "__main__":
    main()
