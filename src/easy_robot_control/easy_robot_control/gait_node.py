"""
This node is responsible for chosing targets and positions.

Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

from typing import Dict, Final, Literal, Optional, Sequence, overload
import re
import numpy as np
from numpy.typing import NDArray
import quaternion as qt
import rclpy
from rclpy.clock import Time
from rclpy.task import Future
from rclpy.node import Union, List
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.publisher import Publisher
from std_srvs.srv import Empty
from EliaNode import (
    Client,
    EliaNode,
    error_catcher,
    np2TargetSet,
    np2tf,
    myMain,
    rosTime2Float,
    targetSet2np,
)

from std_msgs.msg import Float64
from geometry_msgs.msg import Transform

from custom_messages.srv import (
    ReturnTargetBody,
    ReturnVect3,
    Vect3,
    TFService,
    ReturnTargetSet,
    SendTargetSet,
    SendTargetBody,
)
from custom_messages.msg import TargetBody, TargetSet

float_formatter = "{:.1f}".format
np.set_printoptions(formatter={"float_kind": float_formatter})

AvailableMvt = Literal["shift", "transl", "rot", "hop"]

MVT2SRV: Final[Dict[AvailableMvt, str]] = {
    "shift": "shift",
    "transl": "rel_transl",
    "rot": "rot",
    "hop": "rel_hop",
}


class JointMini:
    def __init__(self, joint_name: str, prefix: str, parent_node: "EliaNode"):
        self.joint_name = joint_name
        self.parent_node = parent_node
        self.angle: Optional[float] = None
        self.prefix = prefix
        self.parent_node.create_subscription(
            Float64,
            f"{prefix}read_{self.joint_name}",
            self.angle_readCBK,
            10,
        )
        self.to_angle_below = self.parent_node.create_publisher(
            Float64, f"{prefix}ang_{self.joint_name}_set", 10
        )
        self.speed_target: Optional[float] = None
        self.last_apply_speed_call: Time = self.parent_node.getNow()
        self.speedTMR = self.parent_node.create_timer(0.01, self.speedTMRCBK)
        self.MAX_DELTA = 1

    @error_catcher
    def angle_readCBK(self, msg: Float64):
        """recieves angle reading from joint, stores value in array.
        Starts timer to publish new tip position.

        Args:
            msg: Ros2 Float64 - angle reading
        """
        self.angle = msg.data

    def set_speed(self, speed: Optional[float]) -> None:
        """Moves the joint at a "very approximate speed" by increasing the angle.
        speed of 1 (or -1) is max speed of moonbot joint. 
        (real speed it is not linear with this value)
        None or 0 stops the angle update.

        Args:
            speed: approx speed value (max speed if for 1) or None
        """
        if speed == 0:
            speed = None
        if self.speed_target == speed:
            return
        self.speed_target = speed
        if speed is None:
            self.__speed_tmr_off()
        else:
            self.__speed_tmr_on()

    def __speed_tmr_on(self):
        """starts to publish angles based on speed"""
        if self.speedTMR.is_canceled():
            self.parent_node.execute_in_cbk_group(self.speedTMR.reset)
            self.parent_node.execute_in_cbk_group(self.speedTMRCBK)

    def __speed_tmr_off(self):
        """starts to publish angles based on speed"""
        if not self.speedTMR.is_canceled():
            self.parent_node.execute_in_cbk_group(self.speedTMR.cancel)

    @error_catcher
    def speedTMRCBK(self):
        """Updates angle based on stored speed. Stops if speed is None"""
        if self.speed_target == None:
            return
        next_target = self.angle
        if next_target is None:
            return

        now = self.parent_node.getNow()
        dt = rosTime2Float(now - self.last_apply_speed_call)
        delta = self.speed_target * dt
        delta = np.clip(delta, -self.MAX_DELTA, self.MAX_DELTA)
        # self.parent_node.pinfo(delta)
        next_target += self.speed_target * 0.01 * 7
        self.to_angle_below.publish(Float64(data=float(next_target)))
        self.last_apply_speed_call = now

    def set_angle(self, angle: float) -> None:
        """Sets angle target for the joint, and cancels speed command

        Args:
            angle: angle target
        """
        self.set_speed(0)
        out_msg = Float64()
        out_msg.data = float(angle)
        self.to_angle_below.publish(out_msg)


class Leg:
    """Helps you use lvl 1-2-3 of a leg

    Attributes:
        number: leg number
        parent: parent node spinning
        joint_name_list: list of joints belonging to the leg
    """

    def __init__(self, number: int, parent: EliaNode) -> None:
        self.number = number
        self.parent = parent

        self._ikPUB = self.parent.create_publisher(
            Transform, f"leg{number}/set_ik_target", 10
        )

        self._mvt_clients: Dict[AvailableMvt, Client] = {}
        self.joint_name_list: Sequence[str] = []
        self.joints: Dict[str, JointMini] = {}
        self._joint_pub: Sequence[Publisher] = []
        for mvt, srv in MVT2SRV.items():
            self._mvt_clients[mvt] = self.parent.get_and_wait_Client(
                f"leg{self.number}/{srv}", TFService
            )
        self.update_joint_pub()

    @staticmethod
    def do_i_exist(number: int, parent: EliaNode, timeout: float = 0.1):
        """Slow do not use to spam scan.
        Returns True if the leg is alive"""
        cli = parent.create_client(srv_type=Empty, srv_name=f"leg{number}/leg_alive")
        is_alive = cli.wait_for_service(timeout_sec=timeout)
        parent.destroy_client(cli)
        return is_alive

    def update_joint_pub(self):
        """scans and updates the list of joints of this leg"""
        new_joints = self.find_new_joints()
        for n in new_joints:
            self.joints[n] = JointMini(n, f"leg{self.number}/", self.parent)
        self.joint_name_list = sorted(list(self.joints.keys()))

    def go2zero(self):
        """sends angle target of 0 on all joints"""
        for j in self.joint_name_list:
            self.set_angle(angle=0, joint=j)

    def get_joint_obj(self, joint: Union[int, str]) -> Optional[JointMini]:
        if isinstance(joint, int):
            if joint >= len(self.joint_name_list):
                self.parent.perror(
                    f"[leg {self.number} object] " f"index {joint} out of range"
                )
                return None
            jname = self.joint_name_list[joint]
        else:
            if joint not in self.joints.keys():
                self.parent.perror(
                    f"[leg {self.number} object] joint name {joint} not in joint list"
                )
                return None
            jname = joint
        joint_obj = self.joints[jname]
        return joint_obj

    def get_angle(self, joint: Union[int, str]) -> Optional[float]:
        """Gets an angle from a joint

        Args:
            joint: joint name or number (alphabetically ordered)

        Returns:
            last recieved angle as float
        """
        joint_obj = self.get_joint_obj(joint)
        if joint_obj is None:
            return None
        return joint_obj.angle

    def set_angle(self, angle: float, joint: Union[int, str]) -> bool:
        """Sends a angle to a joint

        Args:
            angle: rad
            joint: joint name or number (alphabetically ordered)

        Returns:
            True if message sent
        """
        joint_obj = self.get_joint_obj(joint)
        if joint_obj is None:
            return False
        joint_obj.set_angle(angle)
        return True

    def ik(
        self,
        xyz: Union[None, NDArray, Sequence[float]] = None,
        quat: Optional[qt.quaternion] = None,
    ) -> None:
        """Publishes an ik target for the leg () relative to baselink. Motion stack lvl2

        Args:
            xyz:
            quat:
        """
        self.parent.pwarn(xyz)
        self.parent.pwarn(quat)
        msg = np2tf(coord=xyz, quat=quat, sendNone=True)
        self._ikPUB.publish(msg)
        return

    @overload
    def move(
        self,
        xyz: Union[None, NDArray, Sequence[float]] = None,
        quat: Optional[qt.quaternion] = None,
        mvt_type: AvailableMvt = "shift",
        blocking: Literal[True] = True,
    ) -> Future: ...

    @overload
    def move(
        self,
        xyz: Union[None, NDArray, Sequence[float]] = None,
        quat: Optional[qt.quaternion] = None,
        mvt_type: AvailableMvt = "shift",
        blocking: Literal[False] = False,
    ) -> TFService.Response: ...

    def move(
        self,
        xyz: Union[None, NDArray, Sequence[float]] = None,
        quat: Optional[qt.quaternion] = None,
        mvt_type: AvailableMvt = "shift",
        blocking: bool = True,
    ) -> Union[Future, TFService.Response]:
        """Calls the leg's movement service. Motion stack lvl3

        Args:
            xyz: vector part of the tf
            quat: quat of the tf
            mvt_type: type of movement to call
            blocking: if false returns a Future. Else returns the response

        Returns:

        """
        request = TFService.Request()
        request.tf = np2tf(coord=xyz, quat=quat, sendNone=True)
        shiftCMD = self._mvt_clients[mvt_type]
        # shiftCMD.wait_for_service(0.5) # laggy ???
        if blocking:
            call = shiftCMD.call(request)
        else:
            call = shiftCMD.call_async(request)
        return call

    def find_new_joints(self) -> Sequence[str]:
        """Finds topics that look like a joint

        Returns:
            List of topic name sorted alphabetically
        """
        topics = self.parent.get_topic_names_and_types()
        joint_list: Sequence[str] = []
        for top, typ in topics:
            extracted = re.search(rf"leg{self.number}/ang_(.*?)_set", top)
            if extracted is None:
                continue
            if extracted.group(1) == "":
                continue
            if extracted in self.joint_name_list:
                continue
            joint_list.append(extracted.group(1))
        return sorted(joint_list)


class GaitNode(EliaNode):
    def __init__(self):
        # rclpy.init()
        super().__init__("gait_node")  # type: ignore
        self.Alias = "G"
        self.setAndBlockForNecessaryClients("mover_alive")

        # V Params V
        #   \  /   #
        #    \/    #
        self.declare_parameter("std_movement_time", 2.0)
        self.MVT_TIME = (
            self.get_parameter("std_movement_time").get_parameter_value().double_value
        )
        self.declare_parameter("robot_name", "None")
        self.ROBOT_NAME = (
            self.get_parameter("robot_name").get_parameter_value().string_value
        )
        self.declare_parameter("number_of_legs", 1)
        self.NUMBER_OF_LEG = (
            self.get_parameter("number_of_legs").get_parameter_value().integer_value
        )
        self.declare_parameter("leg_list", [0])
        self.LEG_LIST: Sequence[int] = (  # type: ignore
            self.get_parameter("leg_list").get_parameter_value().integer_array_value
        )
        self.legs: Dict[int, Leg] = {}
        for number in self.LEG_LIST:
            self.legs[number] = Leg(number, self)
        #    /\    #
        #   /  \   #
        # ^ Params ^

        # V Publishers V
        #   \  /   #
        #    \/    #
        #    /\    #
        #   /  \   #
        # ^ Publishers ^

        # V Service client V
        #   \  /   #
        #    \/    #

        self.returnTargetSet: Client = self.get_and_wait_Client(
            "get_targetset", ReturnTargetSet
        )
        self.sendTargetBody: Client = self.get_and_wait_Client(
            "go2_targetbody", SendTargetBody
        )

        #    /\    #
        #   /  \   #
        # ^ Service client ^

        # V Service server V
        #   \  /   #
        #    \/    #

        #    /\    #
        #   /  \   #
        # ^ Service server ^

        self.firstSpin = self.create_timer(
            1, self.firstSpinCBK, callback_group=MutuallyExclusiveCallbackGroup()
        )

    @error_catcher
    def firstSpinCBK(self):
        self.pinfo("go")
        self.destroy_timer(self.firstSpin)
        tsnow = self.getTargetSetBlocking()
        # self.perror(self.ROBOT_NAME)
        if "hero_vehicle" == self.ROBOT_NAME:
            self.hero_vehicle()
        if "hero_dragon" == self.ROBOT_NAME:
            self.hero_dragon()
        if "mglimb_7dof" in self.ROBOT_NAME:
            self.gustavo()
        if "hero_7dof" in self.ROBOT_NAME:
            self.hero_arm()
        if self.ROBOT_NAME == "ur16_3f":
            self.ashutosh()
        return
        # self.randomPosLoop()
        # while 1:
        # self.mZeroBasicSetAndWalk()
        # self.goToTargetBodyBlocking(ts=np.array([[-1100, 0, 460]]))
        shiftcmd = self.get_and_wait_Client("leg_0_rel_transl", TFService)

        while 1:
            self.crawl1Wheel()

    def hero_vehicle(self):
        wheel_j = [
            "/leg13/spe_2wheel_left_joint_set",
            "/leg13/spe_2wheel_right_joint_set",
            "/leg14/spe_1wheel_left_joint_set",
            "/leg14/spe_1wheel_right_joint_set",
        ]
        wheel_spe: Sequence[Publisher] = [
            self.create_publisher(Float64, name, 10) for name in wheel_j
        ]

        def forward(ang_speed: float):
            for p in wheel_spe:
                p.publish(Float64(data=float(ang_speed)))

        main_leg_ind = self.LEG_LIST[0]  # default for all moves
        main_leg = self.legs[main_leg_ind]  # default for all moves

        default_coord = [0, -1000, 0]
        main_leg.ik(xyz=default_coord, quat=qt.one)
        self.sleep(2)

        for direction in [1, -1]:
            forward(-0.5)
            rot = qt.from_rotation_vector(np.array([0, 0, 0.5 * direction]))
            main_leg.move(quat=rot)
            self.goToTargetBody(bodyQuat=rot)

            forward(0)
            unrot = 1 / rot
            self.goToTargetBody(bodyQuat=unrot)
            origin = qt.from_rotation_vector(np.array([0, 0, 0]))
            main_leg.move(xyz=default_coord, quat=origin, mvt_type="transl")

    def hero_dragon(self):
        wheel_j = [
            "/leg13/spe_2wheel_left_joint_set",
            "/leg13/spe_2wheel_right_joint_set",
            "/leg14/spe_1wheel_left_joint_set",
            "/leg14/spe_1wheel_right_joint_set",
        ]
        wheel_s_pub: Sequence[Publisher] = [
            self.create_publisher(Float64, name, 10) for name in wheel_j
        ]

        def forward(ang_speed: float):
            for p in wheel_s_pub:
                p.publish(Float64(data=float(ang_speed)))

        main_leg_ind = self.LEG_LIST[0]  # default for all moves
        manip_leg_ind = self.LEG_LIST[1]
        main_leg = self.legs[main_leg_ind]  # default for all moves
        manip_leg = self.legs[manip_leg_ind]

        # quit()
        manip_leg.set_angle(-np.pi / 2, 0)
        manip_leg.set_angle(np.pi, 5)
        self.sleep(0.1)

        rot = qt.from_rotation_vector(np.array([1, 0, 0]) * np.pi / 2)
        rot = qt.from_rotation_vector(np.array([0, 1, 0]) * np.pi / 2) * rot
        manip_leg.ik(xyz=[-100, 500, 700], quat=rot)

        main_leg.ik(xyz=[0, -1200, 0], quat=qt.one)
        self.sleep(2)

        for direction in [1, -1]:
            forward(-0.5)
            rot = qt.from_rotation_vector(np.array([0, 0, 0.5 * direction]))
            main_leg.move(quat=rot)
            self.goToTargetBody(bodyQuat=rot)

            forward(0)
            unrot = 1 / rot
            self.goToTargetBody(bodyQuat=unrot)
            origin = qt.from_rotation_vector(np.array([0, 0, 0]))
            main_leg.move(quat=origin, mvt_type="transl")

    def hero_arm(self):
        wheel_j = [  # I wanna publish speeds directly to wheel joints
            "/leg11/spe_11wheel_left_joint_set",
            "/leg11/spe_11wheel_right_joint_set",
            "/leg12/spe_12wheel_left_joint_set",
            "/leg12/spe_12wheel_right_joint_set",
            "/leg13/spe_13wheel_left_joint_set",
            "/leg13/spe_13wheel_right_joint_set",
            "/leg14/spe_14wheel_left_joint_set",
            "/leg14/spe_14wheel_right_joint_set",
        ]
        wheel_s_pub: Sequence[Publisher] = [
            self.create_publisher(Float64, name, 10) for name in wheel_j
        ]

        def forward(ang_speed: float):
            for p in wheel_s_pub:
                p.publish(Float64(data=float(ang_speed)))

        tsnow = self.getTargetSetBlocking()  # gets current tip xyz for all legs

        # I send an lvl2 IK call to all legs
        # leg 1 will not move, then leg 2 a bit more, 3 more, 4 will be on end_mov/rot
        call_list: Sequence[Future] = []
        end_mov = np.array([200, 0, -200], dtype=float)
        end_rot = np.pi / 4

        for ind, leg in enumerate(self.legs.values()):
            x = ind / (len(self.legs.values()) - 0)  # x in [0, 1]
            movement = end_mov * x
            rot_axis = np.array([0, 1, 0], dtype=float)
            rot_axis = rot_axis / np.linalg.norm(rot_axis)
            rot_magnitude = end_rot * x
            rot_vec = rot_magnitude * rot_axis
            rotation: qt.quaternion = qt.from_rotation_vector(rot_vec)

            current_xyz = tsnow[ind, :]
            # sorry we have to do that manually
            current_quat = qt.from_rotation_matrix(
                [
                    [0, 0, -1],  # z is oposite to x0
                    [0, 1, 0],  # y is aligned with y0
                    [1, 0, 0],  # x is aligned with z0
                ]
            )
            leg.ik(
                xyz=current_xyz + movement + np.array([-100, 0, 0]),
                quat=current_quat * rotation,
            )

        self.sleep(6)

        movement = np.array([100, 0, 0], dtype=float)

        rot_axis = np.array([1, 0, 0], dtype=float)
        rot_axis = rot_axis / np.linalg.norm(rot_axis)
        rot_magnitude = 0
        rot_vec = rot_magnitude * rot_axis
        rotation: qt.quaternion = qt.from_rotation_vector(rot_vec)

        while 1:
            forward(0.5)
            self.goToTargetBody(
                bodyXYZ=movement,
            )
            forward(0)
            self.goToTargetBody(
                bodyXYZ=-movement,
            )

            forward(-0.5)
            call_list: Sequence[Future] = []
            for leg in self.legs.values():
                call = leg.move(xyz=movement, blocking=False)
                call_list.append(call)
            self.wait_on_futures(call_list)

            forward(0)
            call_list: Sequence[Future] = []
            for leg in self.legs.values():
                call = leg.move(xyz=-movement, blocking=False)
                call_list.append(call)
            self.wait_on_futures(call_list)

    def gustavo(self):
        # get the current end effector position
        your_arm_number = self.LEG_LIST[0]
        get_tipCMD: Client = self.get_and_wait_Client(
            f"leg{your_arm_number}/tip_pos", ReturnVect3
        )
        tip_pos_result: ReturnVect3.Response = get_tipCMD.call(
            ReturnVect3.Request(),
        )  # this blocks until recieved

        tip_pos = np.array(
            [
                tip_pos_result.vector.x,
                tip_pos_result.vector.y,
                tip_pos_result.vector.z,
            ],
            dtype=float,
        )  # get your data

        # let's go to the same postion but y=300, z=0, and reset the ee quaternion
        target = tip_pos.copy()
        target[1] += 500
        target[2] = 0
        request = TFService.Request()
        request.tf = np2tf(
            coord=target,
            quat=qt.one,
            sendNone=True,
        )
        shiftCMD = self.get_and_wait_Client(f"leg{your_arm_number}/rel_transl", TFService)
        shiftCMD.call(request)

        # let's go y+200, x+200, and rotate a little
        movement = np.array([200, 200, 0], dtype=float)

        rot_axis = np.array([0, 1, 0], dtype=float)
        rot_axis = rot_axis / np.linalg.norm(rot_axis)
        rot_magnitude = -np.pi / 2
        rot_vec = rot_magnitude * rot_axis
        rotation: qt.quaternion = qt.from_rotation_vector(rot_vec)

        request = TFService.Request()
        request.tf = np2tf(
            coord=movement,
            quat=rotation,
            sendNone=True,
        )
        shiftCMD = self.get_and_wait_Client("leg0/shift", TFService)
        shiftCMD.call(request)

        # let's go  x-200mm, wait 1 s then y-200
        movement = np.array([-200, 0, 0], dtype=float)
        request = TFService.Request()
        request.tf = np2tf(
            coord=movement,
            quat=None,
            sendNone=True,
        )
        call1: Future = shiftCMD.call_async(request)
        self.sleep(1)

        movement = np.array([0, -200, 0], dtype=float)
        request = TFService.Request()
        request.tf = np2tf(
            coord=movement,
            quat=None,
            sendNone=True,
        )
        call2: Future = shiftCMD.call_async(request)

        self.wait_on_futures([call1, call2])  # will block until all your futures are done
        quit()

    def ashutosh(self, res=None) -> None:

        # get the current end effector position
        get_tipCMD: Client = self.get_and_wait_Client("leg0/tip_pos", ReturnVect3)
        tip_pos_result: ReturnVect3.Response = get_tipCMD.call(
            ReturnVect3.Request(),
        )  # this blocks until recieved

        tip_pos = np.array(
            [
                tip_pos_result.vector.x,
                tip_pos_result.vector.y,
                tip_pos_result.vector.z,
            ],
            dtype=float,
        )  # get your data

        # let's go to the same postion but y=-200, 100m backward, and reset to ee quaternion
        target = tip_pos.copy()
        target[0] += 100
        target[1] = -200
        request = TFService.Request()
        request.tf = np2tf(
            coord=target,
            quat=qt.one,
            sendNone=True,
        )
        shiftCMD = self.get_and_wait_Client("leg0/rel_transl", TFService)
        shiftCMD.call(request)

        # let's go down 200mm and back 400mm, and rotate the ee
        movement = np.array([400, 0, -200], dtype=float)

        rot_axis = np.array([1, 0, 0], dtype=float)
        rot_axis = rot_axis / np.linalg.norm(rot_axis)
        rot_magnitude = -np.pi / 2
        rot_vec = rot_magnitude * rot_axis
        rotation: qt.quaternion = qt.from_rotation_vector(rot_vec)

        request = TFService.Request()
        request.tf = np2tf(
            coord=movement,
            quat=rotation,
            sendNone=True,
        )
        shiftCMD = self.get_and_wait_Client("leg0/shift", TFService)
        shiftCMD.call(request)

        # let's go forward 400mm, wait 1 s then go left 300mm
        movement = np.array([-400, 0, 0], dtype=float)
        request = TFService.Request()
        request.tf = np2tf(
            coord=movement,
            quat=None,
            sendNone=True,
        )
        call1: Future = shiftCMD.call_async(request)
        self.sleep(1)

        movement = np.array([0, -300, 0], dtype=float)
        request = TFService.Request()
        request.tf = np2tf(
            coord=movement,
            quat=None,
            sendNone=True,
        )
        call2: Future = shiftCMD.call_async(request)

        self.wait_on_futures([call1, call2])  # will block until all your futures are done

        # lets go back to the some position, but not by doing a trajectory. Simply
        # executing the IK immediately
        self.sleep(0.3)
        arm = list(self.legs.values())[0]
        arm.ik([-600, -300, 800], qt.one)
        self.sleep(1.5)

        self.ashutosh()

    def shiftCrawlDebbug(self, res=None):
        raise Exception("fix that shit")
        tsnow = self.getTargetSetBlocking()
        mvt = np.array([100, 0, 0], dtype=float)
        ts_next = tsnow + mvt.reshape(1, 3)
        self.goToTargetBodyBlocking(ts=ts_next)
        fut: Future = self.goToTargetBody(bodyXYZ=mvt)
        fut.add_done_callback(self.shiftCrawlDebbug)

    @error_catcher
    def crawl1Wheel(self):
        """for moonbot hero one arm+wheel only"""
        tsnow = self.getTargetSetBlocking()

        shiftcmd = self.get_and_wait_Client("leg0/shift", TFService)
        mvt = np.array([400, 0, 0], dtype=float)
        speed = np.linalg.norm(mvt) / self.MVT_TIME

        rollcmd: Publisher = self.create_publisher(
            Float64,
            f"leg0/smart_roll",
            10,
        )
        rollcmd.publish(Float64(data=speed))

        shiftcmd.call(
            TFService.Request(
                tf=np2tf(
                    coord=mvt,
                )
            )
        )

        rollcmd.publish(Float64(data=0.0))

        self.goToTargetBodyBlocking(bodyXYZ=mvt)

    @error_catcher
    def mZeroBasicSetAndWalk(self):
        """for moonbot 0"""
        height = 220
        width = 250
        targetSet = np.array(
            [
                [width, 0, -height],
                [0, width, -height],
                [-width, 0, -height],
                [0, -width, -height],
            ],
            dtype=float,
        )

        self.sendTargetBody.call(
            SendTargetBody.Request(
                target_body=TargetBody(
                    target_set=np2TargetSet(targetSet),
                    body=np2tf(),
                )
            )
        )
        self.sendTargetBody.call(
            SendTargetBody.Request(
                target_body=TargetBody(
                    # target_set=np2TargetSet(ts),
                    body=np2tf(np.array([-50.0, 0.0, 0.0], None)),
                )
            )
        )

        for l in range(4):
            newTS = np.empty(shape=(4, 3), dtype=float)
            newTS[:, :] = np.nan
            newTS[l, :] = targetSet[l, :]
            self.sendTargetBody.call(
                SendTargetBody.Request(
                    target_body=TargetBody(
                        target_set=np2TargetSet(newTS),
                        body=np2tf(np.array([0.0, 0.0, 0.0], None)),
                    )
                )
            )

    def randomPosLoop(self):
        """for multi legged robots"""
        self.goToTargetBody(bodyXYZ=np.array([-200, 0, 0], dtype=float))
        d = 400
        v = np.random.random(size=(3,)) * 0
        while 1:
            dir = np.random.random(size=(3,))
            vnew = (dir - 0.5) * d
            self.goToTargetBody(bodyXYZ=vnew - v)
            v = vnew

    def getTargetSet(self) -> Future:
        call = self.returnTargetSet.call_async(ReturnTargetSet.Request())
        processFuture = Future()

        @error_catcher
        def processMessage(f: Future) -> None:
            response: Optional[ReturnTargetSet.Response] = f.result()
            assert response is not None
            ts = targetSet2np(response.target_set)
            processFuture.set_result(ts)

        call.add_done_callback(processMessage)
        return processFuture

    def getTargetSetBlocking(self) -> NDArray:
        response: ReturnTargetSet.Response = self.returnTargetSet.call(
            ReturnTargetSet.Request()
        )
        return targetSet2np(response.target_set)

    @overload
    def goToTargetBody(
        self,
        ts: Optional[NDArray] = None,
        bodyXYZ: Optional[NDArray] = None,
        bodyQuat: Optional[qt.quaternion] = None,
        blocking: Literal[False] = False,
    ) -> SendTargetBody.Response: ...

    @overload
    def goToTargetBody(
        self,
        ts: Optional[NDArray] = None,
        bodyXYZ: Optional[NDArray] = None,
        bodyQuat: Optional[qt.quaternion] = None,
        blocking: Literal[True] = True,
    ) -> Future: ...

    def goToTargetBody(
        self,
        ts: Optional[NDArray] = None,
        bodyXYZ: Optional[NDArray] = None,
        bodyQuat: Optional[qt.quaternion] = None,
        blocking: bool = True,
    ) -> Union[Future, SendTargetBody.Response]:
        target = TargetBody(
            target_set=np2TargetSet(ts),
            body=np2tf(bodyXYZ, bodyQuat),
        )
        request = SendTargetBody.Request(target_body=target)

        if blocking:
            call = self.sendTargetBody.call(request)
            return call
        else:
            call = self.sendTargetBody.call_async(request)
            return call

    def crawlToTargetSet(self, NDArray) -> None: ...

    def goToDefault(self): ...

    def stand(self): ...


def main(args=None):
    myMain(GaitNode, multiThreaded=True)


if __name__ == "__main__":
    main()
