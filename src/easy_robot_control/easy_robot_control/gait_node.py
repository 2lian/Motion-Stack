"""
This node is responsible for chosing targets and positions.

Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

from typing import Dict, Final, Literal, Optional, overload
import numpy as np
from numpy.typing import NDArray
import quaternion as qt
import rclpy
from rclpy.task import Future
from rclpy.node import Union, List
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.publisher import Publisher
from EliaNode import (
    Client,
    EliaNode,
    error_catcher,
    np2TargetSet,
    np2tf,
    myMain,
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


MVT2TOPIC: Final[Dict[str, str]] = {
    "shift": "shift",
    "transl": "rel_transl",
    "rot": "rot",
    "hop": "rel_hop",
}


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
        self.declare_parameter("number_of_legs", 4)
        self.NUMBER_OF_LEG = (
            self.get_parameter("number_of_legs").get_parameter_value().integer_value
        )
        self.declare_parameter("leg_list", [0])
        self.LEG_LIST: List[int] = (  # type: ignore
            self.get_parameter("leg_list").get_parameter_value().integer_array_value
        )
        #    /\    #
        #   /  \   #
        # ^ Params ^

        # V Publishers V
        #   \  /   #
        #    \/    #
        self.ikPUB = [
            self.create_publisher(Transform, f"leg{k}/set_ik_target", 10)
            for k in self.LEG_LIST
        ]
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

    def hero_dragon(self):
        wheel_j = [
            "/leg13/spe_2wheel_left_joint_set",
            "/leg13/spe_2wheel_right_joint_set",
            "/leg14/spe_1wheel_left_joint_set",
            "/leg14/spe_1wheel_right_joint_set",
        ]
        wheel_spe: List[Publisher] = [
            self.create_publisher(Float64, name, 10) for name in wheel_j
        ]

        def forward(speed: float):
            for p in wheel_spe:
                p.publish(Float64(data=float(speed)))

        main_leg = 0  # default for all moves
        manip_leg = 1

        # quit()
        joint_midPUB = self.create_publisher(
            Float64, "leg3/ang_leg3base_link_link2_set", 10
        )
        joint_last2PUB = self.create_publisher(
            Float64, "leg3/ang_leg3link6_link7_set", 10
        )
        joint_midPUB.publish(Float64(data=-np.pi / 2))
        joint_last2PUB.publish(Float64(data=np.pi))
        # quit()
        self.sleep(0.1)

        rot = qt.from_rotation_vector(np.array([1, 0, 0]) * np.pi / 2)
        rot = qt.from_rotation_vector(np.array([0, 1, 0]) * np.pi / 2) * rot
        self.leg_ik(xyz=[-100, 500, 700], quat=rot, leg_ind=manip_leg)

        self.leg_ik(xyz=[0, -1200, 0], quat=qt.one)
        self.sleep(2)

        for direction in [1, -1]:
            forward(-0.5)
            rot = qt.from_rotation_vector(np.array([0, 0, 0.5 * direction]))
            self.leg_move(quat=rot)
            self.goToTargetBody(bodyQuat=rot)

            forward(0)
            unrot = 1 / rot
            self.goToTargetBody(bodyQuat=unrot)
            origin = qt.from_rotation_vector(np.array([0, 0, 0]))
            self.leg_move(quat=origin, mvt_type="transl")

    def hero_arm(self):
        movement = np.array([100, 0, 0], dtype=float)

        rot_axis = np.array([1, 0, 0], dtype=float)
        rot_axis = rot_axis / np.linalg.norm(rot_axis)
        rot_magnitude = 0
        rot_vec = rot_magnitude * rot_axis
        rotation: qt.quaternion = qt.from_rotation_vector(rot_vec)

        while 1:
            self.goToTargetBody(
                bodyXYZ=movement,
            )
            self.goToTargetBody(
                bodyXYZ=-movement,
            )

            call_list: List[Future] = []
            for ind, leg_num in enumerate(self.LEG_LIST):
                call = self.leg_move(xyz=movement, blocking=False, leg_ind=ind)
                call_list.append(call)
            self.wait_on_futures(call_list)

            call_list: List[Future] = []
            for ind, leg_num in enumerate(self.LEG_LIST):
                call = self.leg_move(xyz=-movement, blocking=False, leg_ind=ind)
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
        msg = np2tf(
            coord=np.array([-600, -300, 800]),
            quat=None,
            sendNone=False,
        )
        self.ikPUB[0].publish(msg)
        self.ikPUB[0].publish(msg)
        self.ikPUB[0].publish(msg)
        self.ikPUB[0].publish(msg)
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

    @overload
    def leg_move(
        self,
        xyz: Union[None, NDArray, List[float]] = None,
        quat: Optional[qt.quaternion] = None,
        leg_ind: int = 0,
        mvt_type: Union[
            Literal["shift"], Literal["transl"], Literal["rot"], Literal["hop"]
        ] = "shift",
        blocking: Literal[True] = True,
    ) -> TFService.Response: ...

    @overload
    def leg_move(
        self,
        xyz: Union[None, NDArray, List[float]] = None,
        quat: Optional[qt.quaternion] = None,
        leg_ind: int = 0,
        mvt_type: Union[
            Literal["shift"], Literal["transl"], Literal["rot"], Literal["hop"]
        ] = "shift",
        blocking: Literal[False] = False,
    ) -> Future: ...

    def leg_move(
        self,
        xyz: Union[None, NDArray, List[float]] = None,
        quat: Optional[qt.quaternion] = None,
        leg_ind: int = 0,
        mvt_type: Union[
            Literal["shift"],
            Literal["transl"],
            Literal["rot"],
            Literal["hop"],
        ] = "shift",
        blocking: bool = True,
    ) -> Union[Future, TFService.Response]:

        if isinstance(xyz, list):
            xyz = np.array(xyz, dtype=float)

        leg_num = self.LEG_LIST[leg_ind]
        request = TFService.Request()
        request.tf = np2tf(
            coord=xyz,
            quat=quat,
            sendNone=True,
        )
        # TODO: better
        shiftCMD = self.get_and_wait_Client(
            f"leg{leg_num}/{MVT2TOPIC[mvt_type]}", TFService
        )
        if blocking:
            call = shiftCMD.call(request)
            self.destroy_client(shiftCMD)
        else:
            call = shiftCMD.call_async(request)
            call.add_done_callback(lambda r: self.destroy_client(shiftCMD))
        return call

    def leg_ik(
        self,
        xyz: Union[None, NDArray, List[float]] = None,
        quat: Optional[qt.quaternion] = None,
        leg_ind: int = 0,
    ):
        if isinstance(xyz, list):
            xyz = np.array(xyz, dtype=float)
        msg = np2tf(coord=xyz, quat=quat, sendNone=True)
        self.ikPUB[leg_ind].publish(msg)
        self.ikPUB[leg_ind].publish(msg)
        return

    def crawlToTargetSet(self, NDArray) -> None: ...

    def goToDefault(self): ...

    def stand(self): ...


def main(args=None):
    myMain(GaitNode, multiThreaded=True)


if __name__ == "__main__":
    main()
