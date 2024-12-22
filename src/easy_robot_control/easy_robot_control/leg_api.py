"""
Provides api anc controllers to control leg ik and joints directly

Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

from copy import deepcopy
from dataclasses import dataclass
from typing import (Any, Dict, Final, Literal, Optional, Sequence, Tuple,
                    TypeVar, overload)

import nptyping as nt
import numpy as np
from geometry_msgs.msg import Transform
from motion_stack_msgs.srv import ReturnJointState, TFService
from nptyping import NDArray, Shape
from rclpy.node import List, Union
from rclpy.publisher import Publisher
from rclpy.task import Future
from rclpy.time import Duration, Time
from std_srvs.srv import Empty

from easy_robot_control.EliaNode import (Client, EliaNode, error_catcher,
                                         get_src_folder, np2tf, rosTime2Float,
                                         tf2np)
from easy_robot_control.utils.hyper_sphere_clamp import (clamp_to_sqewed_hs,
                                                         clamp_xyz_quat)
from easy_robot_control.utils.joint_state_util import (JointState, JState,
                                                       js_from_ros,
                                                       stateOrderinator3000)
from easy_robot_control.utils.math import Quaternion, qt, qt_repr

float_formatter = "{:.1f}".format
np.set_printoptions(formatter={"float_kind": float_formatter})

AvailableMvt = Literal["shift", "transl", "rot", "hop"]
Farr = NDArray[Any, nt.Float]
Barr = NDArray[Any, nt.Bool]
Flo3 = NDArray[Shape["3"], nt.Floating]
Flo4 = NDArray[Shape["4"], nt.Floating]

ALLWOED_DELTA_JOINT = np.deg2rad(7)  # for joint motor control

# ik2 commands cannot be further than ALLOWED_DELTA_XYZ | ALLOWED_DELTA_QUAT away
# from the current tip pose
ALLOWED_DELTA_XYZ = 50  # mm ;
ALLOWED_DELTA_QUAT = np.deg2rad(5)  # rad ; same but for rotation


@dataclass()
class Pose:
    time: Time
    xyz: Flo3
    quat: Quaternion

    def __deepcopy__(self, memo):
        return Pose(time=self.time, xyz=(self.xyz.copy()), quat=(self.quat.copy()))

    def __sub__(self, other: "Pose") -> "Pose":
        return Pose(
            time=self.time - other.time,
            xyz=self.xyz - other.xyz,
            quat=self.quat / other.quat,
        )

    def close2zero(self, atol=(1, 0.01)) -> bool:
        a = np.allclose(self.xyz, 0, atol=atol[0])
        b = qt.allclose(self.quat, qt.one, atol=atol[1])
        return bool(a and b)


MVT2SRV: Final[Dict[AvailableMvt, str]] = {
    "shift": "shift",
    "transl": "rel_transl",
    "rot": "rot",
    "hop": "rel_hop",
}


class JointMini:
    def __init__(self, joint_name: str, prefix: str, parent_leg: "Leg"):
        self.name = joint_name
        self.leg = parent_leg
        self.__node = parent_leg.parent
        self.state = JState(name=self.name)
        self.prefix = prefix
        self._speed_target: Optional[float] = None
        self.__speed_set_time: Optional[Time] = None
        self.__speed_set_angle: Optional[float] = None
        self.__speed_set_angle_save: Optional[float] = None
        self._speedTMR = self.__node.create_timer(0.05, self.__speedTMRCBK)
        self._speedTMR.cancel()
        self.max_delta = ALLWOED_DELTA_JOINT

    def is_active(self):
        if self.state.time is None:
            return False
        return rosTime2Float(self.__node.getNow() - self.state.time) < 5

    def self_report(self):
        header = f"{self.name}, "
        if self.state.time is None:
            return header + "NO sensor data"
        if not self.is_active():
            return header + "OLD sensor data"
        return ""

    @property
    def effort(self) -> Optional[float]:
        return self.state.effort

    @effort.setter
    def effort(self, value: Optional[float]):
        self.state.time = self.__node.getNow()
        self.state.effort = value

    @property
    def speed(self) -> Optional[float]:
        return self.state.velocity

    @speed.setter
    def speed(self, value: Optional[float]):
        self.state.time = self.__node.getNow()
        self.state.velocity = value

    @property
    def angle(self) -> Optional[float]:
        return self.state.position

    @angle.setter
    def angle(self, value: Optional[float]):
        self.state.time = self.__node.getNow()
        self.state.position = value

    def apply_speed_target(self, speed: Optional[float]) -> None:
        """Moves the joint at a given speed using position command.
        It sends everincreasing angle target
        (this is for safety, so it stops when nothing happens).

        The next angle target sent cannot be more than MAX_DELTA radian away
        from the current sensor angle
        (safe, because you can only do small movements with this method)
        If it is too far away, the speed value will decrease
        to match the max speed of the joint (not exactly, it will be a little higher).

        Args:
            speed: approx speed value (max speed if for 1) or None
        """
        if speed == 0:
            speed = None
        if self._speed_target == speed:
            return
        self._speed_target = speed
        self.__speed_set_angle = self.angle
        self.__speed_set_angle_save = self.__speed_set_angle
        self.__speed_set_time = self.__node.getNow()
        # compensate for starting late
        self.__speed_set_time -= Duration(nanoseconds=self._speedTMR.timer_period_ns)
        if speed is None:
            self.__speed_tmr_off()
            self.__node.execute_in_cbk_group(self.__speedTMRCBK)
        else:
            self.__speed_tmr_on()

    def __speed_tmr_on(self):
        """starts to publish angles based on speed"""
        # return
        if self._speedTMR.is_canceled():
            # avoids a ros2 bug I think
            self.__node.execute_in_cbk_group(self._speedTMR.reset)

    def __speed_tmr_off(self):
        """starts to publish angles based on speed"""
        return
        if not self._speedTMR.is_canceled():
            # avoids a ros2 bug I think
            self.__node.execute_in_cbk_group(self._speedTMR.cancel)

    @error_catcher
    def __speedTMRCBK(self):
        """Updates angle based on stored speed. Stops if speed is None"""
        if (
            self._speed_target is None
            or self.angle is None
            or self.__speed_set_angle is None
            or self.__speed_set_angle_save is None
            or self.__speed_set_time is None
        ):
            return
        now = self.__node.getNow()
        delta_time = rosTime2Float(now - self.__speed_set_time)
        delta_ang = self._speed_target * delta_time
        integrated_angle = self.__speed_set_angle + delta_ang

        upper_bound = self.angle + self.max_delta
        lower_bound = self.angle - self.max_delta

        if not (lower_bound < integrated_angle < upper_bound):
            clipped = np.clip(integrated_angle, lower_bound, upper_bound)
            real_speed = (self.angle - self.__speed_set_angle_save) / (delta_time)
            if abs(real_speed) < 0.0001:
                real_speed = 0.0002
            if delta_time > 1 and abs(self._speed_target / real_speed) > 1.2:
                # will slowly converge towward real speed*1.05
                self._speed_target = self._speed_target * 0.9 + (real_speed * 1.05) * 0.1
                self.__node.pwarn(f"Speed fast, reduced to {self._speed_target:.3f}")
            else:
                pass
            self.__speed_set_angle = clipped - self._speed_target * delta_time
            integrated_angle = clipped

        self.__publish_angle_cmd(integrated_angle)

    def apply_angle_target(self, angle: float) -> None:
        """Sets angle target for the joint, and cancels speed command

        Args:
            angle: angle target
        """
        self.apply_speed_target(0)
        self.__publish_angle_cmd(angle)

    def __publish_angle_cmd(self, angle: Optional[float]):
        js = JState(name=self.name, time=self.__node.getNow(), position=angle)
        self.__publish_cmd(js)

    def __publish_cmd(self, js: JState):
        self.leg._send_joint_cmd([js])


T = TypeVar("T", NDArray, Quaternion)


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

        self._ikSUB = self.parent.create_subscription(
            Transform, f"leg{number}/tip_pos", self.__ikSUBCBK, 10
        )
        self.xyz_now: Optional[NDArray] = None
        self.quat_now: Optional[Quaternion] = None
        self._jointPUB = self.parent.create_publisher(
            JointState, f"leg{number}/joint_set", 10
        )
        self._ikPUB = self.parent.create_publisher(
            Transform, f"leg{number}/set_ik_target", 10
        )
        self._jointSUB = self.parent.create_subscription(
            JointState, f"leg{number}/joint_read", self._listen_jointsCBK, 10
        )

        self._mvt_clients: Dict[AvailableMvt, Client] = {}
        self.joint_name_list: Sequence[str] = []
        self.joints: Dict[str, JointMini] = {}
        self._joint_pub: Sequence[Publisher] = []
        self._joint_getterCLI = self.parent.create_client(
            ReturnJointState, f"leg{self.number}/advertise_joints"
        )
        self.connect_movement_clients()
        self.look_for_joints()

        self.ik2 = Ik2(self)

    def connect_movement_clients(self):
        # return
        for mvt, srv in MVT2SRV.items():
            self._mvt_clients[mvt] = self.parent.get_and_wait_Client(
                f"leg{self.number}/{srv}", TFService
            )

    @staticmethod
    def do_i_exist(number: int, parent: EliaNode, timeout: float = 1):
        """Slow do not use to spam scan.
        Returns True if the leg is alive"""
        cli = parent.create_client(srv_type=Empty, srv_name=f"leg{number}/joint_alive")
        is_alive = cli.wait_for_service(timeout_sec=timeout)
        parent.destroy_client(cli)
        return is_alive

    def self_report(self) -> str:
        msg = ""
        if self.xyz_now is None or self.quat_now is None:
            msg += "Issue [IK]: No end effector data received\n"
        for j in self.joints.values():
            if j.self_report() != "":
                msg += f"Issue [J]: {j.self_report()}\n"
        msg_head = f"Leg {self.number} report: "
        if msg == "":
            return msg_head + "no issues :)"
        else:
            return msg_head + "\n" + msg

    def _listen_jointsCBK(self, msg: JointState):
        states = js_from_ros(msg)
        self._update_joints_sensor(states)

    def _update_joints_sensor(self, states: List[JState]):
        for state in states:
            j = self.joints.get(state.name)
            if j is None:
                continue
            j.angle = state.position

    def _send_joint_cmd(self, states: List[JState]):
        msgs = stateOrderinator3000(states)
        for msg in msgs:
            self._jointPUB.publish(msg)

    def __ikSUBCBK(self, msg: Transform):
        """recieves tip position form ik lvl2"""
        self.xyz_now, self.quat_now = tf2np(msg)

    def add_joints(self, all_joints: List[str]) -> List[JointMini]:
        old_joint = self.joints.keys()
        new_joints = set(all_joints) - set(old_joint)
        for n in new_joints:
            self.joints[n] = JointMini(n, f"leg{self.number}/", self)
        self.joint_name_list = sorted(list(self.joints.keys()))
        return [self.joints[n] for n in new_joints]

    def look_for_joints(self):
        """scans and updates the list of joints of this leg"""
        fut = self._joint_getterCLI.call_async(ReturnJointState.Request())

        def next_step(msg: Future):
            if msg.result() is None:
                return
            res: JointState = msg.result().js
            all_joints = list(res.name)
            self.add_joints(all_joints)

        fut.add_done_callback(next_step)

    def go2zero(self):
        """sends angle target of 0 on all joints"""
        for j in self.joint_name_list:
            self.set_angle(angle=0, joint=j)

    def get_joint_obj(self, joint: Union[int, str]) -> Optional[JointMini]:
        """Gets the corresponding joint object is exists

        Args:
            joint: joint name or number (alphabetically ordered)
        """
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
        joint_obj.apply_angle_target(angle)
        return True

    def ik(
        self,
        xyz: Union[None, NDArray, Sequence[float]] = None,
        quat: Optional[Quaternion] = None,
    ) -> None:
        """Publishes an ik target for the leg () relative to baselink. Motion stack lvl2

        Args:
            xyz:
            quat:
        """
        # self.parent.pinfo(f"{xyz} --- {quat}")
        msg = np2tf(coord=xyz, quat=quat, sendNone=True)
        self._ikPUB.publish(msg)
        return

    @overload
    def move(
        self,
        xyz: Union[None, NDArray, Sequence[float]] = None,
        quat: Optional[Quaternion] = None,
        mvt_type: AvailableMvt = "shift",
        blocking: Literal[True] = True,
    ) -> Future: ...

    @overload
    def move(
        self,
        xyz: Union[None, NDArray, Sequence[float]] = None,
        quat: Optional[Quaternion] = None,
        mvt_type: AvailableMvt = "shift",
        blocking: Literal[False] = False,
    ) -> TFService.Response: ...

    def move(
        self,
        xyz: Union[None, NDArray, Sequence[float]] = None,
        quat: Optional[Quaternion] = None,
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
        shiftCMD = self._mvt_clients.get(mvt_type)
        if shiftCMD is None:
            self.parent.pwarn(f"Client for {mvt_type} not connected")
            cancelled = Future()
            cancelled.cancel()
            return cancelled
        if blocking:
            call = shiftCMD.call(request)
        else:
            call = shiftCMD.call_async(request)
        return call


class Ik2:
    """Provides ik2 methods given a leg"""

    def __init__(self, leg: Leg) -> None:
        self.parent = leg.parent
        self.leg = leg
        self.last_pose: Optional[Pose]
        self.clear()

        self.task_future: Future = Future()
        self.task_future.set_result(True)
        self.task = lambda: None

        # ik2 next step will be inside of this sphere (centered on the current EE pos)
        self.sphere_xyz_radius: float = ALLOWED_DELTA_XYZ  # mm
        self.sphere_quat_radius: float = ALLOWED_DELTA_QUAT  # rad

        self.task_executor = self.parent.create_timer(0.1, self.run_task)
        # self.recording = np.empty(shape=(1+7+7+7))
        # self.rec_start = self.parent.getNow()
        # tmr = self.parent.create_timer(2, self.save_recording)

    # @error_catcher
    # def save_recording(self):
        # np.save(f"{get_src_folder('easy_robot_control')}/recording.npy", self.recording)

    @error_catcher
    def run_task(self):
        self.task()
        if self.task_future.done():
            # del self.task
            self.task = lambda: None
            return

    @property
    def quat_now(self) -> Optional[Quaternion]:
        return self.leg.quat_now

    @property
    def xyz_now(self) -> Optional[Flo3]:
        return self.leg.xyz_now

    @property
    def now_pose(self) -> Optional[Pose]:
        x = self.xyz_now
        q = self.quat_now
        if q is None or x is None:
            return None
        return Pose(time=self.parent.getNow(), xyz=x, quat=q)

    def reset(self) -> Pose:
        """Resets the trajectory start point onto the current end effector position"""
        s = self.now_pose
        assert s is not None, "reset impossible, no tip pos data available"
        self.last_pose = deepcopy(s)
        return self.last_pose

    def clear(self):
        """Clears the trajectory start point"""
        self.last_pose = None

    def _previous_point(self) -> Pose:
        """Last point executed from the trajectory and sent to ik"""
        if self.last_pose is None:
            return self.reset()
        else:
            return self.last_pose

    def make_abs_pos(
        self,
        xyz: Optional[Flo3],
        quat: Optional[Quaternion],
        ee_relative: Optional[bool] = False,
    ) -> Optional[Pose]:
        now = self.now_pose
        if now is None:
            self.parent.pwarn(f"[leg#{self.leg.number}] tip_pos UNKNOWN, ik2 ignored")
            return None

        previous = self._previous_point()
        if ee_relative:
            if xyz is None:
                xyz = np.zeros_like(previous.xyz)
            if quat is None:
                quat = qt.one
            xyz += previous.xyz
            quat = previous.quat * quat
        else:
            if xyz is None:
                xyz = previous.xyz
            if quat is None:
                quat = previous.quat

        target = Pose(time=self.parent.getNow(), xyz=xyz, quat=quat)
        return target

    def controlled_motion(
        self,
        xyz: Optional[Flo3],
        quat: Optional[Quaternion],
        ee_relative: Optional[bool] = False,
    ) -> Future:
        """Continuously calls self.step_toward(...) in the task timer.

        Cancels the previous task and future.
        Replaces it with a new function to execute and new future.
        Returns the assiciated future (you can cancel it, and know when it's done).

        Args:
            xyz: target in mm
            quat: target as quaternion
            ee_relative: if the movement should bee performed relative to the end effector

        Returns
            Future assiciated with the movement's task.
        """
        self.task_future.cancel()

        pose = self.make_abs_pos(xyz, quat, ee_relative)
        future = Future()
        if pose is None:
            future.cancel()
            return future

        def step_toward_target():
            if future.cancelled():
                self.parent.pinfo("cancelled moving")
                return
            if future.done():
                self.parent.pwarn("ruh ho")
                return

            self.parent.pinfo("moving")
            move_done = self.step_toward(xyz=pose.xyz, quat=pose.quat, ee_relative=False)

            if move_done:
                self.parent.pinfo("target reached")
                future.set_result(move_done)

        self.task = step_toward_target
        self.task_future = future
        future.add_done_callback(lambda *_: self.parent.pinfo("movement terminated"))
        return future

    def step_toward(
        self,
        xyz: Optional[Flo3],
        quat: Optional[Quaternion],
        ee_relative: Optional[bool] = False,
    ) -> bool:
        """Sends a single ik command to move toward the target.
        Not meant to reach the target!!!

        This method is robust to IK errors and motor speed saturation. It will adapt its
        speed according to the robot response to keep track with the path.

        The command will clamp not farther from the current EE than self.sphere_xyz_radius
        and self.sphere_quat_radius.
        Increase those values to allow for a wider margin of error
        (also leading to higher speed)

        The math become imprecise with big deltas in quaternions. See clamp_xyz_quat(...)
        Do not use if self.last_pose.quat is opposite to quat.
        (idk what will happen but you wont like it)

        Args:
            xyz: target in mm
            quat: target as quaternion
            ee_relative: if the movement should bee performed relative to the end effector
        """
        now = self.now_pose
        if now is None:
            self.parent.pwarn(f"[leg#{self.leg.number}] tip_pos UNKNOWN, ik2 ignored")
            return False

        previous = self._previous_point()
        if ee_relative:
            if xyz is None:
                xyz = np.zeros_like(previous.xyz)
            if quat is None:
                quat = qt.one
            xyz += previous.xyz
            quat = previous.quat * quat
        else:
            if xyz is None:
                xyz = previous.xyz
            if quat is None:
                quat = previous.quat

        target = Pose(time=self.parent.getNow(), xyz=xyz, quat=quat)

        now = deepcopy(now)
        previous = deepcopy(previous)
        target = deepcopy(target)

        x, q = clamp_xyz_quat(
            center=(now.xyz, now.quat),
            start=(previous.xyz, previous.quat),
            end=(target.xyz, target.quat),
            radii=(self.sphere_xyz_radius, self.sphere_quat_radius),
        )

        clamped = Pose(time=target.time, xyz=x, quat=q)

        self.leg.ik(
            xyz=clamped.xyz,
            quat=clamped.quat,
        )
        self.last_pose = clamped

        res = (clamped - target).close2zero()

        # new_data = np.empty((1+7+7+7))
        # new_data[0] = rosTime2Float(self.parent.getNow()-self.rec_start)
        # for i,k in enumerate([now, target, clamped]):
        #     off = i * 7 + 1
        #     new_data[off:off + 3] = k.xyz
        #     new_data[off + 3:off + 7] = qt.as_float_array(k.quat)
        # self.recording = np.vstack((self.recording, new_data.reshape(1, -1)))

        return res

    def offset(
        self,
        xyz: Optional[Flo3],
        quat: Optional[Quaternion],
        ee_relative: Optional[bool] = False,
    ):
        """Wrapper around self.step_toward(...),
        Origin is placed onto the EE.
        ee_relative specifies if the origin should have the same orientation as the EE

        Args:
            xyz: mm to move by
            quat: quaternion to move by
            ee_relative:
                True: movement origin is the EE.
                False: movement origin is the baselink.
        """
        if ee_relative:
            self.step_toward(xyz, quat, ee_relative)

        now = self.now_pose
        if now is None:
            self.parent.pwarn(f"[leg#{self.leg.number}] tip_pos UNKNOWN, offset ignored")
            return
        previous = self._previous_point()

        if xyz is None:
            xyz = np.zeros_like(previous.xyz)
        if quat is None:
            quat = qt.one
        self.step_toward(previous.xyz + xyz, quat * previous.quat, ee_relative)
