"""
I this adds functionalities to the default ros2 Node object

Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

import time
from typing import Any, Callable, Optional, Tuple
from numpy.linalg import qr
from numpy.typing import NDArray
from rclpy.guard_condition import GuardCondition

import roboticstoolbox as rtb
import numpy as np
import quaternion as qt
from rclpy.callback_groups import CallbackGroup
from rclpy.client import Client
from rclpy.clock import Clock, ClockType
from rclpy.task import Future
from rclpy.node import Node, List
from rclpy.time import Duration, Time
from geometry_msgs.msg import TransformStamped, Transform
from roboticstoolbox.robot import Robot
from roboticstoolbox.robot.ET import ET
from roboticstoolbox.robot.ETS import ETS
from roboticstoolbox.robot.Link import Link
from roboticstoolbox.tools import URDF
from roboticstoolbox.tools.urdf.urdf import Joint
from std_srvs.srv import Empty


def replace_incompatible_char_ros2(string_to_correct: str) -> str:
    """replace characcter that cannot be used for Ros2 Topics by _

    Args:
        string_to_correct

    Returns:
        corrected_string
    """
    corrected_string = string_to_correct.replace("-", "_")
    corrected_string = corrected_string.replace(" ", "_")
    return corrected_string


def loadAndSet_URDF(
    urdf_path: str, end_effector_name: Optional[str | int] = None
) -> Tuple[Robot, ETS, List[str], List[Joint], Link | None]:
    """I am so sorry. This works to parse the urdf I don't have time to explain

    Args:
        urdf_path:
        end_effector_name:

    Returns:

    """
    # model = rtb.Robot.URDF_read(file_path=urdf_path, tld = get_package_share_directory("rviz_basic"))
    model = rtb.Robot.URDF(file_path=urdf_path)
    l = model.links
    links, name, urdf_string, urdf_filepath = rtb.Robot.URDF_read(file_path=urdf_path)
    joints_objects = URDF.loadstr(urdf_string, urdf_filepath).joints

    if end_effector_name is None:
        ETchain = model.ets().copy()
        joint_names = [j.name for j in joints_objects if j.joint_type != "fixed"]
        joint_index = list(range(len(joint_names)))

        for et in ETchain:
            et: ET
            if et.qlim is not None:
                if et.qlim[0] == 0.0 and et.qlim[1] == 0.0 or True:
                    et.qlim = None
        return model, ETchain, joint_names, joints_objects, None

    if type(end_effector_name) is int:
        segments = model.segments()
        lengths: NDArray = np.array([len(s) for s in segments], dtype=int)
        n: int = end_effector_name
        nth_longest_index: int = np.argsort(-lengths)[n]
        nth_longest_segment: List[Link | None] = segments[nth_longest_index]
        end_link: Link = nth_longest_segment[-1]
    else:
        end_link = [x for x in l if x.name == end_effector_name][0]
    ETchain: ETS = model.ets(
        # start="base_link",
        end=end_link,
    ).copy()
    for et in ETchain:
        et: ET
        if et.qlim is not None:
            if et.qlim[0] == 0.0 and et.qlim[1] == 0.0:
                et.qlim = None

    link: Link = end_link.copy()
    joint_index = []
    while link.parent is not None:
        link: Link
        parent: Link = link.parent.copy()
        for ind, joint in enumerate(joints_objects):
            if joint.parent == parent.name and joint.child == link.name:
                if joint.joint_type != "fixed":  # skips rigid joints
                    joint_index = [ind] + joint_index  # reverse fill
                link = parent
                break

    joint_names = [joints_objects[j].name for j in joint_index]

    # correct numbering by starting at 1 if not: bug
    counter = 0
    for et in ETchain:
        et: ET
        if et.isjoint:
            et.jindex = counter
            counter += 1

    return model, ETchain, joint_names, joints_objects, end_link
    # return model, ETchain.compile(), joint_names, joints_objects, end_link


def future_list_complete(future_list: List[Future]) -> bool:
    """Returns True is all futures in the input list are done.

    Args:
        future_list: a list of futures

    Returns:
        True if all futures are done
    """
    return bool(np.all([f.done() for f in future_list]))


class ClockRate:
    def __init__(
        self, parent: Node, frequency: float, clock: Optional[Clock] = None
    ) -> None:
        self.frequency = frequency
        self.parent = parent
        self.clock = clock
        self.last_clock = self.parent.get_clock().now()
        self.deltaTime = Time(
            seconds=1 / self.frequency,  # type: ignore
            clock_type=self.parent.get_clock().clock_type,
        )

    def get_clock(self):
        if self.clock is None:
            return self.parent.get_clock()
        else:
            return self.clock

    def sleep(self) -> None:
        second1, nanosec1 = self.last_clock.seconds_nanoseconds()
        second2, nanosec2 = self.deltaTime.seconds_nanoseconds()
        next_time = Time(
            seconds=second1 + second2,
            nanoseconds=nanosec1 + nanosec2,
            clock_type=self.get_clock().clock_type,
        )
        self.last_clock = next_time
        self.get_clock().sleep_until(next_time)

    def destroy(self) -> None:
        del self


class EZRate:
    """Creates a better rate where rate.destroy actually destroys the rate"""

    def __init__(
        self, parent: Node, frequency: float, clock: Optional[Clock] = None
    ) -> None:
        self.__frequency = frequency
        self.__parent = parent
        clock = self.__parent.get_clock() if clock is None else clock

        self.__rate = self.__parent.create_rate(self.__frequency, clock=clock)

    def sleep(self) -> None:
        self.__rate.sleep()

    def destroy(self) -> None:
        self.__parent.destroy_rate(self.__rate)

    def __del__(self):
        self.destroy()


class EliaNode(Node):
    def __init__(self, name: str):
        super().__init__(name)  # type: ignore
        self.Alias = name
        self.Yapping: bool = True

        self.declare_parameter("WAIT_FOR_LOWER_LEVEL", True)
        self.WAIT_FOR_NODES_OF_LOWER_LEVEL = (
            self.get_parameter("WAIT_FOR_LOWER_LEVEL").get_parameter_value().bool_value
        )
        self.NecessaryClientList: List[str] = []

    def sleep(self, seconds: float) -> None:
        """sleeps using the node's clock

        Args:
            seconds: time to sleep
        """
        self.get_clock().sleep_for(Duration(seconds=seconds))  # type: ignore

    def wait_on_futures(self, future_list: List[Future], wait_Hz: float = 10):
        while not future_list_complete(future_list):
            self.sleep(1 / wait_Hz)

    def tf2np(self, tf: Transform) -> Tuple[NDArray, qt.quaternion]:
        """converts a TF into a np array and quaternion

        Args:
            tf: TF to convert

        Returns:
            xyz - NDArray: xyz coordinates
            quat - qt.quaternion: quaternion for the rotation
        """
        xyz = np.array(
            [tf.translation.x, tf.translation.y, tf.translation.z], dtype=float
        )
        quat = qt.quaternion()
        quat.w = tf.rotation.w
        quat.x = tf.rotation.x
        quat.y = tf.rotation.y
        quat.z = tf.rotation.z
        return xyz, quat

    def np2tf(self, coord: np.ndarray, quat: qt.quaternion = qt.one) -> Transform:
        """converts an NDArray and quaternion into a Transform.

        Args:
            xyz - NDArray: xyz coordinates
            quat - qt.quaternion: quaternion for the rotation

        Returns:
            tf: resulting TF
        """
        tf = Transform()
        tf.translation.x, tf.translation.y, tf.translation.z = tuple(
            coord.astype(float).tolist()
        )
        tf.rotation.w = quat.w
        tf.rotation.x = quat.x
        tf.rotation.y = quat.y
        tf.rotation.z = quat.z
        return tf

    def perror(self, object, force: bool = False):
        if self.Yapping or force:
            self.get_logger().error(f"[{self.Alias}] {object}")

    def pwarn(self, object, force: bool = False):
        if self.Yapping or force:
            self.get_logger().warn(f"[{self.Alias}] {object}")

    def pinfo(self, object, force: bool = False):
        if self.Yapping or force:
            self.get_logger().info(f"[{self.Alias}] {object}")

    def setAndBlockForNecessaryClients(
        self,
        LowerLevelClientList: Optional[List[str] | str] = None,
        cut_last: int = 6,
        all_requiered: bool = True,
    ):
        silent = 3
        if type(LowerLevelClientList) is str:
            self.NecessaryClientList = [LowerLevelClientList]
        elif type(LowerLevelClientList) is list:
            self.NecessaryClientList = LowerLevelClientList
        if not self.WAIT_FOR_NODES_OF_LOWER_LEVEL:
            timeout = 0.5
        else:
            timeout = 2
        timeout /= max(1, len(self.NecessaryClientList))

        cli_list = self.NecessaryClientList.copy()
        while cli_list:
            for client_name in cli_list:
                self.necessary_client = self.create_client(Empty, client_name)

                if self.necessary_client.wait_for_service(timeout_sec=timeout):
                    cli_list.remove(client_name)
                    self.pinfo(
                        bcolors.OKBLUE
                        + f"""[{client_name[:-cut_last]}] connected :)"""
                        + bcolors.ENDC,
                        force=True,
                    )
                    if not all_requiered:
                        return

            if not self.WAIT_FOR_NODES_OF_LOWER_LEVEL:
                break
            if cli_list and silent <= 0:
                self.pwarn(
                    f"""Blocking: Waiting for {cli_list} services""",
                    force=True,
                )
            else:
                silent -= 1
        if not self.WAIT_FOR_NODES_OF_LOWER_LEVEL and cli_list:
            self.pinfo(
                f"""{bcolors.WARNING}Launched alone {bcolors.OKBLUE}¯\_(ツ)_/¯{bcolors.ENDC}\nUse self.WAIT_FOR_NODES_OF_LOWER_LEVEL = True to wait""",
                force=True,
            )

    def get_and_wait_Client(
        self, service_name: str, service_type, cbk_grp: Optional[CallbackGroup] = None
    ) -> Client:
        srv = self.create_client(
            service_type,
            service_name,
            callback_group=cbk_grp,  # type: ignore
        )
        while not srv.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"service [{service_name}] not available, waiting ...")
        return srv

    def create_EZrate(self, frequency: float, clock: Optional[Clock] = None) -> EZRate:
        """Creates a better rate where rate.destroy actually destroys the rate

        Args:
            frequency: frequency of the rate
            clock: clock to use

        Returns:
            EZRate manipulating a Rate object

        """
        # return ClockRate(self, frequency, clock)
        return EZRate(self, frequency, clock)

    def execute_in_cbk_group(
        self, fun: Callable, callback_group: Optional[CallbackGroup] = None
    ) -> Tuple[Future, GuardCondition]:
        """Executes the given function by adding it as a callback to a callback_group.

        Pretty sure that's not how it should be done.

        Args:
            fun - function: function to execute
            callback_group - CallbackGroup: callback group in which to execute the function
        Returns:
            future: holds the future results
            quardian: the guard condition object in the callback_group
        """
        if callback_group is None:
            callback_group = self.default_callback_group

        future = Future()

        def fun_with_future():
            if not future.done():  # guardian triggers several times, idk why.
                # so this condition protects this mess from repeting itselfs randomly
                result = fun()
                future.set_result(result)
                return result

        # guardian will execute fun_with_future inside of callback_group
        guardian = self.create_guard_condition(fun_with_future, callback_group)
        guardian.trigger()
        future.add_done_callback(lambda result: self.destroy_guard_condition(guardian))
        return future, guardian


class Bcolors:
    def __init__(self) -> None:
        self.HEADER = """\033[95m"""
        self.OKBLUE = """\033[94m"""
        self.OKCYAN = """\033[96m"""
        self.OKGREEN = """\033[92m"""
        self.WARNING = """\033[93m"""
        self.FAIL = """\033[91m"""
        self.ENDC = """\033[0m"""
        self.BOLD = """\033[1m"""
        self.UNDERLINE = """\033[4m"""


bcolors = Bcolors()
