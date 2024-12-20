"""
I this adds functionalities to the default ros2 Node object

Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

import os
from os import PathLike, getenv
import re

import matplotlib
from launch_ros.substitutions.find_package import get_package_share_directory

matplotlib.use("Agg")  # fix for when there is no display


import signal
import time
import traceback
from time import sleep  # do not use unless you know what you are doing
from typing import Any, Callable, Iterable, Optional, Sequence, Set, Tuple, Union

import numpy as np
import rclpy
import roboticstoolbox as rtb
from geometry_msgs.msg import Transform, TransformStamped, Vector3
from motion_stack_msgs.msg import TargetSet
from motion_stack_msgs.srv import TFService
from numpy.typing import NDArray
from rclpy.callback_groups import CallbackGroup
from rclpy.client import Client
from rclpy.clock import Clock, ClockType
from rclpy.constants import S_TO_NS
from rclpy.executors import (
    ExternalShutdownException,
    MultiThreadedExecutor,
    SingleThreadedExecutor,
)
from rclpy.guard_condition import GuardCondition
from rclpy.node import List, Node, Parameter, Rate
from rclpy.task import Future
from rclpy.time import Duration, Time
from roboticstoolbox.robot import Robot
from roboticstoolbox.robot.ET import ET, SE3
from roboticstoolbox.robot.ETS import ETS
from roboticstoolbox.robot.Link import Link
from roboticstoolbox.tools import URDF
from roboticstoolbox.tools.urdf.urdf import Joint
from std_srvs.srv import Empty, Trigger

from easy_robot_control.utils.math import Quaternion, qt, qt_normalize

ROS_DISTRO = getenv("ROS_DISTRO")


def error_catcher(func):
    # This is a wrapper to catch and display exceptions
    def wrap(*args, **kwargs):
        try:
            out = func(*args, **kwargs)
        except Exception as exception:
            if (
                isinstance(exception, KeyboardInterrupt)
                or isinstance(exception, ExternalShutdownException)
                or isinstance(exception, rclpy._rclpy_pybind11.RCLError)
            ):
                raise exception
            else:
                try:
                    traceback_logger_node = Node("error_node")  # type: ignore
                    traceback_logger_node.get_logger().error(traceback.format_exc())
                    traceback_logger_node.destroy_node()
                    try:
                        rclpy.shutdown()
                    except:
                        pass
                    quit()
                    # raise ExternalShutdownException()
                except Exception as logging_exception:
                    print(f"Logging failed {logging_exception}")
                    raise exception
        return out

    return wrap


def rosTime2Float(time: Union[Time, Duration]) -> float:
    """Converts ros2 time objects to seconds as float

    Args:
        time: ros time obj

    Returns:
        corresponding seconds as float value
    """
    sec: float = time.nanoseconds / S_TO_NS
    return sec


def list_cyanize(l: Iterable, default_color=None) -> str:
    if default_color is None:
        default_color = bcolors.ENDC
    out = "["
    first = True
    for k in l:
        if not first:
            out += ", "
        first = False
        if isinstance(k, str):
            out += f"'{bcolors.OKCYAN}{k}{default_color}'"
        else:
            out += f"{bcolors.OKCYAN}{k}{default_color}"
    out += "]"
    return out


def replace_incompatible_char_ros2(string_to_correct: str) -> str:
    """replace characcter that cannot be used for Ros2 Topics by _
    inserts "WARN" in front if topic starts with a number

    Args:
        string_to_correct

    Returns:
        corrected_string
    """
    corrected_string = string_to_correct
    corrected_string = re.sub(r'[^a-zA-Z0-9/~]', '_', corrected_string)
    corrected_string = re.sub(r'/(?=[^a-zA-Z])', '/WARN', corrected_string)
    if string_to_correct[0].isdigit():
        corrected_string = "WARN" + string_to_correct
    return corrected_string


def get_src_folder(package_name: str) -> str:
    """Absolute path to workspace/src/package.
    Avoid using this

    Args:
        package_name:

    Returns: Absolute path as str

    """
    package_share_directory = get_package_share_directory(package_name)
    workspace_root = os.path.abspath(os.path.join(package_share_directory, "../../../.."))
    package_src_directory = os.path.join(workspace_root, "src", package_name)
    return package_src_directory


def transform_joint_to_transform_Rx(transform: ET, jointET: ET) -> ET:
    """Takes a transform and a joint (TRANSFORM * +-Rxyz), and returns the
    (rotational) transform so that RESULT * Rx = TRANSFORM * +-Rxyz.
    So the transform now places the x base vector onto the axis.

    Args:
        transform:
        jointET:

    Returns:
        RESULT * Rx = TRANSFORM * +-Rxyz

    """
    ax = jointET.axis
    flip_sign = -1 if jointET.isflip else 1
    if ax == "Rx":
        ax_et = SE3.RPY(0, 0, 0) if flip_sign == 1 else SE3.RPY(0, 0, np.pi)
    elif ax == "Ry":
        ax_et = SE3.RPY(0, 0, flip_sign * np.pi / 2)
    elif ax == "Rz":
        ax_et = SE3.RPY(0, flip_sign * np.pi / 2, 0)
    else:
        raise TypeError(f"axis type {ax} is not compatible with Rxyz")
    pure_rot_to_axis = SE3()
    pure_rot_to_axis.A[:3, :3] = (transform.A() * ax_et)[:3, :3]
    return ET.SE3(pure_rot_to_axis)


def loadAndSet_URDF(
    urdf_path: str,
    end_effector_name: Optional[Union[str, int]] = None,
    start_effector_name: Optional[str] = None,
) -> Tuple[Robot, ETS, List[str], List[Joint], Optional[Link]]:
    """I am so sorry. This works to parse the urdf I don't have time to explain

    Args:
        urdf_path:
        end_effector_name:

    Returns:

    """
    # model = rtb.Robot.URDF_read(file_path=urdf_path, tld = get_package_share_directory("rviz_basic"))
    full_model = rtb.Robot.URDF(file_path=urdf_path)
    l = full_model.links
    links, name, urdf_string, urdf_filepath = rtb.Robot.URDF_read(file_path=urdf_path)
    joints_objects = URDF.loadstr(urdf_string, urdf_filepath).joints

    if end_effector_name is None:
        exctracted_chain = full_model.ets().copy()
        joint_names = [j.name for j in joints_objects if j.joint_type != "fixed"]
        joint_index = list(range(len(joint_names)))

        for et in exctracted_chain:
            et: ET
            if et.qlim is not None:
                if (
                    (et.qlim[0] == 0.0 and et.qlim[1] == 0.0)
                    or et.qlim[0] is None
                    or et.qlim[1] is None
                ):
                    et.qlim = None
        return full_model, exctracted_chain, joint_names, joints_objects, None

    if start_effector_name is not None:
        simil_link_names = [x for x in l if x.name == start_effector_name]
        if simil_link_names:
            start_link = [x for x in l if x.name == start_effector_name][0]
        else:
            start_link = l[0]
    else:
        start_link = None

    if type(end_effector_name) is int:  # picks Nth longest segment
        segments = full_model.segments()
        if start_link is not None:
            segments = [seg for seg in segments if start_link in seg]
        lengths: NDArray = np.array([len(s) for s in segments], dtype=int)
        n: int = end_effector_name
        nth_longest_index: int = np.argsort(-lengths)[n]
        nth_longest_segment: List[Optional[Link]] = segments[nth_longest_index]
        end_link: Link = nth_longest_segment[-1]
    elif start_effector_name == end_effector_name:
        end_link = start_link
    else:
        end_links = [x for x in l if x.name == end_effector_name]
        if not end_links:
            raise ValueError(f"{end_effector_name=} not in {urdf_path=}")
        end_link = end_links[0]

    # print(start_link, end_link)
    exctracted_chain: ETS = full_model.ets(
        start=start_link,
        end=end_link,
    ).copy()
    for et in exctracted_chain:
        et: ET
        if et.qlim is not None:
            if (
                (et.qlim[0] == 0.0 and et.qlim[1] == 0.0)
                or et.qlim[0] is None
                or et.qlim[1] is None
            ):
                et.qlim = None

    # exctracts all joints
    link: Link = end_link.copy()
    joint_index = []
    while link.children != start_effector_name and link.parent is not None:
        link: Link
        parent: Link = link.parent.copy()
        for ind, joint in enumerate(joints_objects):
            if joint.parent == parent.name and joint.child == link.name:
                if joint.joint_type != "fixed":  # skips rigid joints
                    joint_index = [ind] + joint_index  # reverse fill
                break
        if link.name == start_effector_name:
            break
        link = parent

    joints_objects = [joints_objects[j] for j in joint_index]
    joint_names = [jo.name for jo in joints_objects]

    # correct numbering by starting at 1 if not: bug
    counter = 0
    for et in exctracted_chain:
        et: ET
        if et.isjoint:
            et.jindex = counter
            counter += 1

    return full_model, exctracted_chain, joint_names, joints_objects, end_link


def future_list_complete(future_list: Union[List[Future], Future]) -> bool:
    """Returns True is all futures in the input list are done.

    Args:
        future_list: a list of futures

    Returns:
        True if all futures are done
    """
    if isinstance(future_list, Future):
        return future_list.done()
    else:
        return bool(np.all([f.done() for f in future_list]))


class ClockRate:
    def __init__(
        self, parent: Node, frequency: float, clock: Optional[Clock] = None
    ) -> None:
        """DO NOT USE"""
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
        self.get_clock().sleep_until(next_time)  # won't work with foxy

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

        self.__rate: Rate = self.__parent.create_rate(self.__frequency, clock=clock)

    def sleep(self) -> None:
        self.__rate.sleep()

    def destroy(self) -> None:
        self.__parent.destroy_rate(self.__rate)

    def is_ready(self) -> bool:
        return self.__rate._timer.is_ready()

    def __del__(self):
        self.destroy()
        del self


class EliaNode(Node):
    def __init__(self, name: str):
        super().__init__(name)  # type: ignore
        self.Alias = name
        self.Yapping: bool = True

        self.declare_parameter("WAIT_FOR_LOWER_LEVEL", True)
        self.WAIT_FOR_NODES_OF_LOWER_LEVEL = (
            self.get_parameter("WAIT_FOR_LOWER_LEVEL").get_parameter_value().bool_value
        )
        self.__necessary_clients: Set[str] = set()

        self.check_duplicateTMR = self.create_timer(1, self.check_duplicateTMRCBK)

    def wait_for_lower_level(
        self, more_services: Iterable[str] = set(), all_requiered: bool = False
    ):
        """You can overload this or use the launch setting to wait for a service"""
        self.declare_parameter("services_to_wait", [""])
        from_prams = set(
            self.get_parameter("services_to_wait")
            .get_parameter_value()
            .string_array_value
        ) - {""}
        self.__necessary_clients |= set(more_services)
        self.__necessary_clients |= from_prams
        self.set_parameters(
            [
                Parameter(
                    name="services_to_wait",
                    type_=Parameter.Type.STRING_ARRAY,
                    value=list(self.__necessary_clients),
                )
            ]
        )
        self.setAndBlockForNecessaryClients(all_requiered=all_requiered)

    @error_catcher
    def check_duplicateTMRCBK(self):
        self.destroy_timer(self.check_duplicateTMR)
        node_info = self.get_node_names_and_namespaces()
        my_name = self.get_name()
        my_namespace = self.get_namespace()
        i_have_seen_myself = False
        # self.pwarn(node_info)
        # self.pwarn(my_name)
        # self.pwarn(my_namespace)
        for node_name, node_namespace in node_info:
            if node_name == my_name and node_namespace == my_namespace:
                if not i_have_seen_myself:
                    i_have_seen_myself = True
                    continue
                for k in range(3):
                    self.perror(
                        f"CRITICAL WARNING: node with similar name and namespace '{my_namespace+my_name}'. You might have forgoten to kill a previous node.",
                        force=True,
                    )
                    time.sleep(1)

    def getNow(self) -> Time:
        return self.get_clock().now()

    def sleep(self, seconds: float) -> None:
        """sleeps using the node's clock

        Args:
            seconds: time to sleep
        """
        if ROS_DISTRO == "humble":
            # self.pinfo("humble sleep")
            self.get_clock().sleep_for(Duration(seconds=seconds))  # type: ignore
        else:
            # foxy does not have the above function, so we fall back here
            # self.pinfo("foxy sleep")
            # time.sleep(seconds)
            # rate = self.create_rate(1 / seconds)
            # rate.sleep()

            end_time = self.get_clock().now() + Duration(
                seconds=seconds
            )  # End time is the current time plus duration

            # Loop and sleep in increments until the end time is reached
            while self.get_clock().now() < end_time:
                # self.pinfo("z")
                time.sleep(1 / 100)

    def wait_on_futures(
        self, future_list: Union[List[Future], Future], wait_Hz: float = 10
    ):
        """Waits for the completion of a list of futures, checking completion at the
        provided rate.

        Args:
            future_list: List of Future to wait for
            wait_Hz: rate at which to wait
        """
        while not future_list_complete(future_list):
            self.sleep(1 / wait_Hz)

    def np2tfReq(
        self, coord: Optional[np.ndarray] = None, quat: Optional[Quaternion] = None
    ) -> TFService.Request:
        """converts an NDArray and quaternion into a Transform request for a service.

        Args:
            xyz - NDArray: xyz coordinates
            quat - Quaternion: quaternion for the rotation

        Returns:
            TFService.Request: resulting Request for a service call
        """
        request = TFService.Request()
        request.tf = np2tf(coord, quat)
        return request

    def perror(self, object: Any, force: bool = False):
        """Prints/Logs error if Yapping==True (default) or force==True.

        Args:
            object: Thing to print
            force - bool: if True the message will print whatever if self.Yapping is.
        """
        if self.Yapping or force:
            self.get_logger().error(f"[{self.Alias}] {object}")

    def pwarn(self, object, force: bool = False):
        """Prints/Logs error if Yapping==True (default) or force==True.

        Args:
            object: Thing to print
            force - bool: if True the message will print whatever if self.Yapping is.
        """
        if self.Yapping or force:
            self.get_logger().warn(f"[{self.Alias}] {object}")

    def pinfo(self, object, force: bool = False):
        """Prints/Logs error if Yapping==True (default) or force==True.

        Args:
            object: Thing to print
            force - bool: if True the message will print whatever if self.Yapping is.
        """
        if self.Yapping or force:
            self.get_logger().info(f"[{self.Alias}] {object}")

    def resolve_service_name(self, service: str, *, only_expand: bool = False) -> str:
        if ROS_DISTRO == "humble":
            return super().resolve_service_name(service, only_expand=only_expand)
        elif ROS_DISTRO == "foxy":
            # oh no nothing exists
            name = service
            if name[0] != "/":
                name = self.get_namespace() + "/" + name
            return name
        else:
            return super().resolve_service_name(service, only_expand=only_expand)

    def setAndBlockForNecessaryClients(
        self,
        all_requiered: bool = True,
    ) -> None:
        """Waits for all clients in LowerLevelClientList to be alive"""
        silent = 3

        cli_list: List[str] = list(self.__necessary_clients.copy())
        for i, n in enumerate(cli_list):
            cli_list[i] = self.resolve_service_name(n)
        client_missing: Set[str] = set(cli_list)
        while client_missing:
            servers: Sequence[Tuple[str, List[str]]] = self.get_service_names_and_types()
            alive_names: Set[str] = {n for n, t in servers}
            in_both = client_missing & alive_names
            if in_both:
                self.pinfo(
                    bcolors.OKBLUE
                    + f"""{list_cyanize(list(in_both), default_color=bcolors.OKBLUE)} """
                    f"connected :)" + bcolors.ENDC,
                    force=True,
                )
                if not all_requiered:
                    return
                client_missing -= in_both
            if not self.WAIT_FOR_NODES_OF_LOWER_LEVEL:
                break
            if client_missing and silent == 0:
                self.pwarn(
                    f"""Blocking: Waiting for {client_missing} services""",
                    force=True,
                )
            silent -= 1
            self.sleep(1)
        if not self.WAIT_FOR_NODES_OF_LOWER_LEVEL and client_missing:
            self.pinfo(
                rf"{bcolors.WARNING}Launched alone {bcolors.OKBLUE}¯\_(ツ)_/¯"
                f"{bcolors.ENDC}\nUse self."
                f"WAIT_FOR_NODES_OF_LOWER_LEVEL = True to wait",
                force=True,
            )
        return

    def setAndBlockForNecessaryNodes(
        self,
        necessary_node_names: Union[List[str], str],
        silent_trial: Optional[int] = 3,
        intervalSec: Optional[float] = 0.5,
    ):
        node_names: List[str]
        if isinstance(necessary_node_names, str):
            node_names = [necessary_node_names]
        elif isinstance(necessary_node_names, list):
            node_names = necessary_node_names

        if silent_trial is None:
            silent_trial = 2
        if intervalSec is None:
            intervalSec = 1

        nodes_connected = False

        while not nodes_connected:
            for name in node_names:
                node_info = self.get_node_names_and_namespaces()
                for node_name, node_namespace in node_info:
                    if node_name == name:
                        self.pinfo(
                            bcolors.OKBLUE
                            + f"""[{node_name}] node connected :)"""
                            + bcolors.ENDC,
                            force=True,
                        )
                        nodes_connected = True
                        break

            if not nodes_connected and silent_trial == 0:
                self.pwarn(
                    f"""Blocking: Waiting for {node_names} nodes""",
                    force=True,
                )
                sleep(intervalSec)
            elif not nodes_connected:
                sleep(intervalSec)
            silent_trial -= 1

    def get_and_wait_Client(
        self, service_name: str, service_type, cbk_grp: Optional[CallbackGroup] = None
    ) -> Client:
        """Return the client to the corresponding service, wait for it ot be available.

        Args:
            service_name - str:
            service_type - Ros2 service_type:
            cbk_grp: Not important I think but it's there

        Returns:

        """
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
        # tmr = self.create_timer(0.00001, fun_with_future, callback_group)
        # future.add_done_callback(lambda result: self.destroy_timer(tmr))
        return future, guardian


def np2TargetSet(arr: Optional[NDArray] = None) -> TargetSet:
    if arr is None:
        return TargetSet()
    vects: List[Vector3] = []
    arrf = arr.astype(float, copy=True)
    for row in range(arrf.shape[0]):
        vects.append(
            Vector3(
                x=arrf[row, 0],
                y=arrf[row, 1],
                z=arrf[row, 2],
            )
        )
    return TargetSet(vector_list=vects)


def targetSet2np(ts: TargetSet) -> NDArray:
    vects: Sequence[Vector3] = ts.vector_list
    arr = np.empty(shape=(len(vects), 3), dtype=float)
    for i, v in enumerate(vects):
        v: Vector3
        arr[i, :] = (v.x, v.y, v.z)
    return arr


def tf2np(tf: Transform) -> Tuple[NDArray, Quaternion]:
    """converts a TF into a np array and quaternion

    Args:
        tf: TF to convert

    Returns:
        xyz - NDArray: xyz coordinates
        quat - Quaternion: quaternion for the rotation
    """
    xyz = np.array(
        [tf.translation.x, tf.translation.y, tf.translation.z], dtype=float
    )
    quat = Quaternion()
    quat.w = tf.rotation.w
    quat.x = tf.rotation.x
    quat.y = tf.rotation.y
    quat.z = tf.rotation.z
    quat = qt_normalize(quat)
    return xyz, quat

def np2tf(
    coord: Union[None, NDArray, Sequence[float]] = None,
    quat: Optional[Quaternion] = None,
    sendNone: bool = False,
) -> Transform:
    """converts an NDArray and quaternion into a Transform.

    Args:
        coord - NDArray: xyz coordinates
        quat - Quaternion: quaternion for the rotation

    Returns:
        tf: resulting TF
    """
    xyz: NDArray
    rot: Quaternion
    if coord is None:
        if sendNone:
            xyz = np.array([np.nan] * 3, dtype=float)
        else:
            xyz = np.array([0.0, 0.0, 0.0], dtype=float)
    elif isinstance(coord, list):
        xyz = np.array(coord, dtype=float)
    else:
        xyz = coord.astype(float)
    if quat is None:
        if sendNone:
            rot = qt.from_float_array(np.array([np.nan] * 4, dtype=float))
        else:
            rot = qt.one.copy()
    else:
        rot = quat

    assert isinstance(xyz, np.ndarray)
    assert isinstance(rot, Quaternion)
    assert xyz.shape == (3,)
    assert xyz.dtype == np.float64

    rot = qt_normalize(rot)

    tf = Transform()
    tf.translation.x = xyz[0]
    tf.translation.y = xyz[1]
    tf.translation.z = xyz[2]
    tf.rotation.w = rot.w
    tf.rotation.x = rot.x
    tf.rotation.y = rot.y
    tf.rotation.z = rot.z
    return tf


class Bcolors:
    def __init__(self) -> None:
        self.HEADER = """\033[95m"""
        self.OKBLUE = """\033[94m"""
        self.OKCYAN = """\033[96m"""
        self.OKGREEN = """\033[92m"""
        self.WARNING = """\x1b[33;20m"""
        self.FAIL = """\033[91m"""
        self.ENDC = """\033[0m"""
        self.BOLD = """\033[1m"""
        self.UNDERLINE = """\033[4m"""


bcolors = Bcolors()


def myMain(nodeClass, multiThreaded=False, args=None):
    rclpy.init()

    try:
        node = nodeClass()
    except KeyboardInterrupt:
        m = f"{bcolors.OKCYAN}KeyboardInterrupt intercepted, {bcolors.OKBLUE}shuting down. :){bcolors.ENDC}"
        print(m)
        return
    except ExternalShutdownException:
        m = f"{bcolors.OKCYAN}External Shutdown Command intercepted, {bcolors.OKBLUE}shuting down. :){bcolors.ENDC}"
        print(m)
        return
    # except rclpy._rclpy_pybind11.RCLError:
    #     m = f"{bcolors.OKCYAN}Stuck waiting intercepted, {bcolors.OKBLUE}shuting down. :){bcolors.ENDC}"
    #     print(m)
    #     return
    except Exception as exception:
        m = f"Exception intercepted: {bcolors.FAIL}{traceback.format_exc()}{bcolors.ENDC}"
        print(m)
        return

    if multiThreaded:
        executor = MultiThreadedExecutor()
    else:
        executor = SingleThreadedExecutor()  # better perf
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        m = f"{bcolors.OKCYAN}KeyboardInterrupt intercepted, {bcolors.OKBLUE}shuting down. :){bcolors.ENDC}"
        print(m)
        return
    except ExternalShutdownException:
        m = f"{bcolors.OKCYAN}External Shutdown Command intercepted, {bcolors.OKBLUE}shuting down. :){bcolors.ENDC}"
        print(m)
        return
    # except rclpy._rclpy_pybind11.RCLError:
    #     m = f"{bcolors.OKCYAN}Stuck waiting intercepted, {bcolors.OKBLUE}shuting down. :){bcolors.ENDC}"
    #     print(m)
    #     return

    except Exception as exception:
        m = f"Exception intercepted: \033[91m{traceback.format_exc()}\033[0m"
        print(m)

    try:
        node.destroy_node()
    except:
        pass
    try:
        rclpy.shutdown()
    except:
        pass
