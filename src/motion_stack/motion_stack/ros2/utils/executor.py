import matplotlib

matplotlib.use("Agg")  # fix for when there is no display, don't ask why

import os
import re
import time
import traceback
from functools import wraps
from os import getenv
from time import sleep  # do not use unless you know what you are doing
from typing import Any, Callable, Iterable, Optional, Sequence, Set, Tuple, Union

import numpy as np
import rclpy
from geometry_msgs.msg import Transform, TransformStamped, Vector3
from launch_ros.substitutions.find_package import get_package_share_directory
from motion_stack_msgs.msg import TargetSet
from motion_stack_msgs.srv import TFService
from nptyping import NDArray
from rclpy import Node
from rclpy.callback_groups import CallbackGroup
from rclpy.client import Client
from rclpy.clock import Clock
from rclpy.constants import S_TO_NS
from rclpy.executors import (
    ExternalShutdownException,
    MultiThreadedExecutor,
    SingleThreadedExecutor,
)
from rclpy.guard_condition import GuardCondition
from rclpy.node import List, Parameter, Rate
from rclpy.task import Future
from rclpy.time import Duration
from rclpy.time import Time as TimeRos

from ...core.utils.joint_state import JState, Time
from ...core.utils.printing import TCOL, list_cyanize
from ...core.utils.static_executor import Spinner, extract_inner_type
from .conversion import ros_to_time

ROS_DISTRO = getenv("ROS_DISTRO")


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


class EZRate:

    def __init__(
        self, parent: Node, frequency: float, clock: Optional[Clock] = None
    ) -> None:
        """Creates a better rate where rate.destroy actually destroys the rate

        Note:
            Favorize EliaNode.create_EZrate instead of creating an instance.

        Args:
            parent: spinning node
        """
        self.__frequency = frequency
        self.__parent = parent
        clock = self.__parent.get_clock() if clock is None else clock

        self.__rate: Rate = self.__parent.create_rate(self.__frequency, clock=clock)

    def sleep(self) -> None:
        """sleeps (blocking) until next tick"""
        self.__rate.sleep()

    def destroy(self) -> None:
        """Destroys the object"""
        self.__parent.destroy_rate(self.__rate)

    def is_ready(self) -> bool:
        return self.__rate._timer.is_ready()

    def __del__(self):
        self.destroy()
        del self


class Ros2Spinner(Spinner):
    def __init__(self, node: Node):
        """Ros2 node overloaded with usefull stuff."""
        super().__init__()
        self.node = node
        self.Alias = node.get_name
        self.Yapping: bool = True

        self.WAIT_FOR_LOWER_LEVEL = self.get_parameter(
            "WAIT_FOR_LOWER_LEVEL", bool, False
        )
        self.__necessary_clients: Set[str] = set()

        self.__check_duplicateTMR = self.create_timer(1, self.__check_duplicateTMRCBK)

    def get_parameter(self, name: str, value_type: type, default=None) -> Any:
        self.node.declare_parameter(name, default)
        if value_type == bool:
            return self.node.get_parameter(name).get_parameter_value().bool_value
        if value_type == int:
            return self.node.get_parameter(name).get_parameter_value().integer_value
        if value_type == float:
            return self.node.get_parameter(name).get_parameter_value().double_value
        if value_type == str:
            return self.node.get_parameter(name).get_parameter_value().string_value

        inner_type = extract_inner_type(value_type)
        if inner_type == bool:
            return self.node.get_parameter(name).get_parameter_value().bool_array_value
        if inner_type == int:
            return self.node.get_parameter(name).get_parameter_value().integer_array_value
        if inner_type == float:
            return self.node.get_parameter(name).get_parameter_value().double_array_value
        if inner_type == str:
            return self.node.get_parameter(name).get_parameter_value().string_array_value
        raise ValueError(f"{value_type=} is not part of Ros2 parameter type.")

    def wait_for_lower_level(
        self, more_services: Iterable[str] = set(), all_requiered: bool = False
    ):
        """Blocks until all or any service is available.

        Note:
            - List of waited services is given by `services_to_wait` ros2 param
            - Overload this to wait for an additional service

        Args:
            more_services: add more services
            all_requiered: if True, all listed services must be available
        """
        from_prams = set(self.get_parameter("services_to_wait", List[str], [""])) - {""}
        self.__necessary_clients |= set(more_services)
        self.__necessary_clients |= from_prams
        self.__setAndBlockForNecessaryClients(all_requiered=all_requiered)

    def __check_duplicateTMRCBK(self):
        """Check if a node with similar name exists"""
        self.node.destroy_timer(self.__check_duplicateTMR)
        node_info = self.node.get_node_names_and_namespaces()
        my_name = self.node.get_name()
        my_namespace = self.node.get_namespace()
        i_have_seen_myself = False
        for node_name, node_namespace in node_info:
            if node_name == my_name and node_namespace == my_namespace:
                if not i_have_seen_myself:
                    i_have_seen_myself = True
                    continue
                for k in range(3):
                    self.error(
                        f"CRITICAL WARNING: node with similar name and namespace '{my_namespace+my_name}'. You might have forgoten to kill a previous node.",
                        force=True,
                    )
                    time.sleep(1)

    def now(self) -> Time:
        """quick: self.get_clock().now()"""
        return ros_to_time(self.node.get_clock().now())

    def sleep(self, seconds: float) -> None:
        """sleeps using the node's clock.

        Note:
            Special case for good old foxy

        Args:
            seconds: time to sleep
        """
        if ROS_DISTRO == "foxy":
            end_time = self.now() + Time(
                sec=seconds
            )  # End time is the current time plus duration

            # Loop and sleep in increments until the end time is reached
            while now() < end_time:
                # self.pinfo("z")
                time.sleep(1 / 100)
        else:
            self.node.get_clock().sleep_for(Duration(seconds=seconds))  # type: ignore

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

    def error(self, *args, force: bool = False):
        """Prints/Logs error if Yapping==True (default) or force==True.

        Args:
            object: Thing to print
            force - bool: if True the message will print whatever if self.Yapping is.
        """
        self.node.get_logger().error(f"[{self.Alias}] {str(*args)}")

    def warn(self, *args, force: bool = False):
        """Prints/Logs warning if Yapping==True (default) or force==True.

        Args:
            object: Thing to print
            force - bool: if True the message will print whatever if self.Yapping is.
        """
        self.node.get_logger().warn(f"[{self.Alias}] {str(*args)}")

    def info(self, *args, force: bool = False):
        """Prints/Logs info if Yapping==True (default) or force==True.

        Args:
            object: Thing to print
            force: if True the message will print whatever if self.Yapping is.
        """
        self.node.get_logger().info(f"[{self.Alias}] {str(*args)}")

    def debug(self, *args, force: bool = False):
        """Prints/Logs info if Yapping==True (default) or force==True.

        Args:
            object: Thing to print
            force: if True the message will print whatever if self.Yapping is.
        """
        self.node.get_logger().debug(f"[{self.Alias}] {str(*args)}")

    def resolve_service_name(self, service: str, *, only_expand: bool = False) -> str:
        """
        Return a service name expanded and remapped.

        Note:
            Overloaded to handle missing foxy

        :param service: service name to be expanded and remapped.
        :param only_expand: if `True`, remapping rules won't be applied.
        :return: a fully qualified service name,
            result of applying expansion and remapping to the given `service`.
        """
        if ROS_DISTRO == "humble":
            return self.node.resolve_service_name(service, only_expand=only_expand)
        elif ROS_DISTRO == "foxy":
            # oh no nothing exists
            name = service
            if name[0] != "/":
                name = self.node.get_namespace() + "/" + name
            return name
        else:
            return self.node.resolve_service_name(service, only_expand=only_expand)

    def __setAndBlockForNecessaryClients(
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
            servers: Sequence[Tuple[str, List[str]]] = (
                self.node.get_service_names_and_types()
            )
            alive_names: Set[str] = {n for n, t in servers}
            in_both = client_missing & alive_names
            if in_both:
                self.info(
                    TCOL.OKBLUE
                    + f"""{list_cyanize(list(in_both), default_color=TCOL.OKBLUE)} """
                    f"connected :)" + TCOL.ENDC,
                    force=True,
                )
                if not all_requiered:
                    return
                client_missing -= in_both
            if not self.WAIT_FOR_LOWER_LEVEL:
                break
            if client_missing and silent == 0:
                self.warn(
                    f"""Blocking: Waiting for {client_missing} services""",
                    force=True,
                )
            silent -= 1
            self.sleep(1)
        if not self.WAIT_FOR_LOWER_LEVEL and client_missing:
            self.info(
                rf"{TCOL.WARNING}Launched alone {TCOL.OKBLUE}¯\_(ツ)_/¯"
                f"{TCOL.ENDC}\nUse self."
                f"WAIT_FOR_LOWER_LEVEL = True to wait",
                force=True,
            )
        return

    def setAndBlockForNecessaryNodes(
        self,
        necessary_node_names: Union[List[str], str],
        silent_trial: Optional[int] = 3,
        intervalSec: Optional[float] = 0.5,
    ):
        """Blocks for nodes to be alive"""
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
                node_info = self.node.get_node_names_and_namespaces()
                for node_name, node_namespace in node_info:
                    if node_name == name:
                        self.info(
                            TCOL.OKBLUE
                            + f"""[{node_name}] node connected :)"""
                            + TCOL.ENDC,
                            force=True,
                        )
                        nodes_connected = True
                        break

            if not nodes_connected and silent_trial == 0:
                self.warn(
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
        srv = self.node.create_client(
            service_type,
            service_name,
            callback_group=cbk_grp,  # type: ignore
        )
        while not srv.wait_for_service(timeout_sec=2.0):
            self.warn(f"service [{service_name}] not available, waiting ...")
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
        return EZRate(self.node, frequency, clock)

    def execute_in_cbk_group(
        self, fun: Callable, callback_group: Optional[CallbackGroup] = None
    ) -> Tuple[Future, GuardCondition]:
        """Executes the given function by adding it as a callback to a callback_group.

        Note:
            Pretty sure that's not how it should be done.

        Args:
            fun: function to execute
            callback_group: callback group in which to execute the function

        Returns:
            future: holds the future results
            quardian: the guard condition object in the callback_group
        """
        if callback_group is None:
            callback_group = self.node.default_callback_group

        future = Future()

        def fun_with_future():
            if not future.done():  # guardian triggers several times, idk why.
                # so this condition protects this mess from repeting itselfs randomly
                result = fun()
                future.set_result(result)
                return result

        # guardian will execute fun_with_future inside of callback_group
        guardian = self.node.create_guard_condition(fun_with_future, callback_group)
        guardian.trigger()
        future.add_done_callback(
            lambda result: self.node.destroy_guard_condition(guardian)
        )
        # tmr = self.node.create_timer(0.00001, fun_with_future, callback_group)
        # future.add_done_callback(lambda result: self.node.destroy_timer(tmr))
        return future, guardian


def myMain(nodeClass, multiThreaded: bool = False, args=None):
    """Main function used through the motion stack.

    Args:
        nodeClass: Node to spin
        multiThreaded: using multithreaded executor or not
        args ():
    """
    rclpy.init()

    try:
        node = nodeClass()
    except KeyboardInterrupt:
        m = f"{TCOL.OKCYAN}KeyboardInterrupt intercepted, {TCOL.OKBLUE}shuting down. :){TCOL.ENDC}"
        print(m)
        return
    except ExternalShutdownException:
        m = f"{TCOL.OKCYAN}External Shutdown Command intercepted, {TCOL.OKBLUE}shuting down. :){TCOL.ENDC}"
        print(m)
        return
    except Exception as exception:
        m = f"Exception intercepted: {TCOL.FAIL}{traceback.format_exc()}{TCOL.ENDC}"
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
        m = f"{TCOL.OKCYAN}KeyboardInterrupt intercepted, {TCOL.OKBLUE}shuting down. :){TCOL.ENDC}"
        print(m)
        return
    except ExternalShutdownException:
        m = f"{TCOL.OKCYAN}External Shutdown Command intercepted, {TCOL.OKBLUE}shuting down. :){TCOL.ENDC}"
        print(m)
        return

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
