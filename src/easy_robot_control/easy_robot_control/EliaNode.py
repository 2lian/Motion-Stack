"""
I this adds functionalities to the default ros2 Node object

Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

import time
from typing import Any, Optional, Tuple
from numpy.typing import NDArray

# import rclpy
import numpy as np
import quaternion as qt
from rclpy.callback_groups import CallbackGroup
from rclpy.client import Client
from rclpy.clock import Clock
from rclpy.task import Future
from rclpy.node import Node, Union, List
from rclpy.time import Duration, Time
from geometry_msgs.msg import TransformStamped, Transform
from std_srvs.srv import Empty


def future_list_complete(future_list: List[Future]) -> np.bool_:
    """Returns True is all futures in the input list are done.

    Args:
        future_list: a list of futures

    Returns:
        True if all futures are done
    """
    return np.all([f.done() for f in future_list])


class Bcolors:
    def __init__(self) -> None:
        self.HEADER = "\033[95m"
        self.OKBLUE = "\033[94m"
        self.OKCYAN = "\033[96m"
        self.OKGREEN = "\033[92m"
        self.WARNING = "\033[93m"
        self.FAIL = "\033[91m"
        self.ENDC = "\033[0m"
        self.BOLD = "\033[1m"
        self.UNDERLINE = "\033[4m"


bcolors = Bcolors()


class CustomRate:
    def __init__(self, parent: Node, frequency: float) -> None:
        self.frequency = frequency
        self.parent = parent
        self.last_clock = self.parent.get_clock().now()
        self.deltaTime = Time(
            seconds=1 / self.frequency,  # type: ignore
            clock_type=self.parent.get_clock().clock_type,
        )

    def sleep(self) -> None:
        second1, nanosec1 = self.last_clock.seconds_nanoseconds()
        second2, nanosec2 = self.deltaTime.seconds_nanoseconds()
        next_time = Time(
            seconds=second1 + second2,
            nanoseconds=nanosec1 + nanosec2,
            clock_type=self.parent.get_clock().clock_type,
        )
        self.last_clock = next_time
        self.parent.get_clock().sleep_until(next_time)

    def destroy(self) -> None:
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

    def create_rate(self, frequency: float, clock: Clock = None) -> CustomRate:
        return CustomRate(self, frequency)
