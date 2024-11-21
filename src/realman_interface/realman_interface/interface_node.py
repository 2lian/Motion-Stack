import threading
from typing import List

import numpy as np
import rclpy
from easy_robot_control.EliaNode import EliaNode, bcolors, list_cyanize, myMain
from rm_ros_interfaces.msg import Jointpos
from sensor_msgs.msg import JointState

float_formatter = "{:.2f}".format
np.set_printoptions(formatter={"float_kind": float_formatter})


class RealManInterface(EliaNode):
    def __init__(self):
        super().__init__("realman_interface")

        self.joint_names: List[str] = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ]
        self.follow: bool = False
        self.expand: int = 0
        self.dof: int = len(self.joint_names)

        self.latest_j_cmd = None
        self.last_cmd = None

        # logs
        self.pinfo(
            f"{bcolors.OKBLUE}Joint Names:{bcolors.ENDC} {list_cyanize(self.joint_names)}"
        )
        self.pinfo(
            f"{bcolors.OKBLUE}Follow: {bcolors.OKGREEN}{self.follow}{bcolors.ENDC}"
        )
        self.pinfo(
            f"{bcolors.OKBLUE}Expand: {bcolors.OKGREEN}{self.expand}{bcolors.ENDC}"
        )
        self.pinfo(f"{bcolors.OKBLUE}DOF: {bcolors.OKGREEN}{self.dof}{bcolors.ENDC}")

        # subscriber to /joint_commands
        self.subscription = self.create_subscription(
            JointState, "/joint_commands", self.joint_commands_callback, 10
        )
        self.subscription

        # publisher to RealMan driver using Jointpos message
        self.publisher = self.create_publisher(
            Jointpos, "/rm_driver/movej_canfd_cmd", 10
        )

        self.temp_joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.initial_joint_state_callback, 10
        )

        self.lock = threading.Lock()
        self.latest_joint_angles = [0.0 for _ in range(len(self.joint_names))]
        self.temp_joint_states = [0.0 for _ in range(len(self.joint_names))]

        self.pinfo(
            f"{bcolors.OKGREEN}RealMan Interface Node has been started.{bcolors.ENDC}"
        )

        # timer to publish commands to motor
        self.to_motor_TMR = self.create_timer(0.1, self.to_motor_TMRCBK)

    def initial_joint_state_callback(self, msg: JointState):
        if self.latest_j_cmd is not None:
            return

        with self.lock:
            for i, name in enumerate(msg.name):
                if name in self.joint_names:
                    index = self.joint_names.index(name)
                    if index < len(self.temp_joint_states):
                        self.temp_joint_states[index] = msg.position[i]
                    else:
                        self.pinfo(
                            f'{bcolors.WARNING}Joint index {index} for joint "{name}" out of range for joint_angles array.{bcolors.ENDC}'
                        )

            self.latest_j_cmd = self.temp_joint_states.copy()
            self.last_cmd = np.array(self.latest_j_cmd, dtype=float)
            # self.pinfo(
            #     f"{bcolors.OKGREEN}Initial joint states received: {self.latest_j_cmd}{bcolors.ENDC}"
            # )

        self.destroy_subscription(self.temp_joint_state_sub)

    def joint_commands_callback(self, msg: JointState):
        if not msg.position:
            return
        with self.lock:
            for i, name in enumerate(msg.name):
                if name in self.joint_names:
                    index = self.joint_names.index(name)
                    if index < len(self.latest_joint_angles):
                        self.latest_joint_angles[index] = msg.position[i]
                        # self.pinfo(
                        #     f"Updated {name} to {msg.position[i]} radians."
                        # )
                    else:
                        self.pinfo(
                            f'{bcolors.WARNING}Joint index {index} for joint "{name}" out of range for joint_angles array.{bcolors.ENDC}'
                        )

    def to_motor_TMRCBK(self):
        if self.latest_j_cmd is None:
            self.pwarn(
                f"{bcolors.WARNING}Initial joint states not received yet. Skipping publishing.{bcolors.ENDC}"
            )
            return

        with self.lock:
            last = self.last_cmd
            target = np.array(self.latest_joint_angles)
            next_cmd = np.clip(a=target, a_min=last - 0.1, a_max=last + 0.1)

        movej_msg = Jointpos()
        movej_msg.joint = list(next_cmd)
        movej_msg.follow = self.follow
        movej_msg.expand = float(self.expand)
        movej_msg.dof = self.dof
        self.publisher.publish(movej_msg)
        # self.pwarn(f"Published Jointpos message: {movej_msg}")

        with self.lock:
            self.last_cmd = next_cmd


def main(args=None):
    myMain(RealManInterface, multiThreaded=True)


if __name__ == "__main__":
    main()
