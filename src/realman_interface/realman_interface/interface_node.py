import threading
from typing import List

import rclpy
from easy_robot_control.EliaNode import EliaNode, bcolors, list_cyanize, myMain
from rm_ros_interfaces.msg import Movej
from sensor_msgs.msg import JointState


class RealManInterface(EliaNode):
    def __init__(self):
        super().__init__("realman_interface")

        # parameters
        self.joint_names: List[str] = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ]
        self.speed: int = 20  # speed percentage (0-100)
        self.block: bool = True  # blocking mode
        self.trajectory_connect: int = 0  # typically set to 0 for 7-DOF
        self.dof: int = len(self.joint_names)  # always 7 for 7-DOF arm

        self.pinfo(
            f"{bcolors.OKBLUE}Joint Names:{bcolors.ENDC} {list_cyanize(self.joint_names)}"
        )
        self.pinfo(
            f"{bcolors.OKBLUE}Speed: {bcolors.OKGREEN}{self.speed}%{bcolors.ENDC}"
        )
        self.pinfo(
            f"{bcolors.OKBLUE}Block: {bcolors.OKGREEN}{self.block}{bcolors.ENDC}"
        )
        self.pinfo(
            f"{bcolors.OKBLUE}Trajectory Connect: {bcolors.OKGREEN}{self.trajectory_connect}{bcolors.ENDC}"
        )
        self.pinfo(f"{bcolors.OKBLUE}DOF: {bcolors.OKGREEN}{self.dof}{bcolors.ENDC}")

        # subscriber to /joint_commands
        self.subscription = self.create_subscription(
            JointState, "/joint_commands", self.joint_commands_callback, 10
        )
        self.subscription

        # publisher to realguy driver using Movej message
        self.publisher = self.create_publisher(Movej, "/rm_driver/movej_cmd", 10)

        self.lock = threading.Lock()
        self.latest_joint_angles = [0.0 for _ in range(len(self.joint_names))]

        self.pinfo(
            f"{bcolors.OKGREEN}RealMan Interface Node has been started.{bcolors.ENDC}"
        )

    def joint_commands_callback(self, msg: JointState):
        with self.lock:
            for i, name in enumerate(msg.name):
                if name in self.joint_names:
                    index = self.joint_names.index(name)
                    if index < len(self.latest_joint_angles):
                        self.latest_joint_angles[index] = msg.position[i]
                        self.get_logger().debug(
                            f"Updated {name} to {msg.position[i]} radians."
                        )
                    else:
                        self.pinfo(
                            f'{bcolors.WARNING}Joint index {index} for joint "{name}" out of range for joint_angles array.{bcolors.ENDC}'
                        )

        # After updating joint angles, publish Movej message
        self.publish_movej()

    def publish_movej(self):
        movej_msg = Movej()
        with self.lock:
            # Since it's a 7-DOF arm, we can safely use all 7 joints
            movej_msg.joint = self.latest_joint_angles.copy()
        movej_msg.speed = self.speed
        movej_msg.block = self.block
        movej_msg.dof = self.dof
        movej_msg.trajectory_connect = self.trajectory_connect  # Only for 7-DOF

        self.publisher.publish(movej_msg)
        self.get_logger().debug(f"Published Movej message: {movej_msg}")


def main(args=None):
    myMain(RealManInterface, multiThreaded=True)


if __name__ == "__main__":
    main()
