from collections import namedtuple
from dataclasses import dataclass

from motion_stack_msgs.srv import ReturnJointState
from sensor_msgs.msg import JointState

Interf = namedtuple(
    "Interf", ["type", "name"]
)  #: Ros2 interface class with type and name


class lvl1:
    ...

    class output:
        motor_command = Interf(JointState, "joint_command")
        ik_command = Interf(JointState, "joint_read")
        advertise = Interf(ReturnJointState, "advertise_joints")

    class input:
        motor_sensor = Interf(JointState, "joint_state")
        ik_command = Interf(JointState, "joint_set")
