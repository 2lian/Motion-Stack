"""Holds ros2 communication interface data.

It provides the names and types of every interface (topics, services, actions) used by the motion stack. So no need to remember the right name with the right spelling, import this and use communication.lvl1.output.motor_command.name"""

from collections import namedtuple

from motion_stack_msgs.srv import ReturnJointState
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty as SrvEmpty

Interf = namedtuple(
    "Interf", ["type", "name"]
)  #: Ros2 interface class with type and name


class lvl1:

    alive = Interf(SrvEmpty, "joint_alive")

    class output:
        motor_command = Interf(JointState, "joint_commands")
        ik_command = Interf(JointState, "joint_read")
        advertise = Interf(ReturnJointState, "advertise_joints")

    class input:
        motor_sensor = Interf(JointState, "joint_states")
        ik_command = Interf(JointState, "joint_set")

