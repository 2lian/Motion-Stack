"""Holds ros2 communication interface data.

It provides the names and types of every interface (topics, services, actions) used by the motion stack. So no need to remember the right name with the right spelling, import this and use communication.lvl1.output.joint_state.name to get ``joint_read`` """

from typing import NamedTuple, Type

from geometry_msgs.msg import Transform
from motion_stack_msgs.srv import ReturnJointState
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty as SrvEmpty
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def limb_ns(limb_number: int) -> str:
    """

    Args:
        limb_number: Number of the limb

    Returns:
        Namespace of the limb.

    """
    return f"leg{limb_number}"

qos_reliable = QoSProfile( # use this for system with few robot, good network
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

qos_lossy = QoSProfile( # use this when network slows down
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

DEFAULT_QOS = qos_lossy

class Interf(NamedTuple):
    """Ros2 interface class with type and name"""

    type: Type
    name: str
    qos: QoSProfile = DEFAULT_QOS


class lvl1:

    alive = Interf(SrvEmpty, "joint_alive")

    class output:
        motor_command = Interf(JointState, "joint_commands", qos_reliable)
        joint_state = Interf(JointState, "joint_read")
        advertise = Interf(ReturnJointState, "advertise_joints")

    class input:
        motor_sensor = Interf(JointState, "joint_states")
        joint_target = Interf(JointState, "joint_set")


class lvl2:

    alive = Interf(SrvEmpty, "ik_alive")

    class output:
        joint_target = lvl1.input.joint_target
        tip_pos = Interf(Transform, "tip_pos")

    class input:
        joint_state = lvl1.output.joint_state
        set_ik = Interf(Transform, "set_ik_target")
