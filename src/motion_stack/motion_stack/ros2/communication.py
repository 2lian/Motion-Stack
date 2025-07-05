"""Holds ros2 communication interface data.

It provides the names and types of every interface (topics, services, actions) used by the motion stack. So no need to remember the right name with the right spelling, import this and use communication.lvl1.output.joint_state.name to get ``joint_read``
"""

from typing import NamedTuple, Type

from geometry_msgs.msg import Transform
from motion_stack_msgs.srv import ReturnJointState
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSPresetProfiles, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty as SrvEmpty


def limb_ns(limb_number: int) -> str:
    """

    Args:
        limb_number: Number of the limb

    Returns:
        Namespace of the limb.

    """
    return f"leg{limb_number}"


# use this for system with few robot, good network
qos_reliable = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)

# use this when network slows down
qos_lossy = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
    lifespan=Duration(seconds=0.50), # messages older than this will be discarded
    deadline=Duration(seconds=2), # Triggers event (callback) if exceeded with no messages on THIS topic
    liveliness_lease_duration=Duration(seconds=2), # Triggers event (callback) if exceeded with no messages on ANY topic
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
