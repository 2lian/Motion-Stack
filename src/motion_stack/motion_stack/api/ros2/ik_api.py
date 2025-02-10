"""ROS2 API to send/receive end-effector IK command / FK state to lvl2 and syncronise multiple limbs."""
from typing import Callable, Dict, List, Optional, Set, Tuple, Type

import numpy as np
from geometry_msgs.msg import Transform
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState

from ...core.utils.joint_state import JState, impose_state
from ...core.utils.pose import Pose, XyzQuat
from ...ros2 import communication as comms
from ...ros2.utils.conversion import (
    pose_to_transform,
    ros_now,
    ros_to_time,
    time_to_ros,
    transform_to_pose,
)
from ..ik_syncer import IkSyncer, MultiPose


class IkHandler:
    def __init__(self, node: Node, limb_number: int) -> None:
        self.limb_number: int = limb_number
        self.new_tip_cbk: List[Callable[["IkHandler"],]] = []
        self.ready: Future = Future()

        self._node = node
        self._ee_pose: Optional[Pose] = None
        self._readSUB = node.create_subscription(
            comms.lvl2.output.tip_pos.type,
            f"{comms.limb_ns(self.limb_number)}/{comms.lvl2.output.tip_pos.name}",
            self._update_ee_poseCBK,
            10,
        )
        self._setPUB = node.create_publisher(
            comms.lvl2.input.set_ik.type,
            f"{comms.limb_ns(self.limb_number)}/{comms.lvl2.input.set_ik.name}",
            10,
        )
        self.ready: Future
        self.ready_up()

    @property
    def ee_pose(self) -> Pose:
        """End effector pose"""
        if self._ee_pose is None:
            raise AttributeError("End-Effector pose is yet unknowned")
        return self._ee_pose.copy()

    def ready_up(self) -> Future:
        """
        Returns:
            - Future done the next time end effector pose is received
        """
        self.ready.cancel()
        self.ready = Future()
        return self.ready

    def _update_ee_poseCBK(self, msg):
        assert isinstance(msg, comms.lvl2.output.tip_pos.type)
        self._update_ee(transform_to_pose(msg, time=ros_now(self._node)))

    def _update_ee(self, pose: Pose):
        self._ee_pose = pose
        if not self.ready.done():
            self.ready.set_result(True)
        for f in self.new_tip_cbk:
            f(self)

    def send(self, target_pose: Pose):
        """Sends ik target command to lvl2."""
        tf = pose_to_transform(target_pose)
        self._setPUB.publish(tf)


class IkSyncerRos(IkSyncer):
    """Controls and syncronises several joints, safely executing trajectory to a target.

    Important:
        This class is a ROS2 implementation of the base class: :py:class:`.api.joint_syncer.JointSyncer`. Refere to it for documentation.

    Args:
        joint_handlers: ROS2 objects handling joint communications of several limbs.
    """

    def __init__(
        self,
        ik_handlers: List[IkHandler],
        interpolation_delta: XyzQuat[float, float] = XyzQuat(40, np.deg2rad(4)),
        on_target_delta: XyzQuat[float, float] = XyzQuat(40, np.deg2rad(4)),
    ) -> None:
        super().__init__(interpolation_delta, on_target_delta)
        self._ik_handlers: Dict[int, IkHandler] = {
            ih.limb_number: ih for ih in ik_handlers
        }

    def execute(self):
        """Executes one step of the task/trajectory.

        This must be called frequently in a ros Timer or something else of your liking.
        """
        return super().execute()

    @property
    def sensor(self) -> MultiPose:
        """
        Important:
            This class is a ROS2 implementation of the base class: :py:class:`.api.joint_syncer.JointSyncer`. Refere to it for documentation.
        """
        return {k: v.ee_pose for k, v in self._ik_handlers.items()}

    def send_to_lvl2(self, ee_targets: MultiPose):
        """
        Important:
            This class is a ROS2 implementation of the base class: :py:class:`.api.joint_syncer.JointSyncer`. Refere to it for documentation.
        """
        for limb_number, target in ee_targets.items():
            self._ik_handlers[limb_number].send(target)

    @property
    def FutureT(self) -> Type[Future]:
        """
        Important:
            This class is a ROS2 implementation of the base class: :py:class:`.api.joint_syncer.JointSyncer`. Refere to it for documentation.
        """
        return Future
