from typing import Tuple
from motion_stack_msgs.srv import SendJointState
from rclpy.node import Service, Timer
from motion_stack.ros2.base_node.lvl1 import Lvl1Node

from motion_stack.api.injection.offsetter import OffsetterLvl0
from motion_stack.ros2.utils.joint_state import ros2js


def _set_offsetSRVCBK(
    offsetter: OffsetterLvl0,
    req: SendJointState.Request,
    res: SendJointState.Response,
) -> SendJointState.Response:
    if len(req.js.name) < 1:
        return res
    states: List[JState] = ros2js(req.js)  # type: ignore
    succ, disp = offsetter.apply_offset(states)
    res.success = succ
    res.message = disp
    return res


def setup_lvl0_offsetter(
    node: Lvl1Node, angle_recovery_path: str, offset_path: str
) -> Tuple[Timer, Service]:
    core = node.lvl1
    offsetter = OffsetterLvl0(core, angle_recovery_path, offset_path)

    tmr = node.create_timer(5, offsetter.save_angle_recovery)
    srv = node.create_service(
        SendJointState, "set_offset", lambda *args: _set_offsetSRVCBK(offsetter, *args)
    )
    return tmr, srv
