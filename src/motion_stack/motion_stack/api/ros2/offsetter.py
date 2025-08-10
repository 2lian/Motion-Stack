from typing import Optional, Tuple

from motion_stack.core.utils.time import Time
from motion_stack.ros2.utils.conversion import ros_to_time
from motion_stack_msgs.srv import SendJointState
from rclpy.node import Service, Timer

from motion_stack.api.injection.offsetter import OffsetterLvl0
from motion_stack.ros2.base_node.lvl1 import Lvl1Node
from motion_stack.ros2.utils.joint_state import ros2js

__LAST_STAMP: Optional[Time] = None

def _set_offsetSRVCBK(
    offsetter: OffsetterLvl0,
    req: SendJointState.Request,
    res: SendJointState.Response,
) -> SendJointState.Response:
    global __LAST_STAMP
    req_now = Time(nano=req.js.header.stamp.sec*int(1e9)*req.js.header.stamp.nanosec)
    print(f"Request time: {req_now.nano()}")
    if __LAST_STAMP is not None and req_now.nano() !=0:
        if req_now == __LAST_STAMP:
            print(f"Duplicate offset request with time {req_now.nano()}")
            return res

    if len(req.js.name) < 1:
        return res
    __LAST_STAMP = req_now
    states: List[JState] = ros2js(req.js)  # type: ignore
    succ, disp = offsetter.apply_offset(states)
    res.success = succ
    res.message = disp
    return res


def setup_lvl0_offsetter(
    node: Lvl1Node,
    angle_recovery_path: Optional[str] = None,
    offset_path: Optional[str] = None,
) -> Tuple[Timer, Service]:
    core = node.core
    offsetter = OffsetterLvl0(core, angle_recovery_path, offset_path)

    tmr = node.create_timer(5, offsetter.save_angle_recovery)
    srv = node.create_service(
        SendJointState, "set_offset", lambda *args: _set_offsetSRVCBK(offsetter, *args)
    )
    return tmr, srv
