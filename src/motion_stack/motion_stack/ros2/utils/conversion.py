from rclpy.time import Time as RosTime 
from ...core.utils.joint_state import JState, Time

def ros_to_time(time: RosTime):
    return Time(nano=time.nanoseconds)
