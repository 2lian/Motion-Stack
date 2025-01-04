from rclpy.time import Time as RosTime 
from ...core.utils.time import Time

def ros_to_time(time: RosTime)->Time:
    return Time(nano=time.nanoseconds)

def time_to_ros(time: Time)-> RosTime:
    return RosTime(nanoseconds=time.nano())
