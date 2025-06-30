from typing import Callable, Union

import numpy as np
from geometry_msgs.msg import Transform
from rclpy.node import Node
from rclpy.time import Duration as RosDuration
from rclpy.time import Time as RosTime

from motion_stack.core.utils.math import Quaternion, qt, qt_normalize
from motion_stack.core.utils.pose import Pose

from ...core.utils.time import Time


def ros_to_time(time: Union[RosTime, RosDuration]) -> Time:
    return Time(nano=time.nanoseconds)


def ros_now(node: Node) -> Time:
    return ros_to_time(node.get_clock().now())


def time_to_ros(time: Time) -> RosTime:
    return RosTime(nanoseconds=time.nano())


def delta_time_callable(node: Node) -> Callable[[], Time]:
    """Creates a function that returns the elapsed time since last call.
    First call returns 0
    """
    prev = None

    def dt() -> Time:
        nonlocal prev
        now = ros_now(node)
        if prev is None:
            prev = now
        delta_time = now - prev
        prev = now
        return delta_time

    return dt


def transform_to_pose(tf: Transform, time: Time) -> Pose:
    xyz = np.array([tf.translation.x, tf.translation.y, tf.translation.z], dtype=float)
    quat = Quaternion()
    quat.w = tf.rotation.w
    quat.x = tf.rotation.x
    quat.y = tf.rotation.y
    quat.z = tf.rotation.z
    quat = qt_normalize(quat)
    return Pose(time, xyz, quat)


def pose_to_transform(
    pose: Pose,
    sendNone: bool = False,
) -> Transform:
    coord = pose.xyz
    quat = pose.quat
    if coord is None:
        if sendNone:
            xyz = np.array([np.nan] * 3, dtype=float)
        else:
            xyz = np.array([0.0, 0.0, 0.0], dtype=float)
    elif isinstance(coord, list):
        xyz = np.array(coord, dtype=float)
    else:
        xyz = coord.astype(float)
    if quat is None:
        if sendNone:
            rot = qt.from_float_array(np.array([np.nan] * 4, dtype=float))
        else:
            rot = qt.one.copy()
    else:
        rot = quat

    assert isinstance(xyz, np.ndarray)
    assert isinstance(rot, Quaternion)
    assert xyz.shape == (3,)
    assert xyz.dtype == np.float64

    rot = qt_normalize(rot)

    tf = Transform()
    tf.translation.x = xyz[0]
    tf.translation.y = xyz[1]
    tf.translation.z = xyz[2]
    tf.rotation.w = rot.w
    tf.rotation.x = rot.x
    tf.rotation.y = rot.y
    tf.rotation.z = rot.z
    return tf
