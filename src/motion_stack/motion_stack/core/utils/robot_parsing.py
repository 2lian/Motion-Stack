"""Uses rtb to parse the robot URDF data"""

import re
import tempfile
from typing import Any, Callable, Iterable, List, Optional, Sequence, Set, Tuple, Union

import numpy as np
import roboticstoolbox as rtb
from nptyping import NDArray
from roboticstoolbox.robot import Robot
from roboticstoolbox.robot.ET import ET, SE3
from roboticstoolbox.robot.ETS import ETS
from roboticstoolbox.robot.Link import Link
from roboticstoolbox.tools import URDF
from roboticstoolbox.tools.urdf.urdf import Joint as RTBJoint


def replace_incompatible_char_ros2(string_to_correct: str) -> str:
    """Sanitizes strings for use by ros2.

    replace character that cannot be used for Ros2 Topics by _
    inserts "WARN" in front if topic starts with incompatible char
    """
    corrected_string = string_to_correct
    corrected_string = re.sub(r"[^a-zA-Z0-9/~]", "_", corrected_string)
    corrected_string = re.sub(r"/(?=[^a-zA-Z])", "/WARN", corrected_string)
    if string_to_correct[0].isdigit():
        corrected_string = "WARN" + string_to_correct
    return corrected_string


def get_limit(joint: RTBJoint) -> Tuple[float, float]:
    """Returns the limits of a joint from rtb parsing"""
    try:
        lower: float = joint.limit.lower
        upper: float = joint.limit.upper
    except AttributeError:
        lower: float = -np.inf
        upper: float = np.inf
    return lower, upper


def load_set_urdf_raw(
    urdf: str,
    end_effector_name: Optional[Union[str, int]] = None,
    start_effector_name: Optional[str] = None,
) -> Tuple[Robot, ETS, List[str], List[RTBJoint], Optional[Link]]:
    """Enables calling load_set_urdf with the full urdf string instead of the path
    """
    with tempfile.NamedTemporaryFile(mode="w+", delete=True) as temp_file:
        temp_file.write(urdf)
        temp_file.flush()  # Ensure the data is written to disk

        return load_set_urdf(temp_file.name, end_effector_name, start_effector_name)


def load_set_urdf(
    urdf_path: str,
    end_effector_name: Optional[Union[str, int]] = None,
    start_effector_name: Optional[str] = None,
) -> Tuple[Robot, ETS, List[str], List[RTBJoint], Optional[Link]]:
    """I am so sorry. This works to parse the urdf I don't have time to explain

    Note:
        will change, I hate this

        This is terrible and still in the code

    Args:
        urdf_path:
        end_effector_name:

    Returns:

    """
    # model = rtb.Robot.URDF_read(file_path=urdf_path, tld = get_package_share_directory("rviz_basic"))
    full_model = rtb.Robot.URDF(file_path=urdf_path)
    l = full_model.links
    links, name, urdf_string, urdf_filepath = rtb.Robot.URDF_read(file_path=urdf_path)
    joints_objects = URDF.loadstr(urdf_string, urdf_filepath).joints

    if end_effector_name is None:
        exctracted_chain = full_model.ets().copy()
        joint_names = [j.name for j in joints_objects if j.joint_type != "fixed"]
        joint_index = list(range(len(joint_names)))

        for et in exctracted_chain:
            et: ET
            if et.qlim is not None:
                if (
                    (et.qlim[0] == 0.0 and et.qlim[1] == 0.0)
                    or et.qlim[0] is None
                    or et.qlim[1] is None
                ):
                    et.qlim = None
        return full_model, exctracted_chain, joint_names, joints_objects, None

    if start_effector_name is not None:
        simil_link_names = [x for x in l if x.name == start_effector_name]
        if simil_link_names:
            start_link = [x for x in l if x.name == start_effector_name][0]
        else:
            start_link = l[0]
    else:
        start_link = None

    if type(end_effector_name) is int:  # picks Nth longest segment
        segments = full_model.segments()
        if start_link is not None:
            segments = [seg for seg in segments if start_link in seg]
        lengths: NDArray = np.array([len(s) for s in segments], dtype=int)
        n: int = end_effector_name
        nth_longest_index: int = np.argsort(-lengths)[n]
        nth_longest_segment: List[Optional[Link]] = segments[nth_longest_index]
        end_link: Link = nth_longest_segment[-1]
    elif start_effector_name == end_effector_name:
        end_link = start_link
    else:
        end_links = [x for x in l if x.name == end_effector_name]
        if not end_links:
            raise ValueError(f"{end_effector_name=} not in {urdf_path=}")
        end_link = end_links[0]

    # print(start_link, end_link)
    exctracted_chain: ETS = full_model.ets(
        start=start_link,
        end=end_link,
    ).copy()
    for et in exctracted_chain:
        et: ET
        if et.qlim is not None:
            if (
                (et.qlim[0] == 0.0 and et.qlim[1] == 0.0)
                or et.qlim[0] is None
                or et.qlim[1] is None
            ):
                et.qlim = None

    # exctracts all joints
    link: Link = end_link.copy()
    joint_index = []
    while link.children != start_effector_name and link.parent is not None:
        link: Link
        parent: Link = link.parent.copy()
        for ind, joint in enumerate(joints_objects):
            if joint.parent == parent.name and joint.child == link.name:
                if joint.joint_type != "fixed":  # skips rigid joints
                    joint_index = [ind] + joint_index  # reverse fill
                break
        if link.name == start_effector_name:
            break
        link = parent

    joints_objects = [joints_objects[j] for j in joint_index]
    joint_names = [jo.name for jo in joints_objects]

    # correct numbering by starting at 1 if not: bug
    counter = 0
    for et in exctracted_chain:
        et: ET
        if et.isjoint:
            et.jindex = counter
            counter += 1

    return full_model, exctracted_chain, joint_names, joints_objects, end_link
