"""Uses rtb to parse the robot URDF data"""

from typing import Tuple

import numpy as np
from roboticstoolbox.tools.urdf.urdf import Joint as RTBJoint


def get_limit(joint: RTBJoint) -> Tuple[float, float]:
    """Returns the limits of a joint from rtb parsing"""
    try:
        lower: float = joint.limit.lower
        upper: float = joint.limit.upper
    except AttributeError:
        lower: float = -np.inf
        upper: float = np.inf
    return lower, upper
