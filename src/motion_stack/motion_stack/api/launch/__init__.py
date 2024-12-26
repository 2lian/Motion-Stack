"""Motion Stack all"""
import numpy as np
import scipy

__version__ = "0.0.1"

scipy.randn = np.random

import roboticstoolbox.tools.urdf.urdf as bad

import easy_robot_control.my_rtb_fix.fixed_urdf as fix

bad.URDF.__init__ = fix.URDF.__init__
bad.URDF._recursive_axis_definition = fix.URDF._recursive_axis_definition
bad.URDF.finalize_linking = fix.URDF.finalize_linking
