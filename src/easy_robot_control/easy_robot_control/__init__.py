import numpy as np
import scipy

# from quaternion import quaternion as __Quat
# import quaternion
#
#
# class q(__Quat):
#     def __init__(self, *args, **kwargs) -> None:
#         super().__init__(*args, **kwargs)
#
#     def __repr__(self) -> str:
#         o = quaternion.as_float_array(self)
#         return f"q{o}"
#
#
# quaternion.quaternion = q


scipy.randn = np.random

# from roboticstoolbox import *
import roboticstoolbox.tools.urdf.urdf as bad

import easy_robot_control.my_rtb_fix.fixed_urdf as fix

bad.URDF.__init__ = fix.URDF.__init__
bad.URDF._recursive_axis_definition = fix.URDF._recursive_axis_definition
bad.URDF.finalize_linking = fix.URDF.finalize_linking
