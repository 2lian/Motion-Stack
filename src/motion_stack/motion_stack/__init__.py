"""Motion Stack package.

.. Author: 
    Elian NEPPEL

.. Coauthor: 
    Shamistan KARIMOV
    Ashutosh MISHRA

.. Laboratory: 
    Space Robotics Lab, Tohoku University

.. Maintainer: 
    Elian NEPPEL

.. Note:
    You made a module? add yourself as the author!

"""

import matplotlib
import numpy as np
import scipy

matplotlib.use("Agg")  # fix for when there is no display

__version__ = "0.0.1"

scipy.randn = np.random

import easy_robot_control.my_rtb_fix.fixed_urdf as fix
import roboticstoolbox.tools.urdf.urdf as bad

bad.URDF.__init__ = fix.URDF.__init__
bad.URDF._recursive_axis_definition = fix.URDF._recursive_axis_definition
bad.URDF.finalize_linking = fix.URDF.finalize_linking
