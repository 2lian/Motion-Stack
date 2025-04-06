"""Python core of the motion stack.

There is no ROS2 code in here, only python "nodes". Those the are skeltons to be run by any runtime, either python, ROS2 or else.

This allows for:

.. hlist::
    :columns: 1

    - untangling from ROS2
    - testing through pytest
    - multiple ROS versions and runtime

Authors: 
    .. hlist::
        :columns: 1

        * Elian NEPPEL
        * Shamistan KARIMOV

"""

import matplotlib
import numpy as np
import scipy

matplotlib.use("Agg")  # fix for when there is no display
__version__ = "0.0.1"
scipy.randn = np.random


import roboticstoolbox.tools.urdf.urdf as bad

from .rtb_fix import fixed_urdf as fix

bad.URDF.__init__ = fix.URDF.__init__
bad.URDF._recursive_axis_definition = fix.URDF._recursive_axis_definition
bad.URDF.finalize_linking = fix.URDF.finalize_linking
