from typing import Any, Callable, Dict, List, Optional

import zenoh

from motion_stack.zenoh.utils.auto_session import auto_session

from ...core.lvl1_joint import JointCore
from ...core.utils.joint_state import JState
from ...core.utils.time import Time

def _encode_JS(js = JState) -> byte:

