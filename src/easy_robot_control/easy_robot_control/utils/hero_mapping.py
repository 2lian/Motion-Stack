from os import environ
from typing import Callable, Dict, List

import numpy as np

from easy_robot_control.utils.state_remaper import (
    Shaper,
    StateMap,
    StateRemapper,
)

URDFJointName = str
MOONBOT_PC_NUMBER = str(environ.get("M_LEG"))  # leg number saved on lattepanda
if MOONBOT_PC_NUMBER is None:
    MOONBOT_PC_NUMBER = "1"
num = MOONBOT_PC_NUMBER
JOINTS: List[URDFJointName] = [
    f"leg{num}base_link-link2",
    f"leg{num}link2-link3",
    f"leg{num}link3-link4",
    f"leg{num}link4-link5",
    f"leg{num}link5-link6",
    f"leg{num}link6-link7",
    f"leg{num}link7-link8",
    f"leg{num}grip1",
    f"leg{num}grip2",
]

# lvl0
#   \  /   #
#    \/    #
raw_speed = 500  # raw
duration = 30  # sec
start_pos = 0  # rad
end_pos = 1.6473  # rad
real_speed = (end_pos - start_pos) / duration  # rad/s
real2raw = raw_speed / real_speed

TC_OFFSET: float = 0.0
TC_UPPER: float = np.inf
TC_LOWER: float = -np.inf
TC_GAIN: float = real2raw
lvl0_cmd_shaping: StateMap = {
    x: Shaper(
        position=lambda x: np.clip(x + TC_OFFSET, a_min=TC_LOWER, a_max=TC_UPPER)
        * TC_GAIN
    )
    for x in JOINTS
}
start_raw: int = 2721
end_raw: int = 8456
measured_raw: int = end_raw - start_raw
measured_rad: float = np.pi * 2 / 8
raw2rad = measured_rad / measured_raw

S_OFFSET: float = 0.0
S_GAIN: float = raw2rad / 1
lvl0_sensor_shaping: StateMap = {
    x: Shaper(
        position=lambda x: (x * S_GAIN + S_OFFSET),
    )
    for x in JOINTS
}
#    /\    #
#   /  \   #
# lvl0

map_lvl0 = StateRemapper(
    name_map={},
    unname_map={},
    state_map=lvl0_cmd_shaping,
    unstate_map=lvl0_sensor_shaping,
)
map_lvl2 = StateRemapper(
    name_map={},
    unname_map={},
    state_map={},
    unstate_map={},
)
