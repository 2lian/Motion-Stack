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
if MOONBOT_PC_NUMBER in [None, "", "None"]:
    MOONBOT_PC_NUMBER = "1"
# num = MOONBOT_PC_NUMBER
JOINTS: List[str] = [
    f"leg{num}{suffix}"
    for num in range(20)
    for suffix in [
        "base_link-link2",
        "link2-link3",
        "link3-link4",
        "link4-link5",
        "link5-link6",
        "link6-link7",
        "link7-link8",
        "grip1",
        "grip2",
    ]
]
MOTORS: List[str] = [
    f"base_link{n+1}" for n in range(10)
]

# lvl0
#   \  /   #
#    \/    #
name_remap: Dict[str, str] = {}
for k in range(20):
    name_remap.update({
        JOINTS[0 + k*9]: MOTORS[1],
        JOINTS[1 + k*9]: MOTORS[2],
        JOINTS[2 + k*9]: MOTORS[3],
        JOINTS[3 + k*9]: MOTORS[4],
        JOINTS[4 + k*9]: MOTORS[5],
        JOINTS[5 + k*9]: MOTORS[6],
        JOINTS[6 + k*9]: MOTORS[7],
        JOINTS[7 + k*9]: MOTORS[0],
        JOINTS[8 + k*9]: MOTORS[8],
    })

name_remap: Dict[str, str] = {}
raw_speed = 500  # raw
duration = 30  # sec
start_pos = 0  # rad
end_pos = 1.6473  # rad
real_speed = (end_pos - start_pos) / duration  # rad/s
real2raw = raw_speed / real_speed

TC_OFFSET: float = 0
TC_UPPER: float = np.inf
TC_LOWER: float = -np.inf
TC_GAIN: float = real2raw
TC_GAIN: float = 0

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
S_GAIN: float = 0
lvl0_sensor_shaping: StateMap = {
    x: Shaper(
        velocity=lambda x: (x * S_GAIN + S_OFFSET),
    )
    for x in JOINTS
}
#    /\    #
#   /  \   #
# lvl0

map_lvl0 = StateRemapper(
    name_map=name_remap,
    state_map=lvl0_cmd_shaping,
    unstate_map=lvl0_sensor_shaping,
)
map_lvl2 = StateRemapper(
    name_map={},
    unname_map={},
    state_map={},
    unstate_map={},
)
