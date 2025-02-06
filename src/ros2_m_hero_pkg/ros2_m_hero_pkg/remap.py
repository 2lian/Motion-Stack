from os import environ
from typing import Callable, Dict, List

import numpy as np
from motion_stack.api.injection.remapper import Shaper, StateMap, StateRemapper

URDFJointName = str
MOONBOT_PC_NUMBER = str(environ.get("M_LEG"))  # leg number saved on lattepanda
if MOONBOT_PC_NUMBER in [None, "", "None"]:
    MOONBOT_PC_NUMBER = "1"
# num = MOONBOT_PC_NUMBER
JOINTS: List[str] = []
for num in range(20):
    JOINTS += [
        f"leg{num}{suffix}"
        for suffix in [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
            "grip1",
            "grip2",
            # "base_link_link2",
            # "link2_link3",
            # "link3_link4",
            # "link4_link5",
            # "link5_link6",
            # "link6_link7",
            # "link7_link8",
            # "grip1",
            # "grip2",
        ]
    ] + [
        f"wheel{num}_{suffix}_joint"
        for suffix in [
            "left",
            "right",
        ]
    ]

MOTORS: List[str] = [f"base_link{n+1}_joint" for n in range(10)]

# lvl0
#   \  /   #
#    \/    #
name_remap: Dict[str, str] = {}
jl = 11
for k in range(20):
    name_remap.update(
        {
            JOINTS[0 + k * jl]: MOTORS[1],
            JOINTS[1 + k * jl]: MOTORS[2],
            JOINTS[2 + k * jl]: MOTORS[3],
            JOINTS[3 + k * jl]: MOTORS[4],
            JOINTS[4 + k * jl]: MOTORS[5],
            JOINTS[5 + k * jl]: MOTORS[6],
            JOINTS[6 + k * jl]: MOTORS[7],
            JOINTS[7 + k * jl]: MOTORS[0],
            JOINTS[8 + k * jl]: MOTORS[8],
            JOINTS[9 + k * jl]: MOTORS[0],
            JOINTS[10 + k * jl]: MOTORS[1],
        }
    )

# name_remap: Dict[str, str] = {}
raw_speed = 500  # raw
duration = 30  # sec
start_pos = 0  # rad
end_pos = 1.6473  # rad
real_speed = (end_pos - start_pos) / duration  # rad/s
real2raw = raw_speed / real_speed

raw_speed_grip = 500  # raw
duration_grip = 10  # sec
start_pos_grip = -0.01257620875411004  # meters
end_pos_grip = 0.003456390322457868  # meters
real_speed_grip = (end_pos_grip - start_pos_grip) / duration_grip  # m/s
real2raw_grip = raw_speed_grip / real_speed_grip

TC_OFFSET: float = 0
TC_UPPER: float = np.inf
TC_LOWER: float = -np.inf
TC_GAIN: float = real2raw

TC_OFFSET_GRIP: float = 0
TC_UPPER_GRIP: float = np.inf
TC_LOWER_GRIP: float = -np.inf
TC_GAIN_GRIP: float = -real2raw_grip

lvl0_cmd_shaping: StateMap = {
    x: Shaper(
        velocity=lambda x: np.clip(x + TC_OFFSET, a_min=TC_LOWER, a_max=TC_UPPER)
        * TC_GAIN
    )
    for x in JOINTS
}

for k in range(20):
    for x in range (3):
        lvl0_cmd_shaping[f"leg{k}grip{x}"] = Shaper(
            velocity=lambda x: np.clip(
                x + TC_OFFSET_GRIP, a_min=TC_LOWER_GRIP, a_max=TC_UPPER_GRIP
            )
            * TC_GAIN_GRIP
        )
start_raw: int = 2721
end_raw: int = 8456
measured_raw: int = end_raw - start_raw
measured_rad: float = np.pi * 2 / 8
raw2rad = measured_rad / measured_raw

# grip
# start - 64.5mm, end - 22.8
start_grip_raw: int = 23
end_grip_raw: int = 5221
measured_grip_raw: int = end_grip_raw - start_grip_raw
measured_grip_m: float = 0.0417 / 2
raw2rad_grip = measured_grip_m / measured_grip_raw

S_OFFSET: float = 0.0
S_GAIN: float = raw2rad / 1
S_OFFSET_GRIP: float = 0.0
S_GAIN_GRIP: float = -raw2rad_grip / 1

lvl0_sensor_shaping: StateMap = {
    x: Shaper(
        position=lambda x: (x * S_GAIN + S_OFFSET), velocity=lambda x: x / TC_GAIN
    )
    for x in JOINTS
}

for k in range(20):
    for x in range (3):
        lvl0_sensor_shaping[f"leg{k}grip{x}"] = Shaper(
            position=lambda x: (x * S_GAIN_GRIP + S_OFFSET_GRIP),
            velocity=lambda x: x / TC_GAIN_GRIP,
        )


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
