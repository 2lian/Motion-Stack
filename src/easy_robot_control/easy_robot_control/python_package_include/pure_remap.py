import pytest
import numpy as np
from typing import Callable, Dict, List

URDFJointName = str
CommandJointName = str
SensorJointName = str
CommandTopicName = str
SensorTopicName = str

MOTORS: List[CommandTopicName] = [
    f"/maxon/canopen_motor/base_link{n+1}_joint_velocity_controller/command"
    for n in range(10)
]
JOINTS: List[URDFJointName] = [
    "base_link-link2",
    "link2-link3",
    "link3-link4",
    "link4-link5",
    "link5-link6",
    "link6-link7",
    "link7-link8",
]

# Topic Command
#   \  /   #
#    \/    #
# Angles (or speed if speed mode) commands for those joints in the URDF
# will be published onto those individual topics
remap_topic_com: Dict[URDFJointName, CommandTopicName] = {
    "This_joint_does_not_exist_in_the_URDF": "This_line_will_do_nothing",
    JOINTS[0]: MOTORS[1],
    JOINTS[1]: MOTORS[2],
    JOINTS[2]: MOTORS[3],
    JOINTS[3]: MOTORS[4],
    JOINTS[4]: MOTORS[5],
    JOINTS[5]: MOTORS[6],
    JOINTS[6]: MOTORS[7],
}
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
# Before publishing, the data (angle or speed depending on speed mode)
# will pass through this function
shaping_topic_com: Dict[URDFJointName, Callable[[float], float]] = {
    "leg3_joint1": lambda x: np.clip(x + TC_OFFSET, a_min=TC_LOWER, a_max=TC_UPPER)
    * TC_GAIN,
    "leg3_joint2": lambda x: np.clip(x + TC_OFFSET, a_min=TC_LOWER, a_max=TC_UPPER)
    * TC_GAIN,
    "leg3_joint3": lambda x: np.clip(x + TC_OFFSET, a_min=TC_LOWER, a_max=TC_UPPER)
    * TC_GAIN,
    "leg3_joint4": lambda x: np.clip(x + TC_OFFSET, a_min=TC_LOWER, a_max=TC_UPPER)
    * TC_GAIN,
    "leg3_joint5": lambda x: np.clip(x + TC_OFFSET, a_min=TC_LOWER, a_max=TC_UPPER)
    * TC_GAIN,
    "leg3_joint6": lambda x: np.clip(x + TC_OFFSET, a_min=TC_LOWER, a_max=TC_UPPER)
    * TC_GAIN,
    "leg3_steering_joint": lambda x: np.clip(
        x + TC_OFFSET, a_min=TC_LOWER, a_max=TC_UPPER
    )
    * TC_GAIN,
    "leg3_joint8": lambda x: np.clip(x + TC_OFFSET, a_min=TC_LOWER, a_max=TC_UPPER)
    * TC_GAIN,
    "leg3_joint9": lambda x: np.clip(x + TC_OFFSET, a_min=TC_LOWER, a_max=TC_UPPER)
    * TC_GAIN,
}
#    /\    #
#   /  \   #
# Topic Command

# Topic Sensor (not supported yet)
#   \  /   #
#    \/    #
SENSORS: List[SensorTopicName] = [
    f"/maxon/canopen_motor/base_link{n}_joint_velocity_controller/state" for n in range(8)
]
TS_OFFSET: float = 0
TS_GAIN: float = 0.01
remap_topic_sens: Dict[SensorTopicName, URDFJointName] = {}
# Right after reception, the angle will pass through this function
shaping_topic_sens: Dict[SensorTopicName, Callable[[float], float]] = {}
#    /\    #
#   /  \   #
# Topic Sensor

# JointState Command
#   \  /   #
#    \/    #

# Joints with those name in the URDF will see their name change
# before publication onto /joint_commands
remap_com: Dict[URDFJointName, CommandJointName] = {
    "This_joint_does_not_exist_in_the_URDF": "This_line_will_do_nothing",
    "This_line_will_do_nothing": "This_name_is_not_in_JointState",
    # "leg3_joint1": f"leg3_joint2",
    # "leg3_joint2": f"leg3_joint1",
}
C_OFFSET: float = 0.0
C_UPPER: float = np.inf
C_LOWER: float = -np.inf
C_GAIN: float = 1
# Before publishing, the data (angle or speed depending on speed mode)
# will pass through this function
shaping_com: Dict[URDFJointName, Callable[[float], float]] = {
    JOINTS[0]: lambda x: np.clip(x + C_OFFSET, a_min=C_LOWER, a_max=C_UPPER) * C_GAIN,
    JOINTS[1]: lambda x: np.clip(x + C_OFFSET, a_min=C_LOWER, a_max=C_UPPER) * C_GAIN,
    JOINTS[2]: lambda x: np.clip(x + C_OFFSET, a_min=C_LOWER, a_max=C_UPPER) * C_GAIN,
    JOINTS[3]: lambda x: np.clip(x + C_OFFSET, a_min=C_LOWER, a_max=C_UPPER) * C_GAIN,
    JOINTS[4]: lambda x: np.clip(x + C_OFFSET, a_min=C_LOWER, a_max=C_UPPER) * C_GAIN,
    JOINTS[5]: lambda x: np.clip(x + C_OFFSET, a_min=C_LOWER, a_max=C_UPPER) * C_GAIN,
    JOINTS[6]: lambda x: np.clip(x + C_OFFSET, a_min=C_LOWER, a_max=C_UPPER) * C_GAIN,
}   
#    /\    #
#   /  \   #
# JointState Command

# JointState Sensor
#   \  /   #
#    \/    #
# Joints with those name in the /joint_states (sensor) topic
# will have their name changed when received by the joint node
# Hopefully the new name is the name of a joint in the URDF
remap_sens: Dict[SensorJointName, URDFJointName] = {
    "This_line_will_do_nothing": "This_joint_does_not_exist_in_the_URDF",
    "This_name_is_not_in_JointState": "This_line_will_do_nothing",
    # "base_link2_joint": JOINTS[0],
    # "base_link3_joint": JOINTS[1],
    # "base_link4_joint": JOINTS[2],
    # "base_link5_joint": JOINTS[3],
    # "base_link6_joint": JOINTS[4],
    # "base_link7_joint": JOINTS[5],
    # "base_link8_joint": JOINTS[6],
    # "this is a bug": 2,
}
# start_raw: int = 0
# end_raw: int = 7700
# measured_raw: int = end_raw - start_raw
# measured_rad: float = np.pi * 2 / 6
# raw2rad = measured_rad / measured_raw

start_raw: int = 2721
end_raw: int = 8456
measured_raw: int = end_raw - start_raw
measured_rad: float = np.pi * 2 / 8
raw2rad = measured_rad / measured_raw

S_OFFSET: float = 0.0
S_GAIN: float = raw2rad / 1
# Right after reception, the angle
# will pass through this function
shaping_sens: Dict[SensorTopicName, Callable[[float], float]] = {
    "base_link1_joint": lambda x: (x * S_GAIN + S_OFFSET),
    "base_link2_joint": lambda x: (x * S_GAIN + S_OFFSET),
    "base_link3_joint": lambda x: (x * S_GAIN + S_OFFSET),
    "base_link4_joint": lambda x: (x * S_GAIN + S_OFFSET),
    "base_link5_joint": lambda x: (x * S_GAIN + S_OFFSET),
    "base_link6_joint": lambda x: (x * S_GAIN + S_OFFSET),
    "base_link7_joint": lambda x: (x * S_GAIN + S_OFFSET),
    "base_link8_joint": lambda x: (x * S_GAIN + S_OFFSET),
    "base_link9_joint": lambda x: (x * S_GAIN + S_OFFSET),

    # JOINTS[0]: lambda x: (x * 1 + S_OFFSET),
    # JOINTS[1]: lambda x: (x * 1 - np.pi/2),
    # JOINTS[2]: lambda x: (x * 1 - np.pi/2),
    # JOINTS[3]: lambda x: (x * 1 - np.pi/2),
    # JOINTS[4]: lambda x: (x * 1 - np.pi/2),
    # JOINTS[5]: lambda x: (x * 1 - np.pi/2),
    # JOINTS[6]: lambda x: (x * 1 + S_OFFSET),
}
#    /\    #
#   /  \   #
# Topic Output


def is_valid_ros2_name(name: str) -> bool:
    from re import match

    assert len(name) > 0
    pattern = r"^~?[a-zA-Z_/][a-zA-Z0-9_/{}]*$"
    match_result = match(pattern, name)
    assert match_result, f"'{name}' does not match the expected pattern '{pattern}'"
    assert not name.endswith("/")
    assert not "//" in name or "__" in name
    return True


def run_shaping(f: Callable[[float], float]) -> bool:
    input = np.linspace(-np.pi, np.pi, 10, dtype=float)
    for x in input:
        out = f(float(x))
        assert isinstance(out, float), f"Value {[out]} is not a float"
        assert np.isfinite(out), f"{[out]} is a float but not a number"
    return True


dicts_to_test = [remap_sens, remap_com, remap_topic_sens, remap_topic_com]
params = []
for dic_index, dic in enumerate(dicts_to_test):
    for key, value in dic.items():
        params.append((dic_index, key, value))


@pytest.mark.parametrize("dic_index, key, value", params)
def test_remap(dic_index, key, value):
    assert isinstance(key, str), f"bad value type in entry ['{key}': {value}]"
    assert isinstance(value, str), f"bad value type in entry ['{key}':  {value}]"
    if dic_index == 3:
        is_valid_ros2_name(value)
    if dic_index == 2:
        is_valid_ros2_name(key)


dicts_to_test = [shaping_sens, shaping_com, shaping_topic_sens, shaping_topic_com]
params = []
for dic_index, dic in enumerate(dicts_to_test):
    for key, value in dic.items():
        params.append((dic_index, key, value))


@pytest.mark.parametrize("dic_index, key, value", params)
def test_shape(dic_index, key, value):
    assert isinstance(key, str)
    assert run_shaping(value)


# def run_pytest():
#     # Run pytest programmatically
#     result = pytest.main()
#     return result
#
#     # Check the result, 0 means success (all tests passed)
#     if result == 0:
#         print("All tests passed!")
#     else:
#         print(f"Some tests failed. Pytest exit code: {result}")
#
# if __name__ == "__main__":
#     run_pytest()
