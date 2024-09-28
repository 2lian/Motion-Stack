import pytest
import numpy as np
from typing import Callable, Dict, List, Optional, Tuple

URDFJointName = str
CommandJointName = str
SensorJointName = str
CommandTopicName = str
SensorTopicName = str

# Topic Command
#   \  /   #
#    \/    #
MOTORS: List[CommandTopicName] = [
    f"/maxon/canopen_motor/base_link{n+1}_joint_velocity_controller/command"
    for n in range(7)
]
OFFSET: float = 0.1
UPPER: float = 0.5
LOWER: float = -0.5
GAIN: float = 10
remap_topic_com: Dict[URDFJointName, CommandTopicName] = {
    "This_joint_does_not_exist_in_the_URDF": "It_will_never_be_used",
    "leg3_joint1": MOTORS[0],
    "leg3_joint2": MOTORS[1],
    "leg3_joint3": MOTORS[2],
    "leg3_joint4": MOTORS[3],
    "leg3_joint5": MOTORS[4],
    "leg3_joint6": MOTORS[5],
}
shaping_topic_com: Dict[URDFJointName, Callable[[float], float]] = {
    "leg3_joint5": lambda x: np.clip(x + OFFSET, a_min=LOWER, a_max=UPPER) * GAIN
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
OFFSET: float = 0
GAIN: float = 0.01
remap_topic_sens: Dict[SensorTopicName, URDFJointName] = {
    SENSORS[1]: "leg3_joint5",
}
shaping_topic_sens: Dict[SensorTopicName, Callable[[float], float]] = {
    SENSORS[1]: lambda x: x * GAIN + OFFSET
}
#    /\    #
#   /  \   #
# Topic Sensor

# JointState Command
#   \  /   #
#    \/    #
OFFSET: float = 0.1
UPPER: float = 0.5
LOWER: float = -0.5
GAIN: float = 10
prefix_com = "com_"
remap_com: Dict[URDFJointName, CommandJointName] = {
    "leg3_joint5": f"base_link1_joint",
}
shaping_com: Dict[URDFJointName, Callable[[float], float]] = {
    "leg3_joint5": lambda x: np.clip(x + OFFSET, a_min=LOWER, a_max=UPPER) * GAIN
}
#    /\    #
#   /  \   #
# JointState Command

# JointState Sensor
#   \  /   #
#    \/    #
OFFSET: float = 0
GAIN: float = 0.01
prefix_sens = "sens_"
remap_sens: Dict[SensorJointName, URDFJointName] = {
    "base_link1_joint": "leg3_joint5",
}
shaping_sens: Dict[SensorTopicName, Callable[[float], float]] = {
    "base_link1_joint": lambda x: (x * GAIN + OFFSET),
    # "base_link2_joint": lambda x: np.nan,
    # "base_link3_joint": lambda x: (x * GAIN + OFFSET)/0,
}
#    /\    #
#   /  \   #
# Topic Output


def run_shaping(f: Callable[[float], float]) -> bool:
    input = np.linspace(-np.pi, np.pi, 10, dtype=float)
    for x in input:
        out = f(float(x))
        assert isinstance(out, float)
        assert np.isfinite(out)
    return True


dicts_to_test = [remap_sens, remap_com, remap_topic_sens, remap_topic_com]
params = []
for dic_index, dic in enumerate(dicts_to_test):
    for key, value in dic.items():
        params.append((dic_index, key, value))


@pytest.mark.parametrize("dic_index, key, value", params)
def test_remap(dic_index, key, value):
    assert isinstance(key, str) and isinstance(value, str)


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
