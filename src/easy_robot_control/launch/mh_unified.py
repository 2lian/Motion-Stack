from time import sleep
from typing import Any, Dict, Iterable, Optional
from typing import List, Union
from os import environ
import dataclasses
from default_params import RVIZ_REMAP


@dataclasses.dataclass
class LaunchOptions:
    name: str
    leg_index: Optional[List[int]]
    lvl_to_launch: List[int]


ALL = "ALL"  # Debugg PC, we run Rviz and everything
NOTH = "NOTHING"  # Debugg PC, we run nothing form the stack
BASE = "BASE"  # Command station, will run 4-5
LEG1 = "1"  # is a leg so we run 1-2-3
LEG2 = "2"
LEG3 = "3"
LEG4 = "4"
WHE1 = "11"  # is a wheel so we run 1
WHE2 = "12"
WHE3 = "13"
WHE4 = "14"

CASES = {
    ALL: LaunchOptions(name=ALL, leg_index=None, lvl_to_launch=[1, 2, 3, 4, 5]),
    NOTH: LaunchOptions(name=NOTH, leg_index=None, lvl_to_launch=[]),
    BASE: LaunchOptions(name=BASE, leg_index=None, lvl_to_launch=[4, 5]),
    LEG1: LaunchOptions(name=LEG1, leg_index=[1], lvl_to_launch=[1, 2, 3]),
    LEG2: LaunchOptions(name=LEG2, leg_index=[2], lvl_to_launch=[1, 2, 3]),
    LEG3: LaunchOptions(name=LEG3, leg_index=[3], lvl_to_launch=[1, 2, 3]),
    LEG4: LaunchOptions(name=LEG4, leg_index=[4], lvl_to_launch=[1, 2, 3]),
    WHE1: LaunchOptions(name=WHE1, leg_index=[11], lvl_to_launch=[1]),
    WHE2: LaunchOptions(name=WHE2, leg_index=[12], lvl_to_launch=[1]),
    WHE3: LaunchOptions(name=WHE3, leg_index=[13], lvl_to_launch=[1]),
    WHE4: LaunchOptions(name=WHE4, leg_index=[14], lvl_to_launch=[1]),
}

MOONBOT_PC_NUMBER = str(environ.get("M_LEG"))  # leg number saved on lattepanda
if MOONBOT_PC_NUMBER in [None, "", "none", "None", "ALL"]:
    MOONBOT_PC_NUMBER = ALL

USE_RVIZ = False
uservizvar = str(environ.get("USE_RVIZ"))  # Should we use rviz envirnoment setting
if uservizvar == "TRUE":
    USE_RVIZ = True

CASE = CASES[MOONBOT_PC_NUMBER]

remaplvl1 = []
if USE_RVIZ:
    remaplvl1 = RVIZ_REMAP


def get_LEG_EE(legs_dic: Dict[int, Union[str, int]]) -> List[Union[str, int]]:
    """
    Args:
        legs_dic: dict of leg number associated with end effector

    Returns:
        list of end effector that should be used based on the env variable
    """
    if CASE.name in [ALL, BASE, NOTH]: #awful code
        leg_ee_out = legs_dic.values()
    elif CASE.name == LEG1:
        leg_ee_out = [legs_dic[int(LEG1)]]
    elif CASE.name == LEG2:
        leg_ee_out = [legs_dic[int(LEG2)]]
    elif CASE.name == LEG3:
        leg_ee_out = [legs_dic[int(LEG3)]]
    elif CASE.name == LEG4:
        leg_ee_out = [legs_dic[int(LEG4)]]
    elif CASE.name == WHE1:
        leg_ee_out = [legs_dic[int(WHE1)]]  # TODO
    elif CASE.name == WHE2:
        leg_ee_out = [legs_dic[int(WHE2)]]  # TODO
    elif CASE.name == WHE3:
        leg_ee_out = [legs_dic[int(WHE3)]]  # TODO
    elif CASE.name == WHE4:
        leg_ee_out = [legs_dic[int(WHE4)]]  # TODO
    else:
        print("wrong case")
        raise Exception(f"wrong case {CASE.name}")
    return list(leg_ee_out)


def get_LEG_IND(legs_dic: Dict[int, Union[str, int]]) -> List[int]:
    """
    Args:
        legs_dic: dict of leg number associated with end effector

    Returns:
        list of leg number that should be used based on the env variable
    """
    if CASE.leg_index is None:
        leg_ind_out = legs_dic.keys()
    else:
        leg_ind_out = CASE.leg_index
    return list(leg_ind_out)


def change_param(param: Dict) -> Dict:
    """changes the default param based on the env variables"""
    param = param.copy()
    if CASE.name in [ALL, NOTH]:
        param["speed_mode"] = False
    else:
        param["speed_mode"] = True

    if USE_RVIZ:
        param["pure_topic_remap"] = False
        param["speed_mode"] = False
    else:
        param["pure_topic_remap"] = True
    return param


print(f"Launch case detected: {CASE.name}")
