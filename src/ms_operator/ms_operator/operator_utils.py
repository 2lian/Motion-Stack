import dataclasses
from dataclasses import dataclass
from typing import Any, Callable, Dict, Final, List, Literal, Optional, Tuple, Union

import numpy as np
import quaternion as qt
from keyboard_msgs.msg import Key
from numpy.typing import NDArray
from sensor_msgs.msg import Joy

from motion_stack.core.utils.pose import Pose

# type def V
ANY: Final[str] = "ANY"
ALWAYS: Final[str] = "ALWAYS"
KeyCodeModifier = Tuple[
    int, Union[int, Literal["ANY"]]
]  # keyboard input: key + modifier
JoyBits = int  # 32 bits to represent all buttons pressed or not
ButtonName = Literal[  # type with all possible buttons
    "NONE",
    "x",
    "o",
    "t",
    "s",
    "L1",
    "R1",
    "L2",
    "R2",
    "share",
    "option",
    "PS",
    "stickLpush",
    "stickRpush",
    "down",
    "right",
    "up",
    "left",
    "stickL",
    "stickR",
]
JoyCodeModifier = Tuple[
    ButtonName, Union[JoyBits, Literal["ANY"]]
]  # joystick input: new press + JoyState
UserInput = Union[  # type of keys to the dict that will give functions to execute
    KeyCodeModifier,
    JoyCodeModifier,
    Literal["ALWAYS"],  # functions associated with "ALWAYS" string will always execute
]
NakedCall = Callable[[], Any]
InputMap = Dict[
    UserInput, List[NakedCall]
]  # User input are linked to a list of function
# ───────────────────────────── JoyStick ─────────────────────────────

# Keys
NOMOD = Key.MODIFIER_NUM

BUTT_BITS: Dict[ButtonName, int] = {  # button name to bit position
    # butts
    "x": 0,
    "o": 1,
    "t": 2,
    "s": 3,
    # backs
    "L1": 4,
    "R1": 5,
    "L2": 6,
    "R2": 7,
    # options
    "share": 8,
    "option": 9,
    "PS": 10,
    # stick pushed
    "stickLpush": 11,
    "stickRpush": 12,
    # dpad
    "down": 13,
    "right": 14,
    "up": 15,
    "left": 16,
    # sticks
    "stickL": 17,  # left stick not centered
    "stickR": 18,  # right stick not centered
}
BITS_BUTT: Dict[int, ButtonName] = {v: k for k, v in BUTT_BITS.items()}
# Bitmask of each button
BUTT_INTS: Dict[ButtonName, JoyBits] = {
    butt: 1 << bit for butt, bit in BUTT_BITS.items()
}
BUTT_INTS["NONE"] = 0
INTS_BUTT: Dict[JoyBits, ButtonName] = {v: k for k, v in BUTT_INTS.items()}


@dataclass
class JoyState:
    bits: JoyBits = 0
    stickR: NDArray = dataclasses.field(
        default_factory=lambda: np.zeros(2, dtype=float)
    )
    stickL: NDArray = dataclasses.field(
        default_factory=lambda: np.zeros(2, dtype=float)
    )
    R2: float = 0.0
    L2: float = 0.0


def msg_to_JoyBits(msg: Joy) -> JoyState:
    """Converts a joy msg to a JoyState"""
    state = JoyState()
    but: List[int] = msg.buttons  # type:ignore
    axes: List[float] = msg.axes  # type:ignore
    sticks_raw: List[float] = [axes[x] for x in [0, 1, 3, 4]]  # type:ignore
    # triggers are already included in the but list
    triggers: List[float] = [axes[x] for x in [2, 5]]  # type:ignore
    dpad_raw: List[float] = axes[-2:]  # type:ignore
    bfield = int(0)
    # buttons
    i = 0
    for i, b in enumerate(but):
        assert b == 1 or b == 0
        bfield = bfield | (b << i)
    # dpad
    next_bit_to_set = i + 1
    dpad = [
        dpad_raw[1] < -0.5,
        dpad_raw[0] < -0.5,
        dpad_raw[1] > 0.5,
        dpad_raw[0] > 0.5,
    ]
    for i, ax in enumerate(dpad):
        i += next_bit_to_set
        ax_active = not np.isclose(ax, 0)
        bfield = bfield | (ax_active << i)
    # sticks
    next_bit_to_set = i + 1
    for i, ax in enumerate(sticks_raw):
        if ax == -0.0:
            sticks_raw[i] = 0.0

    state.stickL = np.array(sticks_raw[:2], dtype=float)
    state.stickL[[0, 1]] = state.stickL[[1, 0]]  # changes xy
    ax_active = not np.isclose(np.linalg.norm(state.stickL), 0, atol=0.2)
    bfield = bfield | (ax_active << BUTT_BITS["stickL"])  # using * is bad

    state.stickR = np.array(sticks_raw[2:], dtype=float)
    state.stickR[[0, 1]] = state.stickR[[1, 0]]  # changes xy
    ax_active = not np.isclose(np.linalg.norm(state.stickR), 0, atol=0.2)
    bfield = bfield | (ax_active << BUTT_BITS["stickR"])  # using * is bad

    state.L2 = (1 - triggers[0]) / 2
    state.R2 = (1 - triggers[1]) / 2

    state.bits = bfield

    return state


def any_pressed(
    bits: JoyBits, button_names: Union[List[ButtonName], ButtonName]
) -> bool:
    """Checks if any button in the list is pressed.

    Args:
        bits: set of joybits to check against
        button_names: list of button names to check if True

    Returns:
        True if any bit corresponding to a button is True.
    """
    if isinstance(button_names, str):
        button_names = [button_names]
    for n in button_names:
        assert n in BUTT_INTS.keys(), f"Button {n} does not exist"

    arr_of_joybits = np.array([BUTT_INTS[n] for n in button_names])
    fused_joybits = np.bitwise_or.reduce(arr_of_joybits)
    return (bits & fused_joybits) != 0


def bits2name(bits: JoyBits) -> List[ButtonName]:
    """Converts a bit field to a list of button names"""
    button_names: List[ButtonName] = []
    while bits:  # handles several button pressed at the same time
        # to handle rare edge cases
        isolated_bit = bits & -bits
        name = one_bit2name(isolated_bit)
        if name is not None:
            button_names.append(name)
        bits &= bits - 1
    return button_names


def one_bit2name(bits: JoyBits) -> Optional[ButtonName]:
    """Converts a bit field with 1 bit to 1, to a single button name"""
    button_name: Optional[ButtonName] = INTS_BUTT.get(bits)
    button_name = None if button_name == "NONE" else button_name
    return button_name


def collapseT_KeyCodeModifier(variable: Any) -> Optional[KeyCodeModifier]:
    """Collapses the variable onto a KeyCodeModifier type, or None

    Returns:
        None if variable is not a KCM
        The variable as a KCM type-hint if it is a KCM

    """
    if not isinstance(variable, tuple):
        return None
    if len(variable) != 2:
        return None
    if not isinstance(variable[0], int):
        return None
    if isinstance(variable[1], int):
        return variable
    if variable[1] == ANY:
        return variable
    return None


def collapseT_JoyCodeModifier(variable: Any) -> Optional[JoyCodeModifier]:
    """Collapses the variable onto a JoyCodeModifier type, or None

    Returns:
        None if variable is not a JCM
        The variable as a JCM type-hint if it is a JCM

    """
    if not isinstance(variable, tuple):
        return None
    if len(variable) != 2:
        return None
    if not isinstance(variable[0], str):
        return None
    if isinstance(variable[1], JoyBits):
        return variable
    if variable[0] == ANY:
        return variable
    return None


def remap_onto_any(mapping: InputMap, input: UserInput):
    """runs the input through the INPUTMap as if the key_modifier was any
    if it is already, it does not run it.
    """
    collapsed_KCM = collapseT_KeyCodeModifier(input)
    if collapsed_KCM is not None:  # is KCM
        if not collapsed_KCM[1] == ANY:
            connect_mapping(mapping, (collapsed_KCM[0], ANY))

    collapsed_JCM = collapseT_JoyCodeModifier(input)
    if collapsed_JCM is not None:  # is JCM
        if not collapsed_JCM[1] == ANY:
            connect_mapping(mapping, (collapsed_JCM[0], ANY))


def connect_mapping(mapping: InputMap, input: UserInput):
    """Given the user input, executes the corresponding function mapping

    Args:
        mapping: Dict of function to execute
        input: key to the entry to execute
    """
    remap_onto_any(mapping, input)
    if input not in mapping.keys():
        return
    to_execute: List[NakedCall] = mapping[input]
    for f in to_execute:
        f()
    return


def rel_to_base_link(ik_syncer, offset):
    track = set(offset.keys())
    prev = ik_syncer._previous_point(track)
    return {
        key: Pose(
            offset[key].time,
            prev[key].xyz + qt.rotate_vectors(qt.one, offset[key].xyz),
            offset[key].quat * prev[key].quat,
        )
        for key in track
    }
