import json
from dataclasses import asdict
from typing import Any, Callable, Dict, List, Optional

import zenoh

from ...core.utils.joint_state import JState
from ...core.utils.time import Time
from .general import Encoded, Method

JSTATE_JSON = "JStateJSON"

def to_json(js: JState) -> Encoded:
    """Human readable, good for debugging"""
    dic = asdict(js)
    del dic["name"]
    Encoded(data= json.dumps(dic, indent=1), encoding=JSTATE_JSON)


def from_json(data: bytes) -> JState:
    """Human readable, good for debugging"""
    json_listdic: Dict = json.loads(data)
    return JState(
        name="NOT_TRANSMITTED",
        time=Time(json_listdic["time"]),
        position=json_listdic["position"],
        velocity=json_listdic["velocity"],
        effort=json_listdic["effort"],
    )


JSON = Method(to_json, from_json, JSTATE_JSON)

DEFAULT = JSON
