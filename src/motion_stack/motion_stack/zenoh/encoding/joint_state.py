import json
from dataclasses import asdict
from typing import Any, Callable, Dict, List, Optional

import zenoh

from ...core.utils.joint_state import JState
from ...core.utils.time import Time
from .general import Encoded, Method

JSTATE_JSON = "ms_js_json"


def to_json(js: JState) -> Encoded:
    """Human readable, good for debugging"""
    dic = asdict(js)
    return Encoded(payload=json.dumps(dic, indent=1), encoding=JSTATE_JSON)


def from_json(data: Encoded) -> JState:
    """Human readable, good for debugging"""
    assert JSTATE_JSON == data.encoding
    json_listdic: Dict = json.loads(data.payload)
    return JState(
        name=json_listdic["name"],
        time=Time(json_listdic["time"]) if json_listdic["time"] is not None else None,
        position=json_listdic["position"],
        velocity=json_listdic["velocity"],
        effort=json_listdic["effort"],
    )


#: Human readable, and self explanatory method, good for debugging
JSON = Method(to_json, from_json, JSTATE_JSON)

DEFAULT = JSON
