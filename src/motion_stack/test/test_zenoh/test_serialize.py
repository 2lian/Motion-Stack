from itertools import product
import random
import string

import pytest

import motion_stack.zenoh.encoding.joint_state as js_encoding
from motion_stack.core.utils.joint_state import JState
from motion_stack.core.utils.time import Time
from motion_stack.zenoh.encoding.general import Encodable, Encoded
from motion_stack.zenoh.encoding.serialization import parse, serialize

fields = {
    "name": "test",
    "time": Time(sec=1.4),
    "position": 1.1,
    "velocity": 1.1,
    "effort": 1.1,
}

js_combinations = []
for mask in product([None, "val"], repeat=5):
    args = {
        "name": fields["name"] if mask[4] else "",
        "time": fields["time"] if mask[0] else None,
        "position": fields["position"] if mask[1] else None,
        "velocity": fields["velocity"] if mask[2] else None,
        "effort": fields["effort"] if mask[3] else None,
    }
    js_combinations.append(JState(**args))


@pytest.mark.parametrize("js", js_combinations)
def test_jstate_json(js):
    meth = js_encoding.JSON
    encoded = meth.serializer(js)
    assert isinstance(encoded, Encoded)
    assert isinstance(encoded.payload, str)
    assert r'"name": ""' in encoded.payload or r'"name": "test"' in encoded.payload
    assert js_encoding.JSTATE_JSON == encoded.encoding
    decoded = meth.parser(encoded)
    assert isinstance(decoded, JState)
    assert js == decoded

N = 10   # number of strings
L = 30   # length of each string

alphabet = string.ascii_letters + string.digits  # you can extend this
str_combinations = ["".join(random.choices(alphabet, k=L)) for _ in range(N)]

@pytest.mark.parametrize("data", js_combinations+str_combinations)
def test_auto_serializer(data: Encodable):
    dtype = type(data)
    encoded = serialize(data)
    assert isinstance(encoded, Encoded)
    assert isinstance(encoded.payload, str)
    decoded = parse(encoded)
    assert isinstance(decoded, dtype)
    assert data == decoded
