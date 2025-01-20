import pytest

from motion_stack.core.utils.time import NANOSEC, Time


def test_creation():
    t = Time(5_000_000_000)
    assert isinstance(t, Time)
    assert isinstance(t.nano(), int)
    assert isinstance(t.sec(), float)
    assert t.nano() == 5_000_000_000
    assert t.sec() == 5.0

    assert Time(sec=5) == Time(nano=5_000_000_000) == t


def test_arithmetic_operations():
    t1 = Time(5_000_000_000)
    t2 = Time(2_000_000_000)

    # Subtraction
    t3 = t1 - t2
    assert isinstance(t3, Time)
    assert t3.nano() == 3_000_000_000

    # Addition
    t4 = t1 + t2
    assert isinstance(t4, Time)
    assert t4.nano() == 7_000_000_000

    # Multiplication
    t5 = (t1 * 1/3)
    assert isinstance(t5, Time)

    # Multiplication
    t5 = t1 * 2
    assert isinstance(t5, Time)
    assert t5.nano() == 10_000_000_000

    # Floor Division
    t6 = t1 // 2
    assert isinstance(t6, Time)
    assert t6.nano() == 2_500_000_000

    # True Division (should round to integer and remain Time)
    t7 = t1 / 2
    assert isinstance(t7, Time)
    assert t7.nano() == 2_500_000_000


def test_modulo_operation():
    t1 = Time(5_000_000_000)
    t2 = Time(3_000_000_000)

    t3 = t1 % t2
    assert isinstance(t3, Time)
    assert t3.nano() == 2_000_000_000


def test_type_handling():
    t1 = Time(5_000_000_000)
    t2 = t1 - 2_000_000_000
    assert isinstance(t2, Time)
    assert t2.nano() == 3_000_000_000

    t3 = t1 + 3
    assert isinstance(t3, Time)
    assert t3.nano() == 5_000_000_003


def test_comparisons():
    t1 = Time(5_000_000_000)
    t2 = Time(3_000_000_000)

    assert t1 > t2
    assert t2 < t1
    assert t1 == Time(5_000_000_000)
    assert t1 != t2


def test_edge_cases():
    t = Time(0)
    assert t.nano() == 0
    assert t.sec() == 0.0

    t_negative = Time(-1_000_000_000)
    assert t_negative.nano() == -1_000_000_000
    assert t_negative.sec() == -1.0
