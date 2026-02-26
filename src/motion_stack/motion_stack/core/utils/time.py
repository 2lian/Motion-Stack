from dataclasses import dataclass
from typing import Final, Optional, Union

NANOSEC: Final[int] = int(1e9)

Number = Union[int, float]


@dataclass(frozen=True, slots=True)
class Time:
    nano: int

    # --- constructors ---

    @classmethod
    def from_sec(cls, sec: Union[int, float]) -> "Time":
        if isinstance(sec, int):
            return cls(sec * NANOSEC)
        if isinstance(sec, float):
            return cls(int(sec * NANOSEC))
        else:
            raise TypeError("sec must be int or float")

    @classmethod
    def sn(cls, sec: Union[int, float] = 0, nano: int = 0) -> "Time":
        return cls.from_parts(sec, nano)

    @classmethod
    def from_parts(cls, sec: Union[int, float] = 0, nano: int = 0) -> "Time":
        return Time.from_sec(sec) + Time(nano)

    # --- accessors ---

    @property
    def sec(self) -> float:
        return self.nano / NANOSEC

    # --- arithmetic ---

    def __add__(self, other: Union["Time", int]) -> "Time":
        if isinstance(other, int):
            return Time(self.nano + other)
        if isinstance(other, Time):
            return Time(self.nano + other.nano)
        return NotImplemented

    def __sub__(self, other: Union["Time", int]) -> "Time":
        if isinstance(other, int):
            return Time(self.nano - other)
        if isinstance(other, Time):
            return Time(self.nano - other.nano)
        return NotImplemented

    def __mul__(self, other: Number) -> "Time":
        if isinstance(other, (int, float)):
            return Time(int(self.nano * other))
        return NotImplemented

    def __rmul__(self, other: Number) -> "Time":
        return self.__mul__(other)

    def __truediv__(self, other: Union["Time", Number]):
        if isinstance(other, (int, float)):
            return Time(int(self.nano / other))
        if isinstance(other, Time):
            # ratio, dimensionless
            return self.nano / other.nano
        return NotImplemented

    def __mod__(self, other: "Time"):
        if isinstance(other, Time):
            return Time(self.nano % other.nano)
        return NotImplemented

    def __floordiv__(self, other: Union["Time", int]):
        if isinstance(other, int):
            return Time(self.nano // other)
        if isinstance(other, Time):
            return self.nano // other.nano
        return NotImplemented

    def __neg__(self) -> "Time":
        return Time(-self.nano)

    def __abs__(self) -> "Time":
        return Time(abs(self.nano))

    # --- comparisons ---

    def __eq__(self, other) -> bool:
        if isinstance(other, Time):
            return self.nano == other.nano
        return NotImplemented

    def __lt__(self, other) -> bool:
        if isinstance(other, Time):
            return self.nano < other.nano
        return NotImplemented

    def __le__(self, other) -> bool:
        if isinstance(other, Time):
            return self.nano <= other.nano
        return NotImplemented
