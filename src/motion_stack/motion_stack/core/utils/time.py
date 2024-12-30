from typing import Final

NANOSEC: Final[int] = int(1e9)


class Time(int):
    def __new__(cls, value=None, *, sec=None, nano=None):
        if sec is not None and nano is not None:
            raise ValueError("Specify either 'sec' or 'nano', not both.")
        if sec is not None:
            value = int(sec * NANOSEC)
        elif nano is not None:
            value = int(nano)
        elif value is None:
            value = 0  # Default to 0 if no arguments are provided
        return super().__new__(cls, value)

    def nano(self):
        """Return the time as nanoseconds."""
        return int(self)

    def sec(self):
        """Return the time as whole seconds."""
        return self.nano() // NANOSEC

    def secf(self):
        """Return the time as fractional seconds."""
        return float(self / NANOSEC)

    def __add__(self, other):
        return Time(super().__add__(other))

    def __sub__(self, other):
        return Time(super().__sub__(other))

    def __mul__(self, other):
        return Time(super().__mul__(other))

    def __floordiv__(self, other):
        return Time(super().__floordiv__(other))

    def __truediv__(self, other):
        return Time(super().__truediv__(other))

    def __mod__(self, other):
        return Time(super().__mod__(other))
