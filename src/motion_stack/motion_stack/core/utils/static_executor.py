from time import time_ns

from .printing import TCOL
from .time import Time


class Spinner:
    def now(self) -> Time: ...
    def error(self) -> None: ...
    def warn(self) -> None: ...
    def info(self) -> None: ...
    def debug(self) -> None: ...


class PythonSpinner(Spinner):
    def __init__(self) -> None:
        self.__now: Time = Time(0)

    def now(self) -> Time:
        return self.__now

    def change_time(self, time: Time):
        self.__now = Time(time)

    def error(self, *args) -> None:
        print(TCOL.FAIL + str(*args) + TCOL.ENDC)

    def warn(self, *args) -> None:
        print(TCOL.WARNING + str(*args) + TCOL.ENDC)

    def info(self, *args) -> None:
        print(*args)

    def debug(self, *args) -> None:
        print(TCOL.OKBLUE + str(*args) + TCOL.ENDC)


class FlexNode:
    spinner: Spinner

    def __init__(self, spinner: Spinner):
        self.spinner = spinner

    def now(self):
        self.spinner.now()

    def error(self, *args):
        self.spinner.error(*args)

    def warn(self, *args):
        self.spinner.warn(*args)

    def info(self, *args):
        self.spinner.info(*args)

    def debug(self, *args):
        self.spinner.debug(*args)
