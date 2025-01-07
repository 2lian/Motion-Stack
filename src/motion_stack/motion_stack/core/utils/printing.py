from typing import Iterable, Optional


class TCOL:
    """Colors for  the terminal"""

    HEADER = """\033[95m"""  # ]
    OKBLUE = """\033[94m"""  # ]
    OKCYAN = """\033[96m"""  # ]
    OKGREEN = """\033[92m"""  # ]
    WARNING = """\x1b[33;20m"""  # ]
    FAIL = """\033[91m"""  # ]
    ENDC = """\033[0m"""  # ]
    BOLD = """\033[1m"""  # ]
    UNDERLINE = """\033[4m"""  # ]

def list_cyanize(l: Iterable, default_color: Optional[str] = None) -> str:
    """Makes each element of a list cyan.

    Args:
        l: Iterable
        default_color: color to go back to outise of the cyan

    Returns:

    """
    if default_color is None:
        default_color = TCOL.ENDC
    out = "["
    first = True
    for k in l:
        if not first:
            out += ", "
        first = False
        if isinstance(k, str):
            out += f"'{TCOL.OKCYAN}{k}{default_color}'"
        else:
            out += f"{TCOL.OKCYAN}{k}{default_color}"
    out += "]"
    return out


