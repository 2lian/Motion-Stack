from os import environ
from typing import Optional

import zenoh

#: Zenoh config to be used by default (user can overide with it own method to get the config)
ZENOH_CONFIG = zenoh.Config.from_file(environ["ZENOH_SESSION_CONFIG_URI"])
_maybe_session: Optional[zenoh.Session] = None


def auto_session() -> zenoh.Session:
    """Returns a global session or creates a new one on the first call of this function
    """
    if _maybe_session is None:
        return zenoh.open(ZENOH_CONFIG)
    else:
        return _maybe_session
