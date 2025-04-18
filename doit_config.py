from glob import glob
from typing import List

from doit import get_var

# This allows you to change doit behavior without using command line variables
# VVV
OVERIDE_CONFIG = {
    #: if 'venv=y', a python virtual environment is used. This is an early feature
    #: necessary for `ros2 jazzy`, it has not been tested for foxy/humble.
    "venv": None,
    #: if 'syml=y', colcon `--use-symlink` is used, making re-building mostly unnecessary.
    "syml": None,
    #: if 'pipforce=y', often fixes python dependencies
    #: --force-reinstall --update all of the python packages to a compatible version,
    #: regardless of other installed pip dependencies.
    "pipforce": None,
    #: if 'low_mem=y', fixes pip issue on low memory systems (<1GB)
    "low_mem": None,
    #: 'pip_args=<your args>', inserts arguments to pip install.
    #: This only affects pip install of the motion stack dependencies. So, not the pip install of other utilities such as pip-tools, wheels, venv ... You can for example use pip_args="--ignore-installed" to skip already installed packages, useful if some are pinned by ros2 and causes install issues.
    "pip_args": None,
    #: 'colcon_args=<your args>', inserts arguments to colcon build commands.
    "colcon_args": None,
    #: if 'dev=y', install dependencies for devloppers (docs, tests ...).
    #: This is enabled by default
    "dev": None,
}
# ^^^

VALID_ROS = {"humble", "foxy", "jazzy"}


def get_ros_distro():
    files: List[str] = glob("/opt/ros/*")
    roses = {(f.split("/")[-1]) for f in files}
    the_ros = roses & VALID_ROS
    if len(the_ros) != 1:
        raise ImportError(
            f"ROS2 distro could not be deduced, found: {the_ros}, valids are: {VALID_ROS}"
        )
    return the_ros.pop()


ros = get_ros_distro()

if ros == "jazzy":
    default_values = {
        "venv": "y",
        "syml": "y",
        "low_mem": "n",
        "pipforce": "n",
        "pip_args": "",
        "colcon_args": "",
        "dev": "y",
    }
elif ros == "humble":
    default_values = {
        "venv": "n",
        "syml": "y",
        "low_mem": "n",
        "pipforce": "n",
        "pip_args": "",
        "colcon_args": "",
        "dev": "y",
    }
elif ros == "foxy":
    default_values = {
        "venv": "n",
        "syml": "y",
        "low_mem": "n",
        "pipforce": "n",
        "pip_args": "",
        "colcon_args": "",
        "dev": "y",
    }
else:
    raise Exception(f"Could not find config for the ros distro.")

for k, v in OVERIDE_CONFIG.items():
    if v is not None:
        default_values[k] = v

config = {
    "ros2": ros,
    "venv": get_var("venv", default_values["venv"]) == "y",
    "syml": get_var("syml", default_values["syml"]) == "y",
    "low_mem": get_var("low_mem", default_values["low_mem"]) == "y",
    "dev": get_var("dev", default_values["dev"]) == "y",
    "pipforce": get_var("pipforce", default_values["pipforce"]) == "y",
    "pip_args": get_var("pip_args", default_values["pip_args"]),
    "colcon_args": get_var("colcon_args", default_values["colcon_args"]),
}

if config["dev"]:
    print(config)
