import importlib.util
from glob import glob
from os import path
from typing import List, Set

from doit.task import clean_targets

SYMLINK = True


def ros_distro():
    files: List[str] = glob("/opt/ros/*")
    roses = {(f.split("/")[-1]) for f in files}
    the_ros = roses & {"humble", "foxy", "jazzy"}
    if len(the_ros) != 1:
        raise ImportError(f"ROS2 distro could not be deduced, found : {the_ros}")
    return the_ros.pop()


ros = ros_distro()
print(f"DETECTED ROS: {ros}")


def task_install_piptool():
    module = importlib.util.find_spec("piptools")
    is_installed = module is not None
    return {"actions": ["pip install pip-tools"], "uptodate": [is_installed]}


def task_compile_req():

    return {
        "actions": [
            f"pip-compile --extra dev -o src/easy_robot_control/requirements-dev.txt src/easy_robot_control/setup.py"
        ],
        "task_dep": ["install_piptool"],
        "targets": ["src/easy_robot_control/requirements-dev.txt"],
        "file_dep": ["src/easy_robot_control/setup.py"],
        "doc": "installs piptools",
        "clean": [
            clean_targets,
        ]
        + remove_dir(["src/easy_robot_controleeasy_robot_control.egg-info/"]),
        "verbosity": 0,
    }


def touch_stamp(targets):
    for t in targets:
        with open(t, "a") as output:
            output.write("doit")


def task_install_pydep():
    req = "src/easy_robot_control/requirements-dev.txt"
    tar = "./src/easy_robot_control/easy_robot_control.egg-info/.stamp.dev"
    return {
        "actions": [
            f"""CXXFLAGS="-fno-fat-lto-objects --param ggc-min-expand=10 --param ggc-min-heapsize=2048" pip install -r {req} --force-reinstall --upgrade && touch {tar}"""
        ],
        "file_dep": [req],
        "targets": [tar],
        "clean": True,
        "verbosity": 2,
    }


def remove_dir(dir: List[str]) -> List[str]:
    return [f"rm -r {d}" for d in dir if path.exists(d)]


def task_colcon_build():
    xml_files = glob("src/*/package.xml")
    packages_names = {(f.split("/")[-2]) for f in xml_files}
    pypak: Set[str] = {
        f[: -len(r"/__init__.py")] for f in glob(r"./src/**/__init__.py", recursive=True)
    }
    linked_pyfiles = {pyfile for dir in pypak for pyfile in glob(f"{dir}/*")}
    source_files = set(glob(r"src/**", recursive=True))
    garbage = set(glob(r"src/**/*cache*", recursive=True))
    # print(pyfiles)
    # print(garbage)
    builded_files = source_files - garbage - linked_pyfiles
    builded_files = {f for f in builded_files if path.isfile(f)}
    return {
        "actions": [
            f". /opt/ros/{ros}/setup.sh && colcon build --symlink-install --cmake-args -Wno-dev"
        ],
        # "task_dep": ["install_pydep"],
        "file_dep": list(builded_files),
        "targets": ["./install/setup.bash"],
        "clean": remove_dir(["build/", "log/", "install/"]),
        "verbosity": 2,
    }
