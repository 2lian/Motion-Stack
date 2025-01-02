""""""

import importlib.util
import shutil
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from glob import glob
from os import path
from time import time
from typing import Callable, Dict, Iterable, List, Sequence, Set, Union

from doit.action import CmdAction
from doit.task import clean_targets
from doit.tools import Interactive, check_timestamp_unchanged

SYMLINK = True
VALID_ROS = {"humble", "foxy"}
WITH_DOCSTRING = ["easy_robot_control", "motion_stack"]
API_DIR = "./docs/source/api"
TEST_REPORT = "log/.test_report"


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
# print(f"DETECTED ROS: {ros}")
ros_src_cmd = f". /opt/ros/{ros}/setup.sh && "
ws_src_cmd = f". ./install/setup.sh && "

docs_src_files = [f for f in glob(f"./docs/source/**", recursive=True) if path.isfile(f)]


@dataclass
class RosPackage:
    name: str
    src: str
    xml: ET.Element
    other_packages: Set[str] = field(default_factory=set)

    @property
    def pkg_depend(self) -> Set[str]:
        depend = {e.text for e in self.xml.findall("depend") if e.text is not None}
        exec_depend = {
            e.text for e in self.xml.findall("exec_depend") if e.text is not None
        }
        test_depend = {
            e.text for e in self.xml.findall("test_depend") if e.text is not None
        }
        all_dep = depend | exec_depend | test_depend
        return all_dep & self.other_packages

    @property
    def version(self) -> Set[str]:
        return self.xml.find("version").text

    @property
    def targets(self) -> List[str]:
        return [
            self.output_setup(self.name),
            # self.output_stamp(self.name),
        ]

    @property
    def code_dep(self) -> Set[str]:
        source_files = set(glob(rf"{self.src}/**", recursive=True))
        garbage = set(glob(rf"{self.src}/**/*cache*", recursive=True)) | set(
            glob(rf"{self.src}/**/*cache*/**", recursive=True)
        )
        pkg_src_files = source_files - garbage
        pkg_src_files = {f for f in pkg_src_files if path.isfile(f)}
        return pkg_src_files

    @property
    def build_dep(self) -> Set[str]:
        pypak_dir: Set[str] = {
            f[: -len(r"/__init__.py")]
            for f in glob(rf"{self.src}/**/__init__.py", recursive=True)
        }
        linked_pyfiles = {pyfile for dir in pypak_dir for pyfile in glob(f"{dir}/*")}
        builded_src_files = self.code_dep - linked_pyfiles
        builded_src_files = {f for f in builded_src_files if path.isfile(f)}
        # print(builded_src_files)

        other_packages_output = {
            self.output_setup(dep_pkg) for dep_pkg in self.pkg_depend
        } | {self.output_stamp(dep_pkg) for dep_pkg in self.pkg_depend}
        return other_packages_output | builded_src_files

    @property
    def is_python(self) -> bool:
        return self.xml.find("export").find("build_type").text == "ament_python"

    @staticmethod
    def output_setup(name) -> str:
        return f"install/{name}/share/{name}/package.sh"

    @staticmethod
    def output_stamp(name) -> str:
        # I don't know why doit doesn't detect colcon properly
        return f"build/{name}/doit.stamp"

    @property
    def cleans(self) -> Sequence[Union[str, Callable]]:
        return remove_dir(
            [f"build/{self.name}", f"log/{self.name}", f"install/{self.name}"]
        )


def extract_package_data(xml_path: str) -> RosPackage:
    xml_root = ET.parse(xml_path).getroot()
    pkg_path = xml_path[: -len("/package.xml")]
    out = RosPackage(
        name=xml_root.find("name").text,
        src=pkg_path,
        xml=xml_root,
    )
    return out


def packages(path) -> Dict[str, RosPackage]:
    xml_files = set(glob(f"{path}/**/package.xml", recursive=True))
    pk_dict = {}
    for f in xml_files:
        p = extract_package_data(f)
        pk_dict[p.name] = p
    for pk in pk_dict.values():
        pk.other_packages = set(pk_dict.keys())
    return pk_dict


src_pkg = packages("src")


def task_build():
    yield {
        "name": None,
        "actions": None,
        "targets": ["./install/setup.sh"],
        "file_dep": [tar for pkg in src_pkg.values() for tar in pkg.targets],
        "uptodate": [
            check_timestamp_unchanged(tar)
            for pkg in src_pkg.values()
            for tar in pkg.targets
        ],
        "clean": remove_dir(["install", "log", "build"]),
        "doc": "Colcon builds packages",
    }
    for name, pkg in src_pkg.items():
        raw_task = {
            "name": f"{pkg.name}",
            # "title": lambda task, name=name: f"Build {name}",
            "actions": [
                f"{ros_src_cmd}colcon build --packages-select {name} "
                f"--symlink-install --cmake-args -Wno-dev &&"
                f"echo 'build time: {time()}' >> ./build/{name}/doit.stamp"
            ],
            "targets": pkg.targets,
            "file_dep": list(pkg.build_dep),
            "task_dep": ["rosdep"],
            "clean": pkg.cleans,
        }
        if raw_task["file_dep"] == []:
            del raw_task["file_dep"]
        yield raw_task


def task_pipcompile():
    req = "src/easy_robot_control/.requirements-dev.txt"
    module = importlib.util.find_spec("piptools")
    is_installed = module is not None
    yield {
        "name": None,
        "actions": None,
        "doc": "Compiles pyhton requirements",
    }
    yield {
        "name": "install-pip-tools",
        "actions": ["python3 -m pip install pip-tools"],
        "uptodate": [is_installed],
    }
    yield {
        "name": "pip-compile",
        "task_dep": ["pipcompile:install-pip-tools"],
        "actions": [
            f"python3 -m piptools compile --extra dev -o {req} src/easy_robot_control/setup.py"
        ],
        # "task_dep": ["install_piptool"],
        "targets": [req],
        "file_dep": ["src/easy_robot_control/setup.py"],
        "clean": [
            clean_targets,
        ]
        + remove_dir(["src/easy_robot_controleeasy_robot_control.egg-info/"]),
        "verbosity": 0,
    }


def task_pydep_soft():
    req = "src/easy_robot_control/.requirements-dev.txt"
    tar = "./src/easy_robot_control/easy_robot_control.egg-info/.stamp.soft"
    return {
        "basename": "pydep-soft",
        "actions": [
            Interactive(
                f"""CXXFLAGS="-fno-fat-lto-objects --param ggc-min-expand=10 --param ggc-min-heapsize=2048" pip install -r {req} && touch {tar}"""
            )
        ],
        "file_dep": [req],
        "targets": [tar],
        "clean": True,
        "verbosity": 2,
        "doc": "Install python dependencies (not garanteed to work)",
    }


def task_pydep_hard():
    req = "src/easy_robot_control/.requirements-dev.txt"
    tar = "./src/easy_robot_control/easy_robot_control.egg-info/.stamp.hard"
    return {
        "basename": "pydep-hard",
        "actions": [
            Interactive(
                f"""CXXFLAGS="-fno-fat-lto-objects --param ggc-min-expand=10 --param ggc-min-heapsize=2048" pip install -r {req} --force-reinstall --upgrade && touch {tar}"""
            )
        ],
        "file_dep": [req],
        "targets": [tar],
        "clean": True,
        "verbosity": 2,
        "doc": "Install python dependencies using --force-reinstall --upgrade",
    }


def task_rosdep():
    check = f"{ros_src_cmd}rosdep check --from-paths src --ignore-src -r"

    missing_rosdep = (
        ["ros-foxy-xacro", "ros-foxy-joint-state-publisher"] if ros == "foxy" else []
    )

    yield {
        "name": None,
        "actions": None,
        "doc": "Install ROS dependencies",
    }
    yield {
        "name": "init",
        "actions": [
            Interactive(f"{ros_src_cmd}sudo rosdep init"),
        ],
        "verbosity": 2,
        "uptodate": [path.exists(f"/etc/ros/rosdep/sources.list.d")],
    }
    for apt_pkg in missing_rosdep:
        yield {
            "name": apt_pkg,
            "actions": [Interactive(f"{ros_src_cmd}sudo apt install {apt_pkg}")],
            "verbosity": 2,
            "uptodate": [f"dpkg -s {apt_pkg}"],
        }
    yield {
        "name": "update",
        "actions": [
            f"{ros_src_cmd}rosdep update --rosdistro {ros}",
        ],
        "task_dep": ["rosdep:init"],
        "verbosity": 2,
        "uptodate": [check],
    }
    yield {
        "name": "install",
        "actions": [
            f"{ros_src_cmd}rosdep install --from-paths src --ignore-src -r",
        ],
        "task_dep": ["rosdep:init", "rosdep:update"]
        + [f"rosdep:{apt_pkg}" for apt_pkg in missing_rosdep],
        "verbosity": 2,
        "uptodate": [check],
    }


def task__docgen():
    for pkg_name in WITH_DOCSTRING:
        yield {
            "name": pkg_name,
            "actions": [
                f"mkdir -p {API_DIR}/{pkg_name}",
                f"{ws_src_cmd}sphinx-apidoc -M -f -d 2 -o {API_DIR}/{pkg_name} "
                f"src/{pkg_name}/{pkg_name}",
                f"echo 'build time: {time()}' >> {API_DIR}/{pkg_name}/.doit.stamp",
            ],
            "targets": [
                f"{API_DIR}/{pkg_name}/modules.rst",
                f"{API_DIR}/{pkg_name}/.doit.stamp",
            ],
            "file_dep": src_pkg[pkg_name].targets + list(src_pkg[pkg_name].code_dep),
            "clean": remove_dir([f"{API_DIR}/{pkg_name}"]),
        }


def task_md_doc():
    build = "./docs/build/md"
    return {
        "actions": [
            f"{ws_src_cmd}sphinx-build -M markdown ./docs/source/ {build} -j 8",
        ],
        "targets": [
            f"{build}/markdown/index.md",
            f"{build}/markdown/manual/use.md",
        ],
        "file_dep": [f"{API_DIR}/{pkg_name}/.doit.stamp" for pkg_name in WITH_DOCSTRING]
        + docs_src_files,
        "clean": remove_dir([build]),
        "verbosity": 0,
        "doc": f"Builds the documentation as markdown in {build}",
    }


def task_html_doc():
    """adds the test badge compared to non-dev"""
    build = "docs/build"
    return {
        "actions": [
            f"{ws_src_cmd}sphinx-build -M html ./docs/source/ {build} -q",
        ],
        "targets": [f"{build}/html/.buildinfo"],
        "file_dep": [f"{API_DIR}/{pkg_name}/.doit.stamp" for pkg_name in WITH_DOCSTRING]
        + docs_src_files
        + ["./docs/source/media/test_badge.rst"],
        "clean": remove_dir([build]),
        "verbosity": 0,
        "doc": f"Builds the documentation as html in {build}/html",
    }


def task_fix_md():
    prefix = "docs/build/md/markdown/"
    media_path = "docs/source/"
    linebreak = r"\ "
    line1 = r"Access the documentation at: [https://motion-stack.deditoolbox.fr/](https://motion-stack.deditoolbox.fr/). (user is \`srl-tohoku\` and password is the one usually used by moonshot). "
    line2 = r"To build the documentation yourself, refer to the install section."
    # line1 = r"Clone, then open the full html documentation in your browser : \`./docs/build/html/index.html\`"
    yield {
            "name": "main_readme",
        "actions": [
            f"cp {prefix}/index.md README.md",
            rf"""sed -i "s|\[\([^]]*\)\](\([^)]*\.md.*\))|[\1]({prefix}\2)|g" "README.md" """,
            # rf"""sed -i "s|\(media/\([^)]*\)\)|{media_path}\1|g" "README.md" """,
            rf"""sed -i '1s|^|<!-- This file is auto-generated from the docs. refere to ./docs/source/manual/README.rst -->\n|' README.md""",
            rf"""sed -i "/^# Guides:$/a {line2}" README.md """,
            rf"""sed -i "/^# Guides:$/a {line1}" README.md """,
            rf"""sed -i "/^# Guides:$/a {linebreak}" README.md """,
        ],
        "targets": [f"./README.md"],
        "file_dep": [f"./docs/build/md/markdown/index.md", "./dodo.py"],
        "verbosity": 1,
        "doc": "Creates ./README.md from the documentation",
    }
    for file in ["./docs/build/md/markdown/manual/use.md"]:
        yield {
            "name": f"media_{file}",
            "actions": [
                rf"""sed -i "s|](media/|]({media_path}media/|g" "{file}" """,
            ],
            "file_dep": [f"{file}", "dodo.py"],
            "verbosity": 1,
            "doc": "Creates ./README.md from the documentation",
        }


def task_test_import():
    yield {
        "name": None,
        "actions": None,
        "doc": "Fast sanity check -- Tests all python file executability",
    }
    for pkg in src_pkg.values():
        test_path = f"{pkg.src}/test/test_imports.py"
        if not path.isfile(test_path):
            continue
        log_file = f"{pkg.src}/test/.test_imports.result.txt"
        yield {
            "name": pkg.name,
            "actions": [
                Interactive(
                    f"{ws_src_cmd}python3 -m pytest -q --log-file {log_file} --log-file-level=INFO {test_path}"
                ),
            ],
            "targets": [log_file],
            "file_dep": list(pkg.code_dep) + pkg.targets,
            "uptodate": [check_timestamp_unchanged(t, "modify") for t in pkg.targets],
            "verbosity": 2,
            "clean": True,
        }


def task_test():
    out = TEST_REPORT
    return {
        "actions": [
            Interactive(
                rf"{ros_src_cmd}colcon test --packages-select motion_stack easy_robot_control ros2_m_hero_pkg rviz_basic --event-handlers console_cohesion+ || true"
            ),
            CmdAction(
                rf"{ws_src_cmd}colcon test-result --verbose > {TEST_REPORT} || true"
            ),
        ],
        "task_dep": ["build"],
        "file_dep": [f for pkg in src_pkg.values() for f in pkg.code_dep],
        "targets": [out],
        "verbosity": 2,
        "doc": "Runs all test, using colcon test",
    }


def task_ci_badge():

    def check(targets):
        with open(TEST_REPORT, "r") as file:
            lines = file.readlines()
        test_failed = len(lines) != 1
        if test_failed:
            src = "./docs/source/media/test_fail.rst"
        else:
            src = "./docs/source/media/test_success.rst"
        for tar in targets:
            shutil.copyfile(src, tar)
        return True

    return {
        "actions": [check],
        "file_dep": [TEST_REPORT],
        "targets": ["./docs/source/media/test_badge.rst"],
        "verbosity": 2,
        "clean": True,
        "doc": "Copies fail/success.rst badge depending on last test result",
    }


def remove_dir(dirs: List[str]) -> List[Callable]:
    out: List[Callable] = []
    for d in dirs:
        if path.exists(d):
            # cmd = (CmdAction(fr"rm -r {d}"))
            out.append(lambda x=d: shutil.rmtree(x) if path.exists(d) else None)
    return out
