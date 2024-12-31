""""""

import importlib.util
import shutil
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from glob import glob
from os import path
from time import time
from typing import Callable, Dict, Iterable, List, Sequence, Set, Union

import doit
from doit import create_after
from doit.action import CmdAction
from doit.task import clean_targets
from doit.tools import Interactive

SYMLINK = True
VALID_ROS = {"humble", "foxy"}
WITH_DOCSTRING = ["easy_robot_control", "motion_stack"]
API_DIR = "./docs/source/api"
LAST_TEST_RESULT = "log/test_result-short.out"


def ros_distro():
    files: List[str] = glob("/opt/ros/*")
    roses = {(f.split("/")[-1]) for f in files}
    the_ros = roses & VALID_ROS
    if len(the_ros) != 1:
        raise ImportError(
            f"ROS2 distro could not be deduced, found: {the_ros}, valids are: {VALID_ROS}"
        )
    return the_ros.pop()


ros = ros_distro()
print(f"DETECTED ROS: {ros}")
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
            self.output_stamp(self.name),
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


def task_sub_build():
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


def task_build():
    raw_task = {
        "actions": None,
        "title": lambda task: f"Build all",
        "targets": ["./install/setup.sh"],
        "file_dep": [tar for pkg in src_pkg.values() for tar in pkg.targets],
        "clean": remove_dir(["install", "log", "build"]),
    }
    return raw_task


def task_install_piptool():
    module = importlib.util.find_spec("piptools")
    is_installed = module is not None
    return {"actions": ["pip install pip-tools"], "uptodate": [is_installed]}


def task_compile_req():
    req = "src/easy_robot_control/.requirements-dev.txt"
    return {
        "actions": [
            f"python3 -m piptools compile --extra dev -o {req} src/easy_robot_control/setup.py"
        ],
        "task_dep": ["install_piptool"],
        "targets": [req],
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
    }


def task_rosdep_init():
    return {
        "actions": [Interactive(f"{ros_src_cmd}sudo rosdep init")],
        "targets": [f"/etc/ros/rosdep/sources.list.d"],
        "verbosity": 2,
        "uptodate": [path.exists(f"/etc/ros/rosdep/sources.list.d")],
    }


def task_rosdep():
    foxy = (
        [
            Interactive(
                f"{ros_src_cmd}sudo apt install ros-foxy-xacro ros-foxy-joint-state-publisher"
            )
        ]
        if ros_distro == "foxy"
        else []
    )
    return {
        "actions": foxy
        + [
            f"{ros_src_cmd}rosdep update",
            f"{ros_src_cmd}rosdep install --from-paths src --ignore-src -r",
        ],
        # "file_dep": [f"/etc/ros/rosdep/sources.list.d"],
        "task_dep": ["rosdep_init"],
        "verbosity": 2,
        "uptodate": [f"{ros_src_cmd}rosdep check --from-paths src --ignore-src -r"],
    }


def task_docstring_linking():
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
        ],
        "file_dep": [f"{API_DIR}/{pkg_name}/.doit.stamp" for pkg_name in WITH_DOCSTRING]
        + docs_src_files,
        "clean": remove_dir([build]),
        "verbosity": 0,
    }


def task_html_doc():
    build = "./docs/build"
    return {
        "actions": [
            f"{ws_src_cmd}sphinx-build -M html ./docs/source/ {build} -j 8",
        ],
        "targets": [f"{build}/html/.buildinfo"],
        "file_dep": [f"{API_DIR}/{pkg_name}/.doit.stamp" for pkg_name in WITH_DOCSTRING]
        + docs_src_files,
        "clean": remove_dir([build]),
        "verbosity": 0,
    }


def task_main_readme():
    prefix = "docs/build/md/markdown"
    linebreak = r"\ "
    line1 = r"Access the documentation at: [https://motion-stack.deditoolbox.fr/](https://motion-stack.deditoolbox.fr/). (user is \`srl-tohoku\` and password is the one usually used by moonshot). "
    line2 = r"To build the documentation yourself, refer to the install section."
    # line1 = r"Clone, then open the full html documentation in your browser : \`./docs/build/html/index.html\`"
    return {
        "actions": [
            f"cp ./docs/build/markdown/index.md README.md",
            rf"""sed -i "s|\[\([^]]*\)\](\([^)]*\.md.*\))|[\1]({prefix}\2)|g" "README.md" """,
            rf"""sed -i '1s|^|<!-- This file is auto-generated from the docs. refere to ./docs/source/manual/README.rst -->\n|' README.md""",
            rf"""sed -i "/^# Guides:$/a {line2}" README.md """,
            rf"""sed -i "/^# Guides:$/a {line1}" README.md """,
            rf"""sed -i "/^# Guides:$/a {linebreak}" README.md """,
        ],
        "targets": [f"./README.md", "./dodo.py"],
        "file_dep": [f"./docs/build/md/markdown/index.md"],
        "verbosity": 1,
    }


def task_test_import():
    for pkg in src_pkg.values():
        test_path = f"{pkg.src}/test/test_imports.py"
        if not path.isfile(test_path):
            continue
        log_file = f"{pkg.src}/test/.test_imports.result.txt"
        # print(
        #     [f for f in glob(f"install/{pkg.name}/**") if path.isfile(f)]
        #     + [f for f in glob(f"build/{pkg.name}/**") if path.isfile(f)]
        # )
        yield {
            "name": pkg.name,
            "actions": [
                Interactive(
                    f"{ws_src_cmd}python3 -m pytest -q --log-file {log_file} --log-file-level=INFO {test_path}"
                ),
            ],
            "targets": [log_file],
            "file_dep": list(pkg.code_dep) + pkg.targets,
            "verbosity": 2,
            "clean": True,
            # "uptodate": [False],
        }


def task_test():
    out = LAST_TEST_RESULT
    return {
        "actions": [
            Interactive(
                rf"{ros_src_cmd}colcon test --packages-select motion_stack easy_robot_control ros2_m_hero_pkg rviz_basic --event-handlers console_cohesion+ || true"
            ),
            CmdAction(
                rf"{ws_src_cmd}(colcon test-result --verbose > {out} || true)",
            ),
        ],
        "task_dep": ["build"],
        "file_dep": [f for f in glob("src/**", recursive=True) if path.isfile(f)],
        "target": out,
        "verbosity": 2,
    }


def remove_dir(dirs: List[str]) -> List[Callable]:
    out: List[Callable] = []
    for d in dirs:
        if path.exists(d):
            # cmd = (CmdAction(fr"rm -r {d}"))
            out.append(lambda x=d: shutil.rmtree(x) if path.exists(d) else None)
    return out
