from glob import glob

from setuptools import find_packages, setup

package_name = "motion_stack"

VALID_ROS = {"humble", "foxy", "jazzy"}


def get_ros_distro():
    files = glob("/opt/ros/*")
    roses = {(f.split("/")[-1]) for f in files}
    the_ros = roses & VALID_ROS
    if len(the_ros) != 1:
        raise ImportError(
            f"ROS2 distro could not be deduced, found: {the_ros}, valids are: {VALID_ROS}"
        )
    return the_ros.pop()


ros = get_ros_distro()

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob("offset*.csv")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (
            f"share/{package_name}/{package_name}/launch",
            glob(f"{package_name}/api/launch/*.py"),
        ),
    ],
    install_requires=[
        # "setuptools==58.2.0",  # necessary for jazzy venv install
        "pytest==6.2.5" if ros != "jazzy" else "pytest",
        # "colcon-core",  # necessary for jazzy venv install
        # "lark",  # necessary for jazzy venv install
        # "catkin_pkg",  # necessary for jazzy venv install
        # "colcon-common-extensions",  # necessary for jazzy venv install
        "numpy>1.20",
        "nptyping",
        "xacro",
        "numpy-quaternion",
        "scipy",
        "spatialmath-python[ros-humble]",
        "roboticstoolbox-python",
        "urwid",
    ],
    zip_safe=True,
    maintainer="Elian_NEPPEL",
    maintainer_email="neppel.elian.s6@dc.tohoku.ac.jp",
    description="Control modular walking robots or robotic arms with ease.",
    # long_description=open("../../README.md").read(),
    long_description_content_type="text/markdown",
    license="MIT",
    tests_require=["pytest==6.2.5"],  # deprecated field
    extras_require={
        "dev": [
            "pytest==6.2.5" if ros != "jazzy" else "pytest",
            "sphinx",
            "myst_parser",
            "sphinx-rtd-theme",
            "sphinx-markdown-builder",
            "sphinx-toolbox",
            "sphinx-copybutton",
            "sphinx_design",
            "doit",
        ],
    },
    # set the shortcuts to run an executable.py, more specifically function of it
    entry_points={
        "console_scripts": [
            f"lvl1 = {package_name}.ros2.default_node.lvl1:main",
            f"lvl2 = {package_name}.ros2.default_node.lvl2:main",
            f"high_level_louis = {package_name}.high_level_louis:main",
            f"lazy_joint_state_publisher = {package_name}.ros2.utils.lazy_joint_state_publisher:main",
            f"trial = {package_name}.ros2.default_node.trial:main",
        ],
    },
)
