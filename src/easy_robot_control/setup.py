import os
from setuptools import setup
from glob import glob
from sys import version_info

package_name = "easy_robot_control"
python_version = f"python{version_info.major}.{version_info.minor}"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (f"lib/{python_version}/site-packages", glob(f"{package_name}/EliaNode.py")),
        (
            f"lib/{python_version}/site-packages/python_package_include",
            glob(f"{package_name}/python_package_include/*.*"),
        ),
        (
            f"lib/{python_version}/site-packages/{package_name}/python_package_include",
            glob(f"{package_name}/python_package_include/*.*"),
        ),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"lib/{python_version}/site-packages", glob("launch/*.py")),
        ("share/" + package_name, glob("*.npy")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
        ("share/" + package_name, glob("meshes/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Elian_NEPPEL",
    maintainer_email="neppel.elian.s6@dc.tohoku.ac.jp",
    description="launch rviz and my interface to mirror joint angle and properly \
            display joint speed",
    license="MIT",
    tests_require=['pytest'],
    # set the shortcuts to run an executable.py, more specifically function of it
    entry_points={
        "console_scripts": [
            f"limit_go_node = {package_name}.limit_go:main",
            f"joint_node = {package_name}.joint_state_interface:main",
            f"ik_node = {package_name}.ik_node:main",
            f"ik_heavy_node = {package_name}.ik_heavy_node:main",
            f"leg_node = {package_name}.leg_node:main",
            f"mover_node = {package_name}.mover_node:main",
            f"gait_node = {package_name}.gait_node:main",
        ],
    },
)
