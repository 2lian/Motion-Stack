from glob import glob

from setuptools import find_packages, setup

package_name = "motion_stack_tuto"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/urdf", glob("urdf/*", recursive=True)),
        (f"share/{package_name}/meshes", glob("meshes/*", recursive=True)),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="elian",
    maintainer_email="elian.neppel@posteo.eu",
    description="TODO: Package description",
    license="MIT",
    entry_points={
        "console_scripts": [
            "lvl1 = motion_stack_tuto.lvl1:main",
            "high_level = motion_stack_tuto.high_level:main",
            "high_dbg = motion_stack_tuto.high_lvl_debug:main",
            "lvl1_dyna = motion_stack_tuto.lvl1_dynamixel:main",
        ],
    },
)
