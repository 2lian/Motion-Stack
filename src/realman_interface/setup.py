import os
from glob import glob

from setuptools import setup

package_name = "realman_interface"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Shamistan KARIMOV",
    maintainer_email="ahresgg@gmail.com",
    description="Interface node for RealMan-75 robotic arm",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "interface_node = realman_interface.interface_node:main",
        ],
    },
)
