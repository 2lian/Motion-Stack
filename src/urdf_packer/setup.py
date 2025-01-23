from setuptools import find_packages, setup
from setuptools import Command, setup
import os
from glob import glob

package_name = "urdf_packer"


def copy2share(directory):
    paths = []
    for path, directories, filenames in os.walk(directory):
        for filename in filenames:
            file_path = os.path.join(path, filename)
            paths.append((os.path.join("share", package_name, path), [file_path]))
    return paths


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ]
    + copy2share("urdf")
    + copy2share("meshes"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="elian",
    maintainer_email="neppel.elian.s6@dc.tohoku.ac.jp",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
