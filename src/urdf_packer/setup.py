from setuptools import find_packages, setup
from setuptools import Command, setup
import os
from glob import glob

package_name = "urdf_packer"

dirUrdf = glob(os.path.join("urdf", "*"), recursive=False)
dirMeshes = glob(os.path.join("meshes", "*"), recursive=False)

urdffolder_copy_list = [
    (os.path.join("share", package_name, folder), glob(os.path.join(folder, "*")))
    for folder in dirUrdf
]

meshfolder_copy_list = [
    (os.path.join("share", package_name, folder), glob(os.path.join(folder, "*")))
    for folder in dirMeshes
]

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ]
    + urdffolder_copy_list
    + meshfolder_copy_list,
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
