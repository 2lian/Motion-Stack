from setuptools import Command, setup
import os
from glob import glob

package_name = "rviz_basic"
folders = ["moonbot_hero2", "moonbot_hero", "hero_3wheel_1hand", "moonbot_7", "moonbot_45"]

# folders += [f"{name}_description" for name in folders]

urdffolder_copy_list = [
    (os.path.join("share", package_name, "urdf", folder), glob(f"urdf/{folder}/*"))
    for folder in folders
]

meshfolder_copy_list = [
    (os.path.join("share", package_name, "meshes", folder), glob(f"meshes/{folder}/*"))
    for folder in folders
]

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        # (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
        # (os.path.join("share", package_name), glob("meshes/moonbot7/*")),
        # (os.path.join('share', package_name, "meshes"), glob('meshes/moonbot2/*')),
    ] + urdffolder_copy_list + meshfolder_copy_list,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Elian_NEPPEL",
    maintainer_email="neppel.elian.s6@dc.tohoku.ac.jp",
    description="launch rviz and its interfaces",
    license="Apache License 2.0",
    # set the shortcuts to run an executable.py, more specifically function of it
    entry_points={
        "console_scripts": [
            f"rviz_interface = {package_name}.rviz_interface:main",
        ],
    },
)
