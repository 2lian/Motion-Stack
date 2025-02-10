from glob import glob

from setuptools import find_packages, setup

package_name = "moonbot_zero_tuto"

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
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lvl1 = moonbot_zero_tuto.lvl1:main",
            "high_level = moonbot_zero_tuto.high_level:main",
        ],
    },
)
