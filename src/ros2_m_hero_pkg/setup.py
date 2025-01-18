from glob import glob

from setuptools import find_packages, setup

package_name = "ros2_m_hero_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (
            f"share/{package_name}/{package_name}/launch",
            glob(f"{package_name}/launch/*.py"),
        ),
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
            "lvl1 = ros2_m_hero_pkg.lvl1:main",
            "calibration = ros2_m_hero_pkg.calibration:main",
            "mocap_to_arm_wheel_tf = ros2_m_hero_pkg.mocap_to_arm_wheel_tf:main",
            "align_arm_to_wheel = ros2_m_hero_pkg.align_arm_to_wheel:main",
        ],
    },
)
