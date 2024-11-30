from glob import glob

from setuptools import find_packages, setup

package_name = "easy_robot_control"

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
            glob(f"{package_name}/launch/*.py"),
        ),
        ("share/" + package_name, glob("*.npy")),
    ],
    install_requires=[
        # "setuptools==58.2.0",
        "numpy",
        "xacro",
        "numpy-quaternion",
        "scipy",
        "pytest>6.2.4",
        "roboticstoolbox-python",
    ],
    zip_safe=True,
    maintainer="Elian_NEPPEL",
    maintainer_email="neppel.elian.s6@dc.tohoku.ac.jp",
    description="Modular walking robots or a single robotic arm, seamlessly bring your robots to life with just a URDF! Built for maximum flexibility, ease of use and source-code customization.",
    license="MIT",
    tests_require=["pytest"],
    # set the shortcuts to run an executable.py, more specifically function of it
    entry_points={
        "console_scripts": [
            f"limit_go_node = {package_name}.limit_go:main",
            f"joint_node = {package_name}.joint_state_interface:main",
            f"ik_node = {package_name}.ik_node:main",
            f"ik_heavy_node = {package_name}.ik_heavy_node:main",
            f"leg_node = {package_name}.leg_node:main",
            f"mover_node = {package_name}.mover_node:main",
            f"keygait_node = {package_name}.gait_key_dev:main",
            f"gait_node = {package_name}.gait_node:main",
        ],
    },
)
