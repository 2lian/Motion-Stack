from glob import glob

from setuptools import find_packages, setup

try:
    from sphinx.setup_command import BuildDoc
except ImportError:
    BuildDoc = None
    pass


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
        "pytest==6.2.5",
        "numpy>1.20",
        "nptyping",
        "xacro",
        "numpy-quaternion",
        "scipy",
        "roboticstoolbox-python",
    ],
    zip_safe=True,
    maintainer="Elian_NEPPEL",
    maintainer_email="neppel.elian.s6@dc.tohoku.ac.jp",
    description="Control modular walking robots or robotic arms with ease.",
    # long_description=open("../../README.md").read(),
    long_description_content_type="text/markdown",
    license="MIT",
    tests_require=["pytest==6.2.5"], # deprecated field
    extras_require={
        "dev": [
            "pytest==6.2.5",
            "sphinx",
            "sphinx-rtd-theme",
            "sphinx-markdown-builder",
            "sphinx-toolbox",
            "sphinx-copybutton",
        ],
    },
    cmdclass={"build_sphinx": BuildDoc},
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
            f"joint_state_publisher = {package_name}.lazy_joint_state_publisher:main",
            f"{__name__} = {package_name}.lazy_joint_state_publisher:main",
        ],
    },
)
