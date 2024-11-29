from setuptools import setup
from glob import glob

package_name = "rviz_basic"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Elian_NEPPEL",
    maintainer_email="neppel.elian.s6@dc.tohoku.ac.jp",
    description="launch rviz and its interfaces",
    license="MIT",
    # set the shortcuts to run an executable.py, more specifically function of it
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"rviz_interface = {package_name}.rviz_interface:main",
        ],
    },
)
