from setuptools import find_packages, setup

package_name = "keyboard_event"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/icons",
            [
                "icons/gogo.bmp",
                "icons/gogo_happy.bmp",
                "icons/gogo_happy2.bmp",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="elian",
    maintainer_email="elian.neppel@posteo.eu",
    description="TODO: Package description",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": ["keyboard = keyboard_event.keyboard:main"],
    },
)
