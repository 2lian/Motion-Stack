from setuptools import find_packages, setup

package_name = "ms_operator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "urwid",  # TUI
    ],
    zip_safe=True,
    author="Shamistan_KARIMOV",
    author_email="karimov.shamistan.p8@dc.tohoku.ac.jp",
    description="OperatorNode + TUI for Motion-Stack",
    license="MIT",
    entry_points={
        "console_scripts": [
            f"operator = {package_name}.operator_node:main",
        ],
    },
)
