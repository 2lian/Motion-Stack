from setuptools import setup
from setuptools import find_packages
# This is to import params from launchfiles
import os
from glob import glob
package_name = 'rviz_custom'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('meshes/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amc-scar-e',
    maintainer_email='amc-scar-e@todo.todo',
    description='code to control SCARE',
    license='Apache License 2.0',
    tests_require=['pytest'],
    # set the shortcuts to run an executable.py, more specifically function of it
    entry_points={
        'console_scripts': [
            f'rviz_interface = {package_name}.rviz_interface:main',
            f'ik_node = {package_name}.ik_node:main',

        ],
    },
)
