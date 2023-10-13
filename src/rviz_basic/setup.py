from setuptools import setup
import os
from glob import glob
package_name = 'rviz_basic'

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
    maintainer='Elian_NEPPEL',
    maintainer_email='neppel.elian.s6@dc.tohoku.ac.jp',
    description='launch rviz and its interfaces',
    license='Apache License 2.0',
    # set the shortcuts to run an executable.py, more specifically function of it
    entry_points={
        'console_scripts': [
            f'rviz_interface = {package_name}.rviz_interface:main',

        ],
    },
)
