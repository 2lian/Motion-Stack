from setuptools import setup
import os
from glob import glob
pACKAGE_NAME = 'rviz_basic'

setup(
    name=pACKAGE_NAME,
    version='0.0.0',
    packages=[pACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + pACKAGE_NAME]),
        ('share/' + pACKAGE_NAME, ['package.xml']),
        (f'share/{pACKAGE_NAME}/launch', glob('launch/*.py')),
        (os.path.join('share', pACKAGE_NAME), glob('urdf/*.urdf')),
        # (os.path.join('share', package_name), glob('urdf/*/*.urdf')),
        (os.path.join('share', pACKAGE_NAME), glob('meshes/moonbot7/*')),
        # (os.path.join('share', package_name, "meshes"), glob('meshes/moonbot2/*')),
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
            f'rviz_interface = {pACKAGE_NAME}.rviz_interface:main',

        ],
    },
)
