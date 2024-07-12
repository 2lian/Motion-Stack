import os
from setuptools import setup
# This is to import params from launchfiles
from glob import glob
pACKAGE_NAME = 'easy_robot_control'

setup(
    name=pACKAGE_NAME,
    version='0.0.0',
    packages=[pACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + pACKAGE_NAME]),
        ('share/' + pACKAGE_NAME, ['package.xml']),
        ('lib/python3.10/site-packages/',
         glob(f'{pACKAGE_NAME}/EliaNode.py')),
        (f'lib/python3.10/site-packages/python_package_include',
         glob(f'{pACKAGE_NAME}/python_package_include/*')),
        (f'lib/python3.10/site-packages/{pACKAGE_NAME}/python_package_include',
         glob(f'{pACKAGE_NAME}/python_package_include/*')),
        (f'share/{pACKAGE_NAME}/launch', glob('launch/*.py')),
        ('share/' + pACKAGE_NAME, glob('*.npy')),
        (os.path.join('share', pACKAGE_NAME, "urdf"), glob('urdf/*.urdf')),
        ('share/' + pACKAGE_NAME, glob('meshes/*'))
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
            f'test = {pACKAGE_NAME}.test:main',
            f'ik_node = {pACKAGE_NAME}.ik_node:main',
            f'ik_heavy_node = {pACKAGE_NAME}.ik_heavy_node:main',
            f'leg_node = {pACKAGE_NAME}.leg_node:main',
            f'mover_node = {pACKAGE_NAME}.mover_node:main',

        ],
    },
)
