from setuptools import setup
# This is to import params from launchfiles
from glob import glob
package_name = 'easy_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/python3.10/site-packages/python_package_include',
         glob(f'{package_name}/python_package_include/*')),
        ('share/' + package_name, glob('*.npy')),
        ('share/' + package_name, glob('urdf/*')),
        ('share/' + package_name, glob('meshes/*'))
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
            f'ik_node = {package_name}.ik_node:main',
            f'leg_node = {package_name}.leg_node:main',
            f'mover_node = {package_name}.mover_node:main',

        ],
    },
)
