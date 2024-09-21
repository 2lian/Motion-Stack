from setuptools import setup
from glob import glob

package_name = 'pcl_reader'

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
        (f'share/{package_name}/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elian_NEPPEL',
    maintainer_email='neppel.elian.s6@dc.tohoku.ac.jp',
    description='publishes pointcloud to rviz from file',
    license="MIT",
    entry_points={
        'console_scripts': [
            f'pointcloud_read_pub = {package_name}.pointcloud_read_pub:main'
        ],
    },
)
