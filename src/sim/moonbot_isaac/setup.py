import os

from setuptools import find_packages, setup

package_name = 'moonbot_isaac'

# Add folders recursively to data_files
def package_files(data_files, directory_list):
    paths_dict = {}

    for directory in directory_list:
        for path, _, filenames in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join("share", package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)

                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=package_files(
        # Files to include
        [
            ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
            ("share/" + package_name, ["package.xml"]),
        ],
        # Directories to include
        [
            "launch/",
            "config/",
            "assets/",
            "environments/",
        ],
    ),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='artefacts',
    maintainer_email='azazdeaz@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'isaac_motion_stack_interface = moonbot_isaac.isaac_motion_stack_interface:main',
            'interface_alive_service = moonbot_isaac.interface_alive_service:main',
            'leg_move_test = moonbot_isaac.leg_move_test:main',
            'tf_ground_truth_republisher = moonbot_isaac.tf_ground_truth_republisher:main',
            'mocap_simulator = moonbot_isaac.mocap_simulator:main',
            'realsense_extrinsics = moonbot_isaac.realsense_extrinsics:main',
        ],
    },
)
