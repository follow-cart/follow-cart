from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'follow_cart'

data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:

        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)

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
    data_files=package_files(data_files, ['launch/', 'rviz/', 'maps/', 'config/', 'urdf/', 'meshes/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bluevery8',
    maintainer_email='bluevery8@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'convoy_controller=follow_cart.convoy_controller:main',
            'fc1_controller=follow_cart.fc1_controller:main',
            'fc2_controller=follow_cart.fc2_controller:main',
            'fc3_controller=follow_cart.fc3_controller:main',
            'fc1_goal_updater=follow_cart.fc1_goal_updater:main',
            'fc2_goal_updater=follow_cart.fc2_goal_updater:main',
            'fc3_goal_updater=follow_cart.fc3_goal_updater:main',
            'convoy_collision_monitor=follow_cart.convoy_collision_monitor:main',
            'fc1_collision_monitor=follow_cart.fc1_collision_monitor:main',
            'fc2_collision_monitor=follow_cart.fc2_collision_monitor:main',
            'fc2_collision_monitor=follow_cart.fc2_collision_monitor:main',
            'pedestrian_controller=follow_cart.pedestrian_controller:main',
            'pedestrian_detect_processor=follow_cart.pedestrian_detect_processor:main',
            'pedestrian_detector=follow_cart.pedestrian_detector:main',
            'pedestrian_follower=follow_cart.pedestrian_follower:main'
        ],
    },
)
