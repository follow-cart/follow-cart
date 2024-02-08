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
    data_files=package_files(data_files, ['launch/', 'worlds/', 'models/', 'rviz/', 'params/', 'maps/', 'config/', 'urdf/', 'meshes/']),
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    #     (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    #     (os.path.join('share', package_name, 'worlds'), glob('worlds/**')),
    #     (os.path.join('share', package_name, 'models'), glob('models/**')),
    #     (os.path.join('share', package_name, 'rviz'), glob('rviz/**')),
    #     (os.path.join('share', package_name, 'params'), glob('params/**')),
    #     (os.path.join('share', package_name, 'maps'), glob('maps/**')),
    #     (os.path.join('share', package_name, 'config'), glob('config/**')),
    #     (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/**')),
    # ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bluevery8',
    maintainer_email='bluevery8@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'spawner = gazebo_ros.spawn_entity:main',
            'convoy_controller = follow_cart.convoy_controller:main'
        ],
    },
)
