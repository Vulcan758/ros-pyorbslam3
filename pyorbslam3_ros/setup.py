from glob import glob

from setuptools import find_packages, setup

package_name = 'pyorbslam3_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config/orbslam3', glob('config/orbslam3/*.yaml')),
        ('share/' + package_name + '/config/params', glob('config/params/*.yaml')),
        ('share/' + package_name + '/config/rviz', glob('config/rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    extras_require={'test': ['pytest']},
    maintainer='mahir',
    maintainer_email='mahirgored@gmail.com',
    description='ROS 2 node for ORB-SLAM3 visual and visual-inertial SLAM, built on pyorbslam3.',
    license='GPL-3.0-or-later',
    entry_points={
        'console_scripts': [
            'slam_node = pyorbslam3_ros.slam_node:main',
            'dataset_player = pyorbslam3_ros.dataset_player:main',
        ],
    },
)
