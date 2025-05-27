from setuptools import setup
import os
from glob import glob

package_name = 'livox_lidar'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='user@todo.todo',
    description='ROS 2 package for Livox LiDAR processing',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'livox_lidar_node = livox_lidar.livox_lidar_node:main',
            'setup_livox = livox_lidar.setup_livox:main',
        ],
    },
) 