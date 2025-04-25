from setuptools import setup
import os
from glob import glob

package_name = 'can_bus'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unicarrier',
    maintainer_email='your.email@example.com',
    description='CAN bus middleware for forklift control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_bus_node = can_bus.can_bus_ros_node:main',
        ],
    },
) 