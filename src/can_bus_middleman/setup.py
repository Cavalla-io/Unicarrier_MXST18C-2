#!/usr/bin/env python3
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'can_bus_middleman'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # This tells ament where to find the package.xml and a resource marker.
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Optionally include launch files if you have any
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Victor Boyd',
    maintainer_email='victor.w.boyd@gmail.com',
    description='CAN Bus Middleman',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            
        ],
    },
)
