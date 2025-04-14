#!/usr/bin/env python3
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'teleop_forklift_control'

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
    description='Teleoperation control for forklift using keyboard inputs.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This creates a console executable called "main" which will invoke main() from teleop_forklift_control/scripts/main.py.
            'main = teleop_forklift_control.scripts.main:main',
            # New entry point for the wheel tracker node
            'wheel_tracker = teleop_forklift_control.scripts.wheel_tracker_node:main',
            # New entry point for the remote start node
            'remote_start = teleop_forklift_control.scripts.remote_start_node:main'
        ],
    },
)
