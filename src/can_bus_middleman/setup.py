from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'can_bus_middleman'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f'{package_name}.nodes'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files if they exist
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py') if os.path.isdir('launch') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='CAN Bus Middleman for interfacing with CAN bus devices',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add the entry point for middleman_node.py
            'middleman_node = can_bus_middleman.nodes.middleman_node:main',
        ],
    },
)