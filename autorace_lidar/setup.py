from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'autorace_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ERICAR',
    maintainer_email='ericar@todo.todo',
    description='autorace_lidar',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'lidar_node = autorace_lidar.lidar_node:main',
            'estop_node = autorace_lidar.estop_node:main',
        ],
    },
)
