from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'autorace_control'

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
    description='autorace_control',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'control_node = autorace_control.control_node:main',
        ],
    },
)
