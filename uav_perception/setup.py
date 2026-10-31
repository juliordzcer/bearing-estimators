# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
from setuptools import find_packages
from setuptools import setup

package_name = 'uav_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/uav_perception']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='Julio Cesar Rodriguez',
    maintainer_email='contact@juliordz.com',
    description='ArUco-based bearing detection and estimation nodes for UAV formation control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_bearing_node = uav_perception.get_bearing_node:main',
        ],
    },
)
