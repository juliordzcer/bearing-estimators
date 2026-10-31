# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
from setuptools import find_packages
from setuptools import setup

package_name = 'uav_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/uav_tools']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Julio Cesar Rodriguez',
    maintainer_email='contact@juliordz.com',
    description='Operator tools for UAV formation control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bearing_logger = uav_tools.bearing_logger_plotter:main',
            'camera_viewer = uav_tools.camera_viewer:main',
            'camera_viewer_cov = uav_tools.camera_viewer_cov:main',
            'server_gui = uav_tools.server:main',
            'cce2026_gui_v2 = uav_tools.interfaz_cce2026_v2:main',
        ],
    },
)
