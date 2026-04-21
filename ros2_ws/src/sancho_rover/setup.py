import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sancho_rover'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='diego-terzi',
    maintainer_email='terzi.diego02@gmail.com',
    description='ROS 2 nodes for the SANCHO tracked UGV.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = sancho_rover.camera_node:main',
            'controller_node = sancho_rover.controller_node:main',
            'motor_bridge_node = sancho_rover.motor_bridge_node:main',
            'sensor_node = sancho_rover.sensor_node:main',
        ],
    },
)
