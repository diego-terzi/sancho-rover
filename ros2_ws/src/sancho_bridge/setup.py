from setuptools import find_packages, setup
package_name = 'sancho_bridge'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.com',
    description='Hardware interface for SANCHO',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'motor_bridge_node = sancho_bridge.motor_bridge_node:main',
        ],
    },
)
