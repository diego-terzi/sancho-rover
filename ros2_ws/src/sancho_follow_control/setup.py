from setuptools import find_packages, setup
package_name = 'sancho_follow_control'
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
    description='Controller node for SANCHO human-follow mode',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'follow_controller_node = sancho_follow_control.follow_controller_node:main',
        ],
    },
)
