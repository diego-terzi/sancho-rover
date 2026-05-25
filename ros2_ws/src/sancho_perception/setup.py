import glob
from setuptools import find_packages, setup
package_name = 'sancho_perception'

# Installa i modelli ONNX presenti in models/ nella share directory del pacchetto.
# I file .onnx sono gitignored e vanno copiati manualmente prima di colcon build.
_model_files = glob.glob('models/*.onnx')

_data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]
if _model_files:
    _data_files.append(('share/' + package_name + '/models', _model_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=_data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.com',
    description='Perception nodes for SANCHO',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_node = sancho_perception.camera_node:main',
            'sim_node = sancho_perception.sim_node:main',
            'sensor_node = sancho_perception.sensor_node:main',
        ],
    },
)
