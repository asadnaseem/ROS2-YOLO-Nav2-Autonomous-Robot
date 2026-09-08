from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'yolo_nav2_semantic'

setup(
    name=package_name,
    version='3.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/semantic_nav.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
        ('share/' + package_name + '/urdf', ['urdf/semantic_rover.urdf']),
        ('share/' + package_name + '/worlds', ['worlds/semantic_warehouse.world']),
        ('share/' + package_name + '/maps', ['maps/warehouse.yaml', 'maps/warehouse.pgm']),
        ('share/' + package_name + '/rviz', ['rviz/semantic_nav.rviz']),
        ('share/' + package_name + '/models', ['models/yolov8n.pt']),
        *[(os.path.join('share', package_name, os.path.dirname(path)), [path])
          for model_name in ('stop_sign',)
          for path in glob(f'models/{model_name}/**/*', recursive=True)
          if os.path.isfile(path)],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muhammad Asad Naseem',
    maintainer_email='asadnaseem10k@gmail.com',
    description='Professional Nav2 sensor fusion with YOLO semantic perception.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'semantic_obstacle_node = yolo_nav2_semantic.semantic_obstacle_node:main',
        'patrol_manager = yolo_nav2_semantic.patrol_manager:main',
    ]},
)
