from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giordano-basso',
    maintainer_email='giordano-basso@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'px4_msgs_to_ros2 = controller.px4_msgs_to_ros2:main',
            'point_cloud_node = controller.point_cloud:main',
            'map_node = controller.occupancy_map:main',
            'drone_mission_client = controller.drone_mission_client:main',
            'drone_mission_server = controller.drone_mission_server:main',
        ],
    },
)
