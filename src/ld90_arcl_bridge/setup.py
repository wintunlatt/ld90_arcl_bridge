from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ld90_arcl_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # include all subfolders
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Win',
    maintainer_email='you@example.com',
    description='ARCL ↔ ROS2 bridge for Omron LD90 with simulation tools',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ld90_bridge = ld90_arcl_bridge.bridge_node:main',
            'fake_arcl = ld90_arcl_bridge.fake_arcl_server:main',
        ],
    },
)
