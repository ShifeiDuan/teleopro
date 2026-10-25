from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'teleopro_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shifei Duan',
    maintainer_email='sdua078@aucklanduni.ac.nz',
    description='Device- and robot-agnostic teleoperation through TF.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'teleop_node = teleopro_ros.teleop_node:main',
        ],
    },
)
