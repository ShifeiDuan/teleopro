import os

from setuptools import find_packages, setup

package_name = 'teleopro_core'

# colcon --symlink-install runs setup.py from the build directory
readme = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'README.md')
long_description = ''
if os.path.exists(readme):
    with open(readme, encoding='utf-8') as f:
        long_description = f.read()

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['numpy'],
    python_requires='>=3.8',
    zip_safe=True,
    maintainer='Shifei Duan',
    maintainer_email='sdua078@aucklanduni.ac.nz',
    description='Device- and robot-agnostic teleoperation mapping, no ROS required.',
    long_description=long_description,
    long_description_content_type='text/markdown',
    license='Apache-2.0',
    url='https://github.com/sdua078/teleopro',
    project_urls={
        'Source': 'https://github.com/sdua078/teleopro',
        'Issues': 'https://github.com/sdua078/teleopro/issues',
    },
    keywords=['teleoperation', 'robotics', 'ros2', 'xr', 'spacemouse'],
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'Intended Audience :: Science/Research',
        'Operating System :: OS Independent',
        'Programming Language :: Python :: 3',
        'Topic :: Scientific/Engineering',
        'Topic :: Software Development :: Libraries :: Python Modules',
    ],
    extras_require={'test': ['pytest']},
)
