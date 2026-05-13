from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'brick_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    package_data={
        package_name: ['ui/*.ui'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hari Mahadevan',
    maintainer_email='hari.mahadevan@student.uts.edu.au',
    description='PyQt5 GUI for the LeBrick n Place pick-and-place system',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # ros2 run brick_gui brick_gui_node
            'brick_gui_node = brick_gui.gui_node:main',
        ],
    },
)
