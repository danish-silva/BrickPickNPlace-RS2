from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'brick_interaction'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hari Mahadevan',
    maintainer_email='hari.mahadevan@student.uts.edu.au',
    description='Interaction and Execution subsystem for the LeBrick n Place pick-and-place system',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # ros2 run brick_interaction brick_interaction_node
            'brick_interaction_node = brick_interaction.interaction_node:main',
            'pose_transform_test    = brick_interaction.pose_transform_test:main',
        ],
    },
)
