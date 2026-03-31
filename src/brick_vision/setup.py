from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'brick_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy',
        'pyrealsense2',
    ],
    zip_safe=True,
    maintainer='b',
    maintainer_email='benjamin.j.costarella@student.uts.edu.au',
    description='LEGO brick detection using Intel RealSense and OpenCV',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # ros2 run brick_vision brick_detector
            "brick_detector = brick_vision.brick_detector:main",
        ],
    },
)
