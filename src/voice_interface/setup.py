from setuptools import find_packages, setup

package_name = 'voice_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dheeraj',
    maintainer_email='dheeraj.panjwani@student.uts.edu.au',
    description='Voice interface for ROS2 task-level control',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [ 'voice_input_node = voice_interface.voice_input_node:main',
                             'command_parser_node = voice_interface.command_parser_node:main',
                             'system_command_listener = voice_interface.system_command_listener:main',
                             'reset_executor_node = voice_interface.reset_executor_node:main',
        ]
    },
)
