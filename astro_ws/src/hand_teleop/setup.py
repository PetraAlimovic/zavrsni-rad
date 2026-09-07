from setuptools import setup
import os
from glob import glob

package_name = 'hand_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'hand_capture_node = hand_teleop.hand_capture_node:main',
            'hand_teleop_node  = hand_teleop.hand_teleop_node:main',
        ],
    },
)