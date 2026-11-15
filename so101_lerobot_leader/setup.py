from setuptools import setup
from glob import glob
import os

package_name = 'so101_lerobot_leader'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmed',
    maintainer_email='ahmed@example.com',
    description='SO-101 LeRobot leader ROS 2 publisher and GUI',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'leader_gui = so101_lerobot_leader.gui:main',
        'leader_audio_player = so101_lerobot_leader.camera_viewer:audio_main',
    ]},
)
