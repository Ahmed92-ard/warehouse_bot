from glob import glob
from setuptools import setup
setup(
    name='rover', version='0.1.0', packages=['rover'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/rover']),
                ('share/rover', ['package.xml']),
                ('share/rover/config', glob('config/*.yaml')),
                ('share/rover/launch', glob('launch/*.py')),
                ('share/rover/urdf', glob('urdf/*')),
                ('share/rover/maps', glob('maps/*'))],
    install_requires=['setuptools'], zip_safe=True,
    entry_points={'console_scripts': [
        'esp32_bridge = rover.esp32_bridge:main',
        'scan_to_pointcloud = rover.scan_to_pointcloud:main',
        'mars_keyboard_teleop = rover.keyboard_teleop:main',
        'warehouse_task_bridge = rover.warehouse_task_bridge:main',
    ]},
)
