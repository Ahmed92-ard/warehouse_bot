from glob import glob
from setuptools import setup
setup(
    name='warehouse_bot', version='0.1.0', packages=['warehouse_bot'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/warehouse_bot']),
                ('share/warehouse_bot', ['package.xml']),
                ('share/warehouse_bot/config', glob('config/*.yaml')),
                ('share/warehouse_bot/launch', glob('launch/*.py')),
                ('share/warehouse_bot/urdf', glob('urdf/*')),
                ('share/warehouse_bot/maps', glob('maps/*'))],
    install_requires=['setuptools'], zip_safe=True,
    entry_points={'console_scripts': [
        'esp32_bridge = warehouse_bot.esp32_bridge:main',
        'scan_to_pointcloud = warehouse_bot.scan_to_pointcloud:main',
        'mars_keyboard_teleop = warehouse_bot.keyboard_teleop:main',
        'warehouse_task_bridge = warehouse_bot.warehouse_task_bridge:main',
    ]},
)
