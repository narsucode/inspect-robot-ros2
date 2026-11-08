from glob import glob
import os

from setuptools import setup

package_name = 'roscar_patrol'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='机房设备状态智能巡检',
    license='MIT',
    entry_points={
        'console_scripts': [
            'patrol_node = roscar_patrol.patrol_node:main',
            'patrol_gui = roscar_patrol.patrol_gui:main',
            'aim_marker = roscar_patrol.aim_marker:main',
        ],
    },
)
