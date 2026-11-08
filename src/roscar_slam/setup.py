from setuptools import setup
from glob import glob
import os

package_name = 'roscar_slam'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='narsu',
    maintainer_email='narsu@example.com',
    description='YoudaoBot SLAM',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)
