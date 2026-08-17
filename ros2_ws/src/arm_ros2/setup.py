import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'arm_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Koutilya Ganapathiraju',
    maintainer_email='gkoutilyaraju@gmail.com',
    description='ROS2 node graph wrapping the vision-guided pick-and-place PyBullet simulation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_node = arm_ros2.sim_node:main',
            'perception_node = arm_ros2.perception_node:main',
            'planner_node = arm_ros2.planner_node:main',
            'task_node = arm_ros2.task_node:main',
        ],
    },
)
