from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_zed_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='turtle1',
    maintainer_email='turtle1@todo.todo',
    description='Turtlebot3 Burger with ZED 2i for SLAM and Navigation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_goal = turtlebot3_zed_nav.send_goal:main',
        ],
    },
)
