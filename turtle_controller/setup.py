import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtle_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohamed',
    maintainer_email='mohamed@todo.todo',
    description='Turtle controller package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = turtle_controller.controller_node:main',
            'reset_service_node = turtle_controller.reset_service_node:main',
            'square_action_server_node = turtle_controller.square_action_server_node:main',
        ],
    },
)
