import os
from glob import glob
from setuptools import setup

package_name = 'simple_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('model/*.sdf')),
        (os.path.join('share', package_name), glob('model/*.urdf')),
        (os.path.join('share', package_name), glob('model/*.xacro')),
        (os.path.join('share', package_name), glob('world/*.sdf')),
        (os.path.join('share', package_name), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Minsik Oh',
    maintainer_email=',michael1015999@gmail.com',
    description='Multi Robot Spawn and Control // Original repository:https://gitlab.ensta-bretagne.fr/zerrbe/ign-ros2-multi-robots-control-update.git',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_test = simple_test.simple_test:main'
        ],
    },
)
