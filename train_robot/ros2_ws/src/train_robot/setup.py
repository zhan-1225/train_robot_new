from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'train_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aiden',
    maintainer_email='13677123828@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control = train_robot.control:main',
            'moveit_node = train_robot.moveit_node:main',
            'gazebo_control = train_robot.gazebo_control:main',
        ],
        
    },
)
