from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_description'


def package_files(directory):
    data_files = []
    for path, _, files in os.walk(directory):
        if files:
            data_files.append((
                os.path.join('share', package_name, path),
                [os.path.join(path, filename) for filename in files]
            ))
    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ] + package_files('urdf') + package_files('meshes') + package_files('config') + package_files('worlds'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyx',
    maintainer_email='hyx23@mails.tsinghua.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'default_joint_state_publisher = robot_description.default_joint_state_publisher:main',
            'virtual_coupler_force_node = robot_description.virtual_coupler_force_node:main',
        ],
    },
)
